"""
motor_node.py – ROS2 node for Damiao DM-J43xx geared motors.

Topics
------
  Published:
    ~/joint_states                       sensor_msgs/JointState
    ~/motors/{name}/state                custom_interfaces/MotorState
    ~/motors/{name}/fault                custom_interfaces/MotorFault   (on change only)
  Subscribed:
    ~/motors/{name}/cmd_mit              custom_interfaces/OperationCommand
    ~/motors/{name}/cmd_position_pv      custom_interfaces/PositionPPCommand
    ~/motors/{name}/cmd_velocity         custom_interfaces/VelocityCommand
    ~/motors/{name}/cmd_force_position   custom_interfaces/PositionCSPCommand

Services
--------
  ~/enable_motor          custom_interfaces/EnableMotor
  ~/set_run_mode          custom_interfaces/SetRunMode
  ~/set_zero_position     custom_interfaces/SetZeroPosition
  ~/read_param            custom_interfaces/ReadParam
  ~/write_param           custom_interfaces/WriteParam
  ~/motor_param           custom_interfaces/MotorParam
  ~/help                  custom_interfaces/Help
  ~/stop_all              std_srvs/Trigger
  ~/clear_faults          std_srvs/Trigger
  ~/save_params           std_srvs/Trigger   (disables motor → saves → re-enables)
  ~/homing                std_srvs/Trigger

Actions
-------
  ~/move_to_position      custom_interfaces/MoveToPosition
  ~/set_velocity          custom_interfaces/SetVelocity

Parameters
----------
  config_path             str    path to config.toml

Notes
-----
  - Damiao motors use request-response CAN; feedback arrives after each command.
    The timer polls joint states at update_rate_hz using the last received feedback.
  - save_params requires the motor to be disabled (Damiao hardware constraint).
    The save_params service and write_param with persist=True handle this automatically.
  - Motor IDs must be 0–15 (4-bit limit of feedback frame encoding).
  - DEC register takes a negative value; config dec = 5.0 is written as -5.0.
"""

import importlib
import os
import time
import threading
from typing import Dict, Optional, Tuple

import tomllib

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from custom_interfaces.msg import (
    MotorFault,
    MotorState,
    OperationCommand,
    PositionCSPCommand,
    PositionPPCommand,
    VelocityCommand,
)
from custom_interfaces.srv import (
    EnableMotor,
    Help,
    MotorParam,
    ReadParam,
    SetRunMode,
    SetZeroPosition,
    WriteParam,
)
from custom_interfaces.action import MoveToPosition
from custom_interfaces.action import SetVelocity as SetVelocityAction

from .comms import DamiaoCANComms
from .motor_base import FaultCode, MotorFeedback, RegAddr, RunMode


# ── Motor class registry ───────────────────────────────────────────────────────

_MOTOR_CLASS_MAP = {
    'J4310_2EC': 'damiao_p.j4310_2ec.J4310_2EC',
}

# ── Register type table (uint32 vs float) ──────────────────────────────────────

_UINT_REGS = {
    int(RegAddr.MST_ID),
    int(RegAddr.ESC_ID),
    int(RegAddr.TIMEOUT),
    int(RegAddr.CTRL_MODE),
    int(RegAddr.NPP),
    int(RegAddr.HW_VER),
    int(RegAddr.SW_VER),
    int(RegAddr.SN),
    int(RegAddr.CAN_BR),
    int(RegAddr.SUB_VER),
    int(RegAddr.BOOT_VER),
}

# ── Parameter metadata for Help service ───────────────────────────────────────

_PARAM_INFO: Dict[int, Tuple[str, str, str]] = {
    int(RegAddr.UV_VALUE):  ('float',  'R_W', 'Undervoltage threshold (V)'),
    int(RegAddr.OT_VALUE):  ('float',  'R_W', 'Over-temperature threshold (°C) [80, 200]'),
    int(RegAddr.OC_VALUE):  ('float',  'R_W', 'Overcurrent threshold per-unit (0.0, 1.0)'),
    int(RegAddr.ACC):       ('float',  'R_W', 'Acceleration ramp (rad/s²)'),
    int(RegAddr.DEC):       ('float',  'R_W', 'Deceleration ramp (rad/s², negative value)'),
    int(RegAddr.MAX_SPD):   ('float',  'R_W', 'Maximum velocity cap (rad/s)'),
    int(RegAddr.MST_ID):    ('uint32', 'R_W', 'Host CAN ID (MST_ID)'),
    int(RegAddr.ESC_ID):    ('uint32', 'R_W', 'Motor command CAN ID (ESC_ID)'),
    int(RegAddr.TIMEOUT):   ('uint32', 'R_W', 'CAN watchdog threshold; 0 = disabled'),
    int(RegAddr.CTRL_MODE): ('uint32', 'R_W', 'Control mode: 1=MIT 2=PV 3=VEL 4=FPH'),
    int(RegAddr.PMAX):      ('float',  'R_W', 'MIT position encoding range ±PMAX (rad)'),
    int(RegAddr.VMAX):      ('float',  'R_W', 'MIT velocity encoding range ±VMAX (rad/s)'),
    int(RegAddr.TMAX):      ('float',  'R_W', 'MIT torque encoding range ±TMAX (N·m)'),
    int(RegAddr.I_BW):      ('float',  'R_W', 'Current-loop bandwidth [100, 10000]'),
    int(RegAddr.KP_ASR):    ('float',  'R_W', 'Velocity loop Kp'),
    int(RegAddr.KI_ASR):    ('float',  'R_W', 'Velocity loop Ki'),
    int(RegAddr.KP_APR):    ('float',  'R_W', 'Position loop Kp'),
    int(RegAddr.KI_APR):    ('float',  'R_W', 'Position loop Ki'),
    int(RegAddr.OV_VALUE):  ('float',  'R_W', 'Overvoltage threshold (V)'),
    int(RegAddr.DETA):      ('float',  'R_W', 'Velocity loop damping coefficient [1, 30]; recommended 4.0'),
    int(RegAddr.V_BW):      ('float',  'R_W', 'Velocity loop filter bandwidth [0, 500]'),
    int(RegAddr.CAN_BR):    ('uint32', 'R_W', 'CAN baud rate code: 0=125K 1=200K 2=250K 3=500K 4=1M 5=2M'),
    int(RegAddr.SW_VER):    ('uint32', 'R',   'Firmware version'),
    int(RegAddr.GR):        ('float',  'R',   'Gear reduction ratio'),
    int(RegAddr.IMAX):      ('float',  'R',   'Driver maximum current (A)'),
    int(RegAddr.VBUS):      ('float',  'R',   'Bus voltage (V)'),
    int(RegAddr.TPCB):      ('float',  'R',   'PCB temperature (°C)'),
    int(RegAddr.TMTR):      ('float',  'R',   'Motor winding temperature (°C)'),
    int(RegAddr.P_M):       ('float',  'R',   'Motor-side rotor position (rad)'),
    int(RegAddr.XOUT):      ('float',  'R',   'Output shaft absolute position (rad)'),
}

# ── Motor parameter sets ───────────────────────────────────────────────────────

# Written to motor registers at startup (key → RegAddr)
_FIRMWARE_MOTOR_PARAMS: Dict[str, RegAddr] = {
    'acc':     RegAddr.ACC,
    'dec':     RegAddr.DEC,
    'max_spd': RegAddr.MAX_SPD,
    'kp_apr':  RegAddr.KP_APR,
    'ki_apr':  RegAddr.KI_APR,
    'kp_asr':  RegAddr.KP_ASR,
    'ki_asr':  RegAddr.KI_ASR,
}

# Kept in node memory only; influence command clamping/offsets
_SOFTWARE_MOTOR_PARAMS = frozenset({
    'kp', 'kd',
    'joint_limit_min', 'joint_limit_max',
    'motor_homing_pos',
    'max_vel', 'max_accel', 'max_decel',
})

_ALL_MOTOR_PARAMS = frozenset(_FIRMWARE_MOTOR_PARAMS) | _SOFTWARE_MOTOR_PARAMS


# ── Node ───────────────────────────────────────────────────────────────────────

class MotorNode(Node):

    def __init__(self, node_name: str = 'motor_node'):
        super().__init__(node_name)

        self._cb_timer    = MutuallyExclusiveCallbackGroup()
        self._cb_services = ReentrantCallbackGroup()
        self._cb_actions  = ReentrantCallbackGroup()
        self._cb_subs     = ReentrantCallbackGroup()

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        config = self._read_toml(config_path)

        if 'defaults' not in config:
            self.get_logger().fatal(
                f'[defaults] section missing from {config_path}'
            )
            raise SystemExit(1)

        self._defaults = config.pop('defaults')
        self._update_rate_hz = float(self._defaults['update_rate_hz'])
        use_node_prefix      = bool(self._defaults.get('use_node_name_as_topic_base', True))
        self._ns             = '~' if use_node_prefix else ''

        self.get_logger().info(
            f'Config loaded from {config_path} ({len(config)} motors) — '
            f'{self._update_rate_hz:.0f} Hz'
        )

        self._buses:         Dict[Tuple, DamiaoCANComms] = {}
        self._motors:        Dict[str, object]           = {}
        self._motor_enabled: Dict[str, bool]             = {}
        self._motor_mode:    Dict[str, Optional[RunMode]] = {}
        self._last_err:      Dict[str, int]              = {}
        self._motor_locks:   Dict[str, threading.Lock]   = {}
        self._motor_cfg:     Dict[str, dict]             = {}

        self._init_motors(config)

        # ── Publishers ─────────────────────────────────────────────────────
        self._joint_state_pub = self.create_publisher(
            JointState, self._topic('joint_states'), 10
        )
        self._state_pubs: Dict[str, object] = {}
        self._fault_pubs: Dict[str, object] = {}

        for name in self._motors:
            self._state_pubs[name] = self.create_publisher(
                MotorState, self._topic(f'motors/{name}/state'), 10
            )
            self._fault_pubs[name] = self.create_publisher(
                MotorFault, self._topic(f'motors/{name}/fault'), 10
            )
            self._motors[name].set_feedback_callback(
                lambda fb, n=name: self._on_feedback(n, fb)
            )

        # ── Subscribers ────────────────────────────────────────────────────
        _mode_subs = [
            (OperationCommand,  'cmd_mit',           self._on_cmd_mit),
            (PositionPPCommand, 'cmd_position_pv',   self._on_cmd_position_pv),
            (VelocityCommand,   'cmd_velocity',      self._on_cmd_velocity),
            (PositionCSPCommand,'cmd_force_position', self._on_cmd_force_position),
        ]
        for name in self._motors:
            for msg_type, suffix, cb in _mode_subs:
                self.create_subscription(
                    msg_type,
                    self._topic(f'motors/{name}/{suffix}'),
                    lambda msg, n=name, fn=cb: fn(msg, n),
                    10,
                    callback_group=self._cb_subs,
                )

        # ── Services ───────────────────────────────────────────────────────
        self.create_service(EnableMotor,     self._topic('enable_motor'),      self._srv_enable_motor,      callback_group=self._cb_services)
        self.create_service(SetRunMode,      self._topic('set_run_mode'),      self._srv_set_run_mode,      callback_group=self._cb_services)
        self.create_service(SetZeroPosition, self._topic('set_zero_position'), self._srv_set_zero_position, callback_group=self._cb_services)
        self.create_service(ReadParam,       self._topic('read_param'),        self._srv_read_param,        callback_group=self._cb_services)
        self.create_service(WriteParam,      self._topic('write_param'),       self._srv_write_param,       callback_group=self._cb_services)
        self.create_service(MotorParam,      self._topic('motor_param'),       self._srv_motor_param,       callback_group=self._cb_services)
        self.create_service(Help,            self._topic('help'),              self._srv_help,              callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('stop_all'),          self._srv_stop_all,          callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('clear_faults'),      self._srv_clear_faults,      callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('save_params'),       self._srv_save_params,       callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('homing'),            self._srv_homing,            callback_group=self._cb_services)

        # ── Actions ────────────────────────────────────────────────────────
        self._move_action = ActionServer(
            self, MoveToPosition, self._topic('move_to_position'),
            execute_callback=self._execute_move_to_position,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_actions,
        )
        self._vel_action = ActionServer(
            self, SetVelocityAction, self._topic('set_velocity'),
            execute_callback=self._execute_set_velocity,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_actions,
        )

        # ── Timer ──────────────────────────────────────────────────────────
        self._timer = self.create_timer(
            1.0 / self._update_rate_hz,
            self._update_cb,
            callback_group=self._cb_timer,
        )

        self.get_logger().info(
            f'MotorNode ready — {len(self._motors)} motor(s), {self._update_rate_hz:.0f} Hz'
        )

    # ── Init helpers ───────────────────────────────────────────────────────────

    def _topic(self, suffix: str) -> str:
        return f'{self._ns}/{suffix}'

    def _read_toml(self, path: str) -> dict:
        try:
            with open(path, 'rb') as f:
                return tomllib.load(f)
        except FileNotFoundError:
            self.get_logger().fatal(f'Config not found: {path}')
            raise
        except Exception as e:
            self.get_logger().fatal(f'Failed to parse config: {e}')
            raise

    def _bus_key(self, cfg: dict) -> Tuple:
        return (
            cfg.get('channel',   self._defaults['channel']),
            cfg.get('bustype',   self._defaults['bustype']),
            cfg.get('bitrate',   self._defaults['bitrate']),
            int(cfg.get('master_id', self._defaults['master_id'])),
        )

    def _get_or_create_bus(self, cfg: dict) -> DamiaoCANComms:
        key = self._bus_key(cfg)
        if key not in self._buses:
            channel, bustype, bitrate, master_id = key
            self.get_logger().info(
                f'Opening CAN bus  channel={channel}  master_id={master_id}'
            )
            bus = DamiaoCANComms(
                channel=channel,
                bustype=bustype,
                bitrate=bitrate,
                master_id=master_id,
                rx_timeout=float(cfg.get('rx_timeout', self._defaults['rx_timeout'])),
                on_error=lambda exc, ch=channel: self.get_logger().error(
                    f'CAN bus error on {ch}: {exc}'
                ),
            )
            bus.start_listener()
            self._buses[key] = bus
        return self._buses[key]

    def _resolve_mode(self, cfg: dict) -> Optional[RunMode]:
        mode_str = cfg.get('operation_mode') or self._defaults.get('operation_mode')
        if not mode_str:
            return None
        try:
            return RunMode[mode_str.upper()]
        except KeyError:
            self.get_logger().warning(
                f'Unknown operation_mode "{mode_str}" — '
                f'valid: {[m.name for m in RunMode]}'
            )
            return None

    def _init_motors(self, config: dict) -> None:
        for name, cfg in config.items():
            motor_type = cfg.get('type')
            if motor_type not in _MOTOR_CLASS_MAP:
                self.get_logger().error(
                    f'Unknown motor type "{motor_type}" for [{name}] — '
                    f'valid: {list(_MOTOR_CLASS_MAP)}'
                )
                continue
            try:
                mod_name, cls_name = _MOTOR_CLASS_MAP[motor_type].rsplit('.', 1)
                MotorClass = getattr(importlib.import_module(mod_name), cls_name)
                bus   = self._get_or_create_bus(cfg)
                motor = MotorClass(
                    motor_id   = int(cfg['motor_id']),
                    master_id  = int(cfg.get('master_id', self._defaults['master_id'])),
                    comms      = bus,
                    rx_timeout = float(cfg.get('rx_timeout', self._defaults['rx_timeout'])),
                )
                self._motors[name]        = motor
                self._motor_enabled[name] = False
                self._motor_mode[name]    = None
                self._last_err[name]      = -1
                self._motor_locks[name]   = threading.Lock()

                params = {k: cfg.get(k) for k in _ALL_MOTOR_PARAMS}
                self._motor_cfg[name] = params

                # Write firmware registers at startup
                for key, addr in _FIRMWARE_MOTOR_PARAMS.items():
                    val = params.get(key)
                    if val is not None:
                        v = float(val)
                        if key == 'dec':
                            v = -abs(v)  # DEC register requires negative value
                        motor.write_param_float(addr, v)

                target_mode = self._resolve_mode(cfg)
                mode_label  = 'none'
                if target_mode is not None:
                    motor.set_run_mode(target_mode)
                    confirmed = motor.read_param_uint(RegAddr.CTRL_MODE)
                    if confirmed == int(target_mode):
                        self._motor_mode[name] = target_mode
                        mode_label = target_mode.name
                    else:
                        actual = str(confirmed) if confirmed is not None else 'no response'
                        self.get_logger().error(
                            f'[{name}] mode mismatch — '
                            f'wrote {target_mode.name}, motor reports {actual}'
                        )

                self.get_logger().info(
                    f'  [{name}]  type={motor_type}  motor_id={cfg["motor_id"]}  '
                    f'channel={cfg.get("channel", self._defaults["channel"])}  '
                    f'mode={mode_label}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to init [{name}]: {e}')

    # ── Timer / feedback ───────────────────────────────────────────────────────

    def _on_feedback(self, name: str, fb: MotorFeedback) -> None:
        now      = self.get_clock().now().to_msg()
        user_pos = self._user_pos(name, fb.position)
        self._state_pubs[name].publish(self._build_state_msg(name, fb, user_pos, now))

    def _update_cb(self) -> None:
        if not self._motors:
            return
        now = self.get_clock().now().to_msg()
        js  = JointState()
        js.header.stamp = now

        for name, motor in self._motors.items():
            fb       = motor.feedback
            user_pos = self._user_pos(name, fb.position)

            js.name.append(name)
            js.position.append(user_pos)
            js.velocity.append(fb.velocity)
            js.effort.append(fb.torque)

            # Velocity-mode joint limit check
            if (self._motor_mode.get(name) == RunMode.VELOCITY
                    and self._motor_enabled.get(name, False)):
                lim = self._motor_cfg[name]
                lo, hi = lim.get('joint_limit_min'), lim.get('joint_limit_max')
                if ((lo is not None and fb.position < lo)
                        or (hi is not None and fb.position > hi)):
                    with self._motor_locks[name]:
                        motor.disable()
                    self._motor_enabled[name] = False
                    self.get_logger().error(
                        f'[{name}] Joint limit exceeded in velocity mode '
                        f'(pos={fb.position:.4f}, limits=[{lo}, {hi}]) — disabled'
                    )

            # Fault change detection
            if fb.err != self._last_err[name]:
                if fb.err not in (FaultCode.DISABLED, FaultCode.ENABLED):
                    self.get_logger().error(
                        f'[{name}] Fault: {FaultCode(fb.err).name}'
                        if fb.err in FaultCode._value2member_map_
                        else f'[{name}] Fault code: 0x{fb.err:X}'
                    )
                elif self._last_err[name] not in (-1, FaultCode.DISABLED, FaultCode.ENABLED):
                    self.get_logger().info(f'[{name}] Fault cleared')
                self._fault_pubs[name].publish(self._build_fault_msg(name, fb, now))
                self._last_err[name] = fb.err

        self._joint_state_pub.publish(js)

    # ── Command helpers ────────────────────────────────────────────────────────

    def _check_mode(self, name: str, required: RunMode) -> bool:
        if not self._motor_enabled.get(name, False):
            self.get_logger().warning(f'[{name}] Motor not enabled')
        current = self._motor_mode.get(name)
        if current is None:
            self.get_logger().warning(f'[{name}] Run mode unknown')
            return False
        if current != required:
            self.get_logger().error(
                f'[{name}] Mode mismatch: in {current.name}, expected {required.name}'
            )
            return False
        return True

    def _check_joint_limits(self, name: str, motor_pos: float) -> bool:
        lim = self._motor_cfg[name]
        lo, hi = lim.get('joint_limit_min'), lim.get('joint_limit_max')
        if lo is not None and motor_pos < lo:
            self.get_logger().error(
                f'[{name}] Rejected: pos {motor_pos:.4f} < limit_min {lo:.4f}'
            )
            return False
        if hi is not None and motor_pos > hi:
            self.get_logger().error(
                f'[{name}] Rejected: pos {motor_pos:.4f} > limit_max {hi:.4f}'
            )
            return False
        return True

    def _clamp_vel(self, name: str, v: float) -> float:
        lim = self._motor_cfg[name].get('max_vel')
        if lim is not None and abs(v) > lim:
            clamped = lim if v > 0.0 else -lim
            self.get_logger().warning(f'[{name}] Velocity clamped to {clamped:.3f} rad/s')
            return clamped
        return v

    def _clamp_accel(self, name: str, v: float) -> float:
        lim = self._motor_cfg[name].get('max_accel')
        if lim is not None and v > lim:
            self.get_logger().warning(f'[{name}] Acceleration clamped to {lim:.3f} rad/s²')
            return lim
        return v

    def _user_pos(self, name: str, motor_pos: float) -> float:
        return motor_pos - (self._motor_cfg[name].get('motor_homing_pos') or 0.0)

    def _motor_pos(self, name: str, cmd_pos: float) -> float:
        return cmd_pos + (self._motor_cfg[name].get('motor_homing_pos') or 0.0)

    # ── Command subscribers ────────────────────────────────────────────────────

    def _on_cmd_mit(self, msg: OperationCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.MIT):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        cfg = self._motor_cfg[name]
        kp  = float(cfg['kp']) if cfg.get('kp') is not None else 0.0
        kd  = float(cfg['kd']) if cfg.get('kd') is not None else 0.0
        with self._motor_locks[name]:
            try:
                self._motors[name].set_operation_control(
                    position  = motor_pos,
                    velocity  = self._clamp_vel(name, msg.velocity),
                    kp        = kp,
                    kd        = kd,
                    torque_ff = msg.torque_ff,
                )
            except Exception as e:
                self.get_logger().error(f'[{name}] MIT command error: {e}')

    def _on_cmd_position_pv(self, msg: PositionPPCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.POSITION_VELOCITY):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        speed = self._clamp_vel(name, msg.speed if msg.speed > 0.0 else 2.0)
        with self._motor_locks[name]:
            try:
                self._motors[name].set_position_velocity(motor_pos, speed)
            except Exception as e:
                self.get_logger().error(f'[{name}] position_pv command error: {e}')

    def _on_cmd_velocity(self, msg: VelocityCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.VELOCITY):
            return
        vel = self._clamp_vel(name, msg.velocity)
        with self._motor_locks[name]:
            try:
                self._motors[name].set_velocity(vel)
            except Exception as e:
                self.get_logger().error(f'[{name}] velocity command error: {e}')

    def _on_cmd_force_position(self, msg: PositionCSPCommand, name: str) -> None:
        # Reuses PositionCSPCommand: position (rad), speed_limit (rad/s, 0-100),
        # current_limit (per-unit 0-1.0 fraction of Imax).
        if not self._check_mode(name, RunMode.FORCE_POSITION_HYBRID):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        speed   = self._clamp_vel(name, msg.speed_limit if msg.speed_limit > 0.0 else 2.0)
        i_pu    = min(max(msg.current_limit, 0.0), 1.0) if msg.current_limit > 0.0 else 1.0
        with self._motor_locks[name]:
            try:
                self._motors[name].set_force_position(motor_pos, speed, i_pu)
            except Exception as e:
                self.get_logger().error(f'[{name}] force_position command error: {e}')

    # ── Services ───────────────────────────────────────────────────────────────

    def _resolve_motors(self, name: str) -> Optional[Dict[str, object]]:
        if name == 'all':
            return dict(self._motors)
        m = self._motors.get(name)
        return {name: m} if m is not None else None

    def _srv_enable_motor(self, req: EnableMotor.Request, res: EnableMotor.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res

        failed = []
        fb = None
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    if req.enable:
                        fb = motor.enable()
                        self._motor_enabled[name] = True
                    else:
                        fb = motor.disable()
                        self._motor_enabled[name] = False
                        if req.clear_fault:
                            motor.clear_faults()
            except Exception as e:
                failed.append(f'{name}: {e}')

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        else:
            res.success = True
            res.message = 'OK'
            if fb and req.name != 'all':
                res.position = fb.position
                res.velocity = fb.velocity
                res.torque   = fb.torque
        return res

    def _srv_set_run_mode(self, req: SetRunMode.Request, res: SetRunMode.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res
        try:
            mode = RunMode(req.mode)
        except ValueError:
            res.success = False
            res.message = (
                f'Invalid mode {req.mode}. '
                'Valid: 1=MIT 2=POSITION_VELOCITY 3=VELOCITY 4=FORCE_POSITION_HYBRID'
            )
            return res

        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    if req.automatic_enable_disable:
                        motor.disable()
                        self._motor_enabled[name] = False
                    motor.set_run_mode(mode)
                    confirmed = motor.read_param_uint(RegAddr.CTRL_MODE)
                    if confirmed is None:
                        failed.append(f'{name}: no response reading back CTRL_MODE')
                        continue
                    if confirmed != int(mode):
                        failed.append(
                            f'{name}: mode mismatch — wrote {int(mode)} ({mode.name}), '
                            f'motor reports {confirmed}'
                        )
                        continue
                    self._motor_mode[name] = mode
                    if req.automatic_enable_disable:
                        motor.enable()
                        self._motor_enabled[name] = True
            except Exception as e:
                failed.append(f'{name}: {e}')

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        else:
            res.success = True
            res.message = f'Run mode set to {mode.name}'
        return res

    def _srv_set_zero_position(self, req: SetZeroPosition.Request, res: SetZeroPosition.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res
        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    motor.set_zero_position()
            except Exception as e:
                failed.append(f'{name}: {e}')
        res.success = not bool(failed)
        res.message = 'Zero position set' if not failed else 'Errors — ' + ', '.join(failed)
        return res

    def _srv_read_param(self, req: ReadParam.Request, res: ReadParam.Response):
        motor = self._motors.get(req.name)
        if motor is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found. "all" not supported for read_param.'
            return res
        try:
            with self._motor_locks[req.name]:
                if req.index in _UINT_REGS:
                    val = motor.read_param_uint(req.index)
                else:
                    val = motor.read_param_float(req.index)
            if val is None:
                res.success = False
                res.message = 'No response from motor'
            else:
                res.success = True
                res.message = 'OK'
                res.value   = float(val)
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_write_param(self, req: WriteParam.Request, res: WriteParam.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res

        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    if req.index in _UINT_REGS:
                        motor.write_param_uint(req.index, int(req.value))
                    else:
                        motor.write_param_float(req.index, req.value)
                    if req.persist:
                        # save_params only works while motor is disabled
                        was_enabled = self._motor_enabled[name]
                        if was_enabled:
                            motor.disable()
                            self._motor_enabled[name] = False
                        motor.save_params()
                        time.sleep(0.05)  # allow 30 ms flash write
                        if was_enabled:
                            motor.enable()
                            self._motor_enabled[name] = True
            except Exception as e:
                failed.append(f'{name}: {e}')

        res.success = not bool(failed)
        res.message = ('OK' + (' (persisted)' if req.persist else '')) if not failed \
                      else 'Errors — ' + ', '.join(failed)
        return res

    def _srv_motor_param(self, req: MotorParam.Request, res: MotorParam.Response):
        motor = self._motors.get(req.name)
        if motor is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found. "all" not supported.'
            res.value   = float('nan')
            return res
        param = req.param.lower()
        if param not in _ALL_MOTOR_PARAMS:
            res.success = False
            res.message = f'Unknown param {req.param!r}. Valid: {sorted(_ALL_MOTOR_PARAMS)}'
            res.value   = float('nan')
            return res
        if req.set:
            self._motor_cfg[req.name][param] = req.value
            if param in _FIRMWARE_MOTOR_PARAMS:
                v = req.value
                if param == 'dec':
                    v = -abs(v)
                with self._motor_locks[req.name]:
                    motor.write_param_float(_FIRMWARE_MOTOR_PARAMS[param], v)
            res.success = True
            res.message = f'{param} set to {req.value}'
            res.value   = req.value
        else:
            val = self._motor_cfg[req.name].get(param)
            res.success = True
            res.value   = float(val) if val is not None else float('nan')
            res.message = 'OK' if val is not None else f'{param} not configured'
        return res

    def _srv_help(self, req: Help.Request, res: Help.Response):
        filt = req.filter.lower()
        reg_names = {int(r): r.name for r in RegAddr}
        for index, (type_str, access, desc) in _PARAM_INFO.items():
            reg_name = reg_names.get(index, f'0x{index:02X}')
            if filt and filt not in reg_name.lower() and filt not in desc.lower():
                continue
            res.codes.append(index)
            res.names.append(reg_name)
            res.types.append(type_str)
            res.access.append(access)
            res.descriptions.append(desc)
        return res

    def _srv_stop_all(self, _req: Trigger.Request, res: Trigger.Response):
        failed = []
        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    motor.disable()
                    self._motor_enabled[name] = False
                except Exception as e:
                    failed.append(f'{name}: {e}')
        res.success = not bool(failed)
        res.message = (f'{len(self._motors)} motor(s) stopped') if not failed \
                      else 'Errors — ' + ', '.join(failed)
        return res

    def _srv_clear_faults(self, _req: Trigger.Request, res: Trigger.Response):
        failed = []
        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    motor.clear_faults()
                except Exception as e:
                    failed.append(f'{name}: {e}')
        res.success = not bool(failed)
        res.message = 'Faults cleared' if not failed else 'Errors — ' + ', '.join(failed)
        return res

    def _srv_save_params(self, _req: Trigger.Request, res: Trigger.Response):
        """Disable each motor, save parameters to flash, then re-enable."""
        failed = []
        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    was_enabled = self._motor_enabled[name]
                    if was_enabled:
                        motor.disable()
                        self._motor_enabled[name] = False
                    motor.save_params()
                    time.sleep(0.05)  # 30 ms max flash write + margin
                    if was_enabled:
                        motor.enable()
                        self._motor_enabled[name] = True
                except Exception as e:
                    failed.append(f'{name}: {e}')
        res.success = not bool(failed)
        res.message = 'Parameters saved' if not failed else 'Errors — ' + ', '.join(failed)
        return res

    def _srv_homing(self, _req: Trigger.Request, res: Trigger.Response):
        """Switch to Position-Velocity mode and command position 0.0 for all motors."""
        failed = []
        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    motor.disable()
                    self._motor_enabled[name] = False
                    motor.set_run_mode(RunMode.POSITION_VELOCITY)
                    self._motor_mode[name] = RunMode.POSITION_VELOCITY
                    motor.enable()
                    self._motor_enabled[name] = True
                    motor.set_position_velocity(self._motor_pos(name, 0.0), velocity_rad_s=2.0)
                except Exception as e:
                    failed.append(f'{name}: {e}')
                    self.get_logger().error(f'Homing failed for [{name}]: {e}')
        res.success = not bool(failed)
        res.message = (f'Homing commanded for {len(self._motors)} motor(s)') if not failed \
                      else 'Errors — ' + ', '.join(failed)
        return res

    # ── Actions ────────────────────────────────────────────────────────────────

    def _execute_move_to_position(self, goal_handle) -> MoveToPosition.Result:
        req    = goal_handle.request
        motor  = self._motors.get(req.name)
        result = MoveToPosition.Result()

        if motor is None:
            result.success = False
            result.message = f'Motor {req.name!r} not found'
            goal_handle.abort()
            return result

        speed = req.speed_limit if req.speed_limit > 0.0 else 2.0

        with self._motor_locks[req.name]:
            motor.set_run_mode(RunMode.POSITION_VELOCITY)
            self._motor_mode[req.name] = RunMode.POSITION_VELOCITY
            motor.enable()
            self._motor_enabled[req.name] = True
            motor.set_position_velocity(req.target_position, velocity_rad_s=speed)

        start    = time.monotonic()
        feedback = MoveToPosition.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success        = False
                result.message        = 'Cancelled'
                result.final_position = motor.feedback.position
                result.elapsed_time   = time.monotonic() - start
                return result

            fb      = motor.feedback
            elapsed = time.monotonic() - start
            error   = abs(req.target_position - fb.position)

            feedback.current_position = fb.position
            feedback.position_error   = error
            feedback.elapsed_time     = elapsed
            goal_handle.publish_feedback(feedback)

            if error <= req.tolerance:
                goal_handle.succeed()
                result.success        = True
                result.message        = 'Target reached'
                result.final_position = fb.position
                result.elapsed_time   = elapsed
                return result

            if req.timeout > 0.0 and elapsed >= req.timeout:
                goal_handle.abort()
                result.success        = False
                result.message        = f'Timeout after {elapsed:.2f}s, error={error:.4f} rad'
                result.final_position = fb.position
                result.elapsed_time   = elapsed
                return result

            time.sleep(0.01)

        goal_handle.abort()
        result.success = False
        result.message = 'Node shutting down'
        return result

    def _execute_set_velocity(self, goal_handle) -> SetVelocityAction.Result:
        req    = goal_handle.request
        motor  = self._motors.get(req.name)
        result = SetVelocityAction.Result()

        if motor is None:
            result.success = False
            result.message = f'Motor {req.name!r} not found'
            goal_handle.abort()
            return result

        with self._motor_locks[req.name]:
            motor.set_run_mode(RunMode.VELOCITY)
            self._motor_mode[req.name] = RunMode.VELOCITY
            motor.enable()
            self._motor_enabled[req.name] = True
            motor.set_velocity(req.target_velocity)

        start     = time.monotonic()
        vel_sum   = 0.0
        vel_count = 0
        feedback  = SetVelocityAction.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._motor_locks[req.name]:
                    motor.set_velocity(0.0)
                goal_handle.canceled()
                result.success          = False
                result.message          = 'Cancelled'
                result.average_velocity = vel_sum / max(vel_count, 1)
                result.elapsed_time     = time.monotonic() - start
                return result

            fb      = motor.feedback
            elapsed = time.monotonic() - start
            vel_sum   += fb.velocity
            vel_count += 1

            feedback.current_velocity = fb.velocity
            feedback.current_torque   = fb.torque
            feedback.elapsed_time     = elapsed
            goal_handle.publish_feedback(feedback)

            if req.duration > 0.0 and elapsed >= req.duration:
                with self._motor_locks[req.name]:
                    motor.set_velocity(0.0)
                goal_handle.succeed()
                result.success          = True
                result.message          = f'Completed after {elapsed:.2f}s'
                result.average_velocity = vel_sum / max(vel_count, 1)
                result.elapsed_time     = elapsed
                return result

            time.sleep(0.01)

        goal_handle.abort()
        result.success = False
        result.message = 'Node shutting down'
        return result

    # ── Message builders ───────────────────────────────────────────────────────

    def _build_state_msg(self, name: str, fb: MotorFeedback, user_pos: float, stamp) -> MotorState:
        msg              = MotorState()
        msg.header.stamp = stamp
        msg.name         = name
        msg.position     = user_pos
        msg.velocity     = fb.velocity
        msg.torque       = fb.torque
        msg.temperature  = fb.t_mos     # MOSFET/driver temperature
        msg.mode         = fb.err       # 0=disabled 1=enabled 8-14=fault codes
        msg.fault        = fb.err if fb.err not in (FaultCode.DISABLED, FaultCode.ENABLED) else 0
        msg.enabled      = self._motor_enabled.get(name, False)
        return msg

    def _build_fault_msg(self, name: str, fb: MotorFeedback, stamp) -> MotorFault:
        msg              = MotorFault()
        msg.header.stamp = stamp
        msg.name         = name
        msg.fault_code   = fb.err
        msg.warning_code = 0
        err = fb.err
        msg.over_temp      = err in (FaultCode.MOS_OVER_TEMP, FaultCode.COIL_OVER_TEMP)
        msg.overvoltage    = (err == FaultCode.OVER_VOLTAGE)
        msg.undervoltage   = (err == FaultCode.UNDER_VOLTAGE)
        msg.a_phase_oc     = (err == FaultCode.OVER_CURRENT)   # general OC mapped here
        msg.stall_overload = (err == FaultCode.OVERLOAD)
        # Fields with no Damiao equivalent
        msg.driver_ic      = False
        msg.b_phase_oc     = False
        msg.c_phase_oc     = False
        msg.encoder_uncal  = False
        msg.hw_id_fault    = False
        msg.pos_init_fault = False
        msg.over_temp_warning = False
        return msg

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        print('[motor_node] Shutting down — disabling motors …')
        for name, motor in self._motors.items():
            try:
                motor.disable()
            except Exception as e:
                print(f'  Could not disable [{name}]: {e}')
        for key, bus in self._buses.items():
            try:
                bus.close()
            except Exception as e:
                print(f'  Could not close bus {key}: {e}')
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    _tmp = rclpy.create_node(f'_damiao_cfg_reader_{os.getpid()}')
    _tmp.declare_parameter('config_path', '')
    config_path = _tmp.get_parameter('config_path').value
    _tmp.destroy_node()

    node_name = 'motor_node'
    if config_path:
        try:
            with open(config_path, 'rb') as _f:
                _cfg = tomllib.load(_f)
            node_name = _cfg.get('defaults', {}).get('node_name', 'motor_node')
        except Exception:
            pass

    node = MotorNode(node_name=node_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
