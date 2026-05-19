"""
motor_node.py – Full ROS2 node for RobStride motors.

Topics
------
  Published:
    ~/joint_states                       sensor_msgs/JointState   all motors combined
    ~/motors/{name}/state                custom_interfaces/MotorState
    ~/motors/{name}/fault                custom_interfaces/MotorFault  (on change only)
  Subscribed:
    ~/motors/{name}/command              custom_interfaces/MotorCommand

Services
--------
  ~/enable_motor                         custom_interfaces/EnableMotor
  ~/set_run_mode                         custom_interfaces/SetRunMode
  ~/set_zero_position                    custom_interfaces/SetZeroPosition
  ~/read_param                           custom_interfaces/ReadParam
  ~/write_param                          custom_interfaces/WriteParam
  ~/help                                 custom_interfaces/Help
  ~/set_active_report                    custom_interfaces/SetActiveReport

Actions
-------
  ~/move_to_position                     custom_interfaces/MoveToPosition
  ~/set_velocity                         custom_interfaces/SetVelocity

Parameters
----------
  config_path              str    path to config.toml
  update_rate_hz           float  feedback publish rate (default 100)
  active_report_interval_ms int   motor push interval in ms (default 10)
"""

import importlib
import math
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
    CurrentCommand,
    MotorFault,
    MotorState,
    OperationCommand,
    PositionCSPCommand,
    PositionPPCommand,
    VelocityCommand,
)
from custom_interfaces.srv import (
    EnableMotor,
    GetCanConfig,
    Help,
    MotorParam,
    ReadParam,
    SetActiveReport,
    SetCanConfig,
    SetRunMode,
    SetZeroPosition,
    WriteParam,
)
from custom_interfaces.action import MoveToPosition
from custom_interfaces.action import SetVelocity as SetVelocityAction

from .comms import CANComms
from .motor_base import FaultBit, MotorFeedback, ParamIndex, RunMode, WarnBit


# ── Motor class registry ───────────────────────────────────────────────────

_MOTOR_CLASS_MAP = {
    'RS01': 'robstride_p.rs01.RS01',
    'RS02': 'robstride_p.rs02.RS02',
    'RS03': 'robstride_p.rs03.RS03',
    'RS04': 'robstride_p.rs04.RS04',
    'RS05': 'robstride_p.rs05.RS05',
}

# ── Parameter metadata for Help service ───────────────────────────────────
#   { ParamIndex → (type_str, access, description) }

_PARAM_INFO: Dict[ParamIndex, Tuple[str, str, str]] = {
    ParamIndex.MOTOR_BAUD:      ('uint8',  'R_W', 'Baud rate flag: 1=1Mbps  2=500Kbps  3=250Kbps  4=125Kbps (re-power required)'),
    ParamIndex.CAN_ID:          ('uint8',  'R_W', 'Motor CAN ID (0-127), effective immediately'),
    ParamIndex.CAN_MASTER:      ('uint8',  'R_W', 'Host CAN ID'),
    ParamIndex.RUN_MODE:        ('uint8',  'R_W', '0=OPERATION 1=POS_PP 2=VELOCITY 3=CURRENT 5=POS_CSP'),
    ParamIndex.IQ_REF:          ('float',  'R_W', 'Current mode Iq command (A), range -16 to 16'),
    ParamIndex.SPD_REF:         ('float',  'R_W', 'Velocity command (rad/s)'),
    ParamIndex.LIMIT_TORQUE:    ('float',  'R_W', 'Torque limit (N·m)'),
    ParamIndex.CUR_KP:          ('float',  'R_W', 'Current loop Kp'),
    ParamIndex.CUR_KI:          ('float',  'R_W', 'Current loop Ki'),
    ParamIndex.CUR_FILT_GAIN:   ('float',  'R_W', 'Current filter gain 0–1, default 0.1'),
    ParamIndex.LOC_REF:         ('float',  'R_W', 'Position command (rad)'),
    ParamIndex.LIMIT_SPD:       ('float',  'R_W', 'CSP_velocity speed limit (rad/s)'),
    ParamIndex.LIMIT_CUR:       ('float',  'R_W', 'Velocity_position current limit (A)'),
    ParamIndex.MECH_POS:        ('float',  'R',   'Mechanical angle of load (rad)'),
    ParamIndex.IQF:             ('float',  'R',   'Iq filter value (A)'),
    ParamIndex.MECH_VEL:        ('float',  'R',   'Speed of load (rad/s)'),
    ParamIndex.VBUS:            ('float',  'R',   'Bus voltage (V)'),
    ParamIndex.LOC_KP:          ('float',  'R_W', 'Position loop Kp, default 40'),
    ParamIndex.SPD_KP:          ('float',  'R_W', 'Speed loop Kp, default 6'),
    ParamIndex.SPD_KI:          ('float',  'R_W', 'Speed loop Ki, default 0.02'),
    ParamIndex.SPD_FILT_GAIN:   ('float',  'R_W', 'Speed filter gain, default 0.1'),
    ParamIndex.ACC_RAD:         ('float',  'R_W', 'Velocity mode acceleration (rad/s²), default 20'),
    ParamIndex.VEL_MAX:         ('float',  'R_W', 'PP position mode max speed (rad/s), default 10'),
    ParamIndex.ACC_SET:         ('float',  'R_W', 'PP position mode acceleration (rad/s²), default 10'),
    ParamIndex.EPS_SCAN_TIME:   ('uint16', 'R_W', 'Active report interval, 1=10 ms, each +1 adds 5 ms'),
    ParamIndex.CAN_TIMEOUT:     ('uint32', 'R_W', 'CAN watchdog threshold, 20000=1 s, 0=disabled'),
    ParamIndex.ZERO_STA:        ('uint8',  'R_W', 'Zero flag: 0 → 0–2π range, 1 → −π to π'),
    ParamIndex.DAMPER:          ('uint8',  'R_W', 'Damping switch: 0=off, 1=on'),
    ParamIndex.ADD_OFFSET:      ('float',  'R_W', 'Zero position offset (rad)'),
    ParamIndex.ALVEOLOUS_OPEN:  ('uint8',  'R_W', 'Cogging compensation switch: 0=off, 1=on'),
    ParamIndex.IQ_TEST:         ('uint8',  'R_W', 'Motor init calibration switch: 0=off, 1=on'),
    ParamIndex.DCC_SET:         ('float',  'R_W', 'PP deceleration (rad/s²), default 10'),
}

# MotorParam service ── firmware params are written to the motor on set
_FIRMWARE_MOTOR_PARAMS: Dict[str, ParamIndex] = {
    'loc_kp':      ParamIndex.LOC_KP,
    'spd_kp':      ParamIndex.SPD_KP,
    'spd_ki':      ParamIndex.SPD_KI,
    'cur_kp':      ParamIndex.CUR_KP,
    'cur_ki':      ParamIndex.CUR_KI,
    'max_torque':  ParamIndex.LIMIT_TORQUE,
    'max_current': ParamIndex.LIMIT_CUR,
}
_SOFTWARE_MOTOR_PARAMS = frozenset({
    'kp', 'kd',
    'joint_limit_min', 'joint_limit_max',
    'max_vel', 'max_accel', 'max_decel',
    'motor_homing_pos',
})
_ALL_MOTOR_PARAMS = frozenset(_FIRMWARE_MOTOR_PARAMS) | _SOFTWARE_MOTOR_PARAMS

# Parameters whose values are unsigned integers (not float)
_UINT_PARAMS = {
    ParamIndex.MOTOR_BAUD,
    ParamIndex.CAN_ID,
    ParamIndex.CAN_MASTER,
    ParamIndex.RUN_MODE,
    ParamIndex.EPS_SCAN_TIME,
    ParamIndex.CAN_TIMEOUT,
    ParamIndex.ZERO_STA,
    ParamIndex.DAMPER,
    ParamIndex.ALVEOLOUS_OPEN,
    ParamIndex.IQ_TEST,
}


# ── Node ───────────────────────────────────────────────────────────────────

class MotorNode(Node):

    def __init__(self, node_name: str = 'motor_node'):
        super().__init__(node_name)

        # Timer gets its own mutually-exclusive group so ticks never overlap.
        # Services and subs are reentrant — motor locks protect shared state.
        # Actions are reentrant so multiple goals can run concurrently.
        self._cb_timer    = MutuallyExclusiveCallbackGroup()
        self._cb_services = ReentrantCallbackGroup()
        self._cb_actions  = ReentrantCallbackGroup()
        self._cb_subs     = ReentrantCallbackGroup()

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('config_path', '')

        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        # ── Config ─────────────────────────────────────────────────────────
        config = self._read_toml(config_path)

        if 'defaults' not in config:
            self.get_logger().fatal(
                f'[defaults] section is missing from {config_path}. '
                'Add master_id, channel, bustype, bitrate, rx_timeout, '
                'active_report_interval_ms, update_rate_hz.'
            )
            raise SystemExit(1)

        self._defaults = config.pop('defaults')

        self._update_rate_hz     = float(self._defaults['update_rate_hz'])
        self._report_interval_ms = int(self._defaults.get('active_report_interval_ms', 10))
        use_node_prefix          = bool(self._defaults.get('use_node_name_as_topic_base', True))
        self._ns                 = '~' if use_node_prefix else ''

        self.get_logger().info(
            f'Config loaded from {config_path} ({len(config)} motors) — {self._update_rate_hz:.0f} Hz'
        )

        # ── State tracking ─────────────────────────────────────────────────
        self._buses:         Dict[Tuple, CANComms]          = {}
        self._motors:        Dict[str, object]             = {}
        self._motor_enabled: Dict[str, bool]               = {}
        self._motor_mode:    Dict[str, Optional[RunMode]]  = {}
        self._last_fault:    Dict[str, int]                = {}
        self._last_warning:  Dict[str, int]                = {}
        self._motor_locks:   Dict[str, threading.Lock]     = {}
        self._motor_cfg:     Dict[str, dict]               = {}

        self._init_motors(config)
        self._calibrate_joint_limits()

        # ── Publishers ─────────────────────────────────────────────────────
        self._joint_state_pub = self.create_publisher(JointState, self._topic('joint_states'), 10)
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

        # ── Subscribers — one topic per control mode ────────────────────────
        _mode_subs = [
            (OperationCommand,   'cmd_operation',    self._on_cmd_operation),
            (PositionPPCommand,  'cmd_position_pp',  self._on_cmd_position_pp),
            (VelocityCommand,    'cmd_velocity',     self._on_cmd_velocity),
            (CurrentCommand,     'cmd_current',      self._on_cmd_current),
            (PositionCSPCommand, 'cmd_position_csp', self._on_cmd_position_csp),
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
        self.create_service(Help,            self._topic('help'),              self._srv_help,              callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('homing'),            self._srv_homing,            callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('stop_all'),          self._srv_stop_all,          callback_group=self._cb_services)
        self.create_service(GetCanConfig,    self._topic('get_can_config'),    self._srv_get_can_config,    callback_group=self._cb_services)
        self.create_service(SetCanConfig,    self._topic('set_can_config'),    self._srv_set_can_config,    callback_group=self._cb_services)
        self.create_service(SetActiveReport, self._topic('set_active_report'), self._srv_set_active_report, callback_group=self._cb_services)
        self.create_service(Trigger,         self._topic('scan_motors'),       self._srv_scan_motors,       callback_group=self._cb_services)
        self.create_service(MotorParam,      self._topic('motor_param'),       self._srv_motor_param,       callback_group=self._cb_services)

        # ── Action servers ─────────────────────────────────────────────────
        self._move_action = ActionServer(
            self,
            MoveToPosition,
            self._topic('move_to_position'),
            execute_callback=self._execute_move_to_position,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_actions,
        )
        self._vel_action = ActionServer(
            self,
            SetVelocityAction,
            self._topic('set_velocity'),
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

    # ── Init helpers ────────────────────────────────────────────────────────

    def _topic(self, suffix: str) -> str:
        """
        Return the full topic / service / action name.

        When use_node_name_as_topic_base = true  (default):
            self._ns = '~'  →  '~/joint_states'  →  /{node_name}/joint_states
        When use_node_name_as_topic_base = false:
            self._ns = ''   →  '/joint_states'   →  /joint_states  (absolute root)
        """
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
            cfg.get('channel',  self._defaults['channel']),
            cfg.get('bustype',  self._defaults['bustype']),
            cfg.get('bitrate',  self._defaults['bitrate']),
        )

    def _get_or_create_bus(self, cfg: dict) -> CANComms:
        key = self._bus_key(cfg)
        if key not in self._buses:
            channel, bustype, bitrate = key
            self.get_logger().info(
                f'Opening CAN bus  channel={channel}  bustype={bustype}  bitrate={bitrate}'
            )
            bus = CANComms(
                channel=channel,
                bustype=bustype,
                bitrate=bitrate,
                rx_timeout=cfg.get('rx_timeout', self._defaults['rx_timeout']),
                on_error=lambda exc, ch=channel: self.get_logger().error(
                    f'CAN bus error on {ch}: {exc} — listener stopped'
                ),
            )
            bus.start_listener()
            self._buses[key] = bus
        return self._buses[key]

    def _resolve_operation_mode(self, cfg: dict) -> Optional[RunMode]:
        """Return the RunMode for a motor, preferring per-motor config over defaults."""
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

    def _calibrate_joint_limits(self) -> None:
        """Shift joint_limit_min/max and motor_homing_pos by ±2π if MECH_POS is outside limits.

        All three values (MECH_POS, joint_limit_min/max, motor_homing_pos) are in motor frame,
        so MECH_POS is compared directly against the limits. When shifted together by the same
        2π offset, the relative distances between them are preserved.
        """
        _2PI = 2.0 * math.pi
        for name, motor in self._motors.items():
            cfg = self._motor_cfg[name]
            lo  = cfg.get('joint_limit_min')
            hi  = cfg.get('joint_limit_max')

            if lo is None or hi is None:
                continue

            try:
                with self._motor_locks[name]:
                    mech_pos = motor.read_param_float(ParamIndex.MECH_POS)
            except Exception as e:
                self.get_logger().error(f'[{name}] calibrate_joint_limits: could not read MECH_POS: {e}')
                continue

            if mech_pos is None:
                self.get_logger().warning(f'[{name}] calibrate_joint_limits: no response for MECH_POS — skipping')
                continue

            if mech_pos > hi:
                cfg['joint_limit_max'] = hi + _2PI
                cfg['joint_limit_min'] = lo + _2PI
                if cfg.get('motor_homing_pos') is not None:
                    cfg['motor_homing_pos'] += _2PI
                self.get_logger().info(
                    f'[{name}] shifted +2π (mech_pos={mech_pos:.4f} > hi={hi:.4f})'
                )
            elif mech_pos < lo:
                cfg['joint_limit_max'] = hi - _2PI
                cfg['joint_limit_min'] = lo - _2PI
                if cfg.get('motor_homing_pos') is not None:
                    cfg['motor_homing_pos'] -= _2PI
                self.get_logger().info(
                    f'[{name}] shifted -2π (mech_pos={mech_pos:.4f} < lo={lo:.4f})'
                )

    def _init_motors(self, config: dict) -> None:
        for name, cfg in config.items():
            motor_type = cfg.get('type')
            if motor_type not in _MOTOR_CLASS_MAP:
                self.get_logger().error(
                    f'Unknown motor type "{motor_type}" for [{name}] — skipping. '
                    f'Valid: {list(_MOTOR_CLASS_MAP)}'
                )
                continue
            try:
                mod_name, cls_name = _MOTOR_CLASS_MAP[motor_type].rsplit('.', 1)
                MotorClass = getattr(importlib.import_module(mod_name), cls_name)
                bus = self._get_or_create_bus(cfg)
                motor = MotorClass(
                    motor_id   = int(cfg['motor_id']),
                    master_id  = int(cfg.get('master_id',  self._defaults['master_id'])),
                    comms      = bus,
                    rx_timeout = float(cfg.get('rx_timeout', self._defaults['rx_timeout'])),
                )
                self._motors[name]        = motor
                self._motor_enabled[name] = False
                self._motor_mode[name]    = None
                self._last_fault[name]    = 0
                self._last_warning[name]  = 0
                self._motor_locks[name]   = threading.Lock()

                lim = {k: cfg.get(k) for k in _ALL_MOTOR_PARAMS}
                self._motor_cfg[name] = lim

                for key, idx in _FIRMWARE_MOTOR_PARAMS.items():
                    if lim[key] is not None:
                        motor.write_param_float(idx, float(lim[key]))

                target_mode = self._resolve_operation_mode(cfg)
                mode_label  = 'none'
                if target_mode is not None:
                    motor.set_run_mode(target_mode)
                    confirmed = motor.read_param_uint(ParamIndex.RUN_MODE)
                    if confirmed == int(target_mode):
                        self._motor_mode[name] = target_mode
                        mode_label = target_mode.name
                    else:
                        actual = str(confirmed) if confirmed is not None else 'no response'
                        self.get_logger().error(
                            f'[{name}] operation_mode mismatch — '
                            f'wrote {target_mode.name}, motor reports {actual}'
                        )

                self.get_logger().info(
                    f'  [{name}]  type={motor_type}  motor_id={cfg["motor_id"]}  '
                    f'channel={cfg.get("channel", self._defaults["channel"])}  '
                    f'operation_mode={mode_label}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to initialise [{name}]: {e}')

    # ── Timer callback ───────────────────────────────────────────────────────

    def _on_feedback(self, name: str, fb) -> None:
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

            if (self._motor_mode.get(name) == RunMode.VELOCITY
                    and self._motor_enabled.get(name, False)):
                lim = self._motor_cfg[name]
                lo, hi = lim.get('joint_limit_min'), lim.get('joint_limit_max')
                if (lo is not None and fb.position < lo) or (hi is not None and fb.position > hi):
                    with self._motor_locks[name]:
                        self._motors[name].disable()
                    self._motor_enabled[name] = False
                    self.get_logger().error(
                        f'[{name}] Joint limit exceeded in velocity mode '
                        f'(motor_pos={fb.position:.4f} rad, limits=[{lo}, {hi}]) — motor disabled'
                    )

            if fb.fault != self._last_fault[name]:
                new_bits     = fb.fault & ~self._last_fault[name]
                cleared_bits = self._last_fault[name] & ~fb.fault
                if new_bits:
                    names = [b.name for b in FaultBit if new_bits & b]
                    self.get_logger().error(f'[{name}] Fault: {", ".join(names)}')
                if cleared_bits:
                    self.get_logger().info(f'[{name}] Fault cleared')
                self._fault_pubs[name].publish(self._build_fault_msg(name, fb, now))
                self._last_fault[name] = fb.fault

            if fb.warning != self._last_warning[name]:
                new_bits     = fb.warning & ~self._last_warning[name]
                cleared_bits = self._last_warning[name] & ~fb.warning
                if new_bits:
                    names = [b.name for b in WarnBit if new_bits & b]
                    self.get_logger().warning(f'[{name}] Warning: {", ".join(names)}')
                if cleared_bits:
                    self.get_logger().info(f'[{name}] Warning cleared')
                self._last_warning[name] = fb.warning

        self._joint_state_pub.publish(js)

    # ── Command subscriber ────────────────────────────────────────────────────

    def _check_mode(self, name: str, required: RunMode) -> bool:
        if not self._motor_enabled.get(name, False):
            self.get_logger().warning(f'[{name}] Motor is not enabled — command may have no effect')
        current = self._motor_mode.get(name)
        if current is None:
            self.get_logger().warning(
                f'[{name}] Run mode unknown — call set_run_mode before sending commands'
            )
            return False
        if current != required:
            self.get_logger().error(
                f'[{name}] Rejected command: motor is in {current.name}, expected {required.name}'
            )
            return False
        return True

    def _check_joint_limits(self, name: str, motor_pos: float) -> bool:
        """Check motor-frame position against joint limits (also in motor frame)."""
        lim = self._motor_cfg[name]
        lo, hi = lim.get('joint_limit_min'), lim.get('joint_limit_max')
        if lo is not None and motor_pos < lo:
            self.get_logger().error(
                f'[{name}] Rejected: motor_pos {motor_pos:.4f} rad < joint_limit_min {lo:.4f} rad'
            )
            return False
        if hi is not None and motor_pos > hi:
            self.get_logger().error(
                f'[{name}] Rejected: motor_pos {motor_pos:.4f} rad > joint_limit_max {hi:.4f} rad'
            )
            return False
        return True

    def _clamp_vel(self, name: str, value: float) -> float:
        limit = self._motor_cfg[name].get('max_vel')
        if limit is not None and abs(value) > limit:
            clamped = limit if value > 0.0 else -limit
            self.get_logger().warning(
                f'[{name}] Velocity {value:.3f} clamped to {clamped:.3f} rad/s (max_vel={limit:.3f})'
            )
            return clamped
        return value

    def _clamp_accel(self, name: str, value: float) -> float:
        limit = self._motor_cfg[name].get('max_accel')
        if limit is not None and value > limit:
            self.get_logger().warning(
                f'[{name}] Acceleration {value:.3f} clamped to {limit:.3f} rad/s² (max_accel={limit:.3f})'
            )
            return limit
        return value

    def _clamp_decel(self, name: str, value: float) -> float:
        limit = self._motor_cfg[name].get('max_decel')
        if limit is not None and value > limit:
            self.get_logger().warning(
                f'[{name}] Deceleration {value:.3f} clamped to {limit:.3f} rad/s² (max_decel={limit:.3f})'
            )
            return limit
        return value

    def _clamp_torque(self, name: str, value: float) -> float:
        """Signed clamp for torque_ff (operation mode)."""
        limit = self._motor_cfg[name].get('max_torque')
        if limit is not None and abs(value) > limit:
            clamped = limit if value > 0.0 else -limit
            self.get_logger().warning(
                f'[{name}] Torque {value:.3f} clamped to {clamped:.3f} N·m (max_torque={limit:.3f})'
            )
            return clamped
        return value

    def _clamp_current(self, name: str, value: float) -> float:
        """Signed clamp for iq_ref (current mode)."""
        limit = self._motor_cfg[name].get('max_current')
        if limit is not None and abs(value) > limit:
            clamped = limit if value > 0.0 else -limit
            self.get_logger().warning(
                f'[{name}] Current {value:.3f} clamped to {clamped:.3f} A (max_current={limit:.3f})'
            )
            return clamped
        return value

    def _resolve_torque_limit(self, name: str, msg_value: float) -> Optional[float]:
        """Return effective torque limit: clamp msg value to max_torque, or use max_torque as default."""
        max_t = self._motor_cfg[name].get('max_torque')
        if msg_value > 0.0:
            return min(msg_value, max_t) if max_t is not None else msg_value
        return max_t  # None → firmware default

    def _resolve_current_limit(self, name: str, msg_value: float) -> Optional[float]:
        """Return effective current limit: clamp msg value to max_current, or use max_current as default."""
        max_c = self._motor_cfg[name].get('max_current')
        if msg_value > 0.0:
            return min(msg_value, max_c) if max_c is not None else msg_value
        return max_c  # None → motor default (MAX_CURRENT_A)

    def _user_pos(self, name: str, motor_pos: float) -> float:
        """Return position relative to homing point (motor_pos − motor_homing_pos). For display only."""
        return motor_pos - (self._motor_cfg[name].get('motor_homing_pos') or 0.0)

    def _motor_pos(self, name: str, cmd_pos: float) -> float:
        """Convert a command (relative to homing point) to absolute motor-frame position."""
        return cmd_pos + (self._motor_cfg[name].get('motor_homing_pos') or 0.0)

    def _on_cmd_operation(self, msg: OperationCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.OPERATION):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        lim   = self._motor_cfg[name]
        vel   = self._clamp_vel(name, msg.velocity)
        kp    = float(lim['kp']) if lim['kp'] is not None else 0.0
        kd    = float(lim['kd']) if lim['kd'] is not None else 0.0
        motor = self._motors[name]
        with self._motor_locks[name]:
            try:
                motor.set_operation_control(
                    position  = motor_pos,
                    velocity  = vel,
                    torque_ff = self._clamp_torque(name, msg.torque_ff),
                    kp        = kp,
                    kd        = kd,
                )
            except Exception as e:
                self.get_logger().error(f'[{name}] operation command error: {e}')

    def _on_cmd_position_pp(self, msg: PositionPPCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.POSITION_PP):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        speed = self._clamp_vel(name,   msg.speed        if msg.speed        > 0.0 else 2.0)
        accel = self._clamp_accel(name, msg.acceleration if msg.acceleration > 0.0 else 10.0)
        decel = self._clamp_decel(name, msg.deceleration if msg.deceleration > 0.0 else accel)
        motor = self._motors[name]
        with self._motor_locks[name]:
            try:
                motor.set_position_pp(
                    position_rad        = motor_pos,
                    speed_rad_s         = speed,
                    acceleration_rad_s2 = accel,
                    deceleration_rad_s2 = decel,
                    torque_limit_nm     = self._resolve_torque_limit(name, msg.torque_limit),
                )
            except Exception as e:
                self.get_logger().error(f'[{name}] position_pp command error: {e}')

    def _on_cmd_velocity(self, msg: VelocityCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.VELOCITY):
            return
        vel   = self._clamp_vel(name,   msg.velocity)
        accel = self._clamp_accel(name, msg.acceleration if msg.acceleration > 0.0 else 20.0)
        motor = self._motors[name]
        with self._motor_locks[name]:
            try:
                motor.set_velocity(
                    velocity_rad_s      = vel,
                    current_limit_a     = self._resolve_current_limit(name, msg.current_limit),
                    acceleration_rad_s2 = accel,
                )
            except Exception as e:
                self.get_logger().error(f'[{name}] velocity command error: {e}')

    def _on_cmd_current(self, msg: CurrentCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.CURRENT):
            return
        iq = self._clamp_current(name, msg.current)
        motor = self._motors[name]
        with self._motor_locks[name]:
            try:
                motor.set_current(iq)
            except Exception as e:
                self.get_logger().error(f'[{name}] current command error: {e}')

    def _on_cmd_position_csp(self, msg: PositionCSPCommand, name: str) -> None:
        if not self._check_mode(name, RunMode.POSITION_CSP):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        speed = self._clamp_vel(name, msg.speed_limit if msg.speed_limit > 0.0 else 2.0)
        motor = self._motors[name]
        with self._motor_locks[name]:
            try:
                motor.set_position_csp(
                    position_rad      = motor_pos,
                    speed_limit_rad_s = speed,
                    current_limit_a   = self._resolve_current_limit(name, msg.current_limit),
                )
            except Exception as e:
                self.get_logger().error(f'[{name}] position_csp command error: {e}')

    # ── Services ─────────────────────────────────────────────────────────────

    def _resolve_motors(self, name: str) -> Optional[Dict[str, object]]:
        """
        Return ``{name: motor}`` for a single motor, or the full motors dict
        when name is ``'all'``.  Returns ``None`` if the name is unknown.
        """
        if name == 'all':
            return dict(self._motors)
        motor = self._motors.get(name)
        if motor is None:
            return None
        return {name: motor}

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
                        fb = motor.disable(clear_fault=req.clear_fault)
                        self._motor_enabled[name] = False
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
                'Valid: 0=OPERATION 1=POS_PP 2=VELOCITY 3=CURRENT 5=POS_CSP'
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
                    confirmed = motor.read_param_uint(ParamIndex.RUN_MODE)
                    if confirmed is None:
                        failed.append(f'{name}: no response when reading back RUN_MODE')
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
            res.message = f'Run mode set to {mode.name}' + (
                ' (auto disable/enable)' if req.automatic_enable_disable else ''
            )
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

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        else:
            res.success = True
            res.message = 'Zero position set'
        return res

    def _srv_read_param(self, req: ReadParam.Request, res: ReadParam.Response):
        # 'all' is not supported — a single float64 value cannot represent all motors.
        motor = self._motors.get(req.name)
        if motor is None:
            res.success = False
            res.message = (
                f'Motor {req.name!r} not found. '
                'Note: "all" is not supported for read_param.'
            )
            return res
        try:
            with self._motor_locks[req.name]:
                if req.index in _UINT_PARAMS:
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
                    if req.index in _UINT_PARAMS:
                        motor.write_param_uint8(req.index, int(req.value))
                    else:
                        motor.write_param_float(req.index, req.value)
                    if req.persist:
                        motor.save_params()
            except Exception as e:
                failed.append(f'{name}: {e}')

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        else:
            res.success = True
            res.message = 'OK' + (' (persisted)' if req.persist else '')
        return res

    def _srv_help(self, req: Help.Request, res: Help.Response):
        filt = req.filter.lower()
        for index, (type_str, access, desc) in _PARAM_INFO.items():
            if filt and filt not in index.name.lower() and filt not in desc.lower():
                continue
            res.codes.append(index.value)
            res.names.append(index.name)
            res.types.append(type_str)
            res.access.append(access)
            res.descriptions.append(desc)
        return res

    _BAUD_FLAG_STR = {1: '1Mbps', 2: '500Kbps', 3: '250Kbps', 4: '125Kbps'}

    def _srv_get_can_config(self, req: GetCanConfig.Request, res: GetCanConfig.Response):
        """Read CAN ID and baud rate directly from the motor firmware."""
        motor = self._motors.get(req.name)
        if motor is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found. "all" is not supported for get_can_config.'
            return res
        try:
            with self._motor_locks[req.name]:
                baud_flag = motor.read_param_uint(ParamIndex.MOTOR_BAUD) or 0
            res.success   = True
            res.message   = 'OK'
            res.can_id    = motor.motor_id
            res.baud_flag = baud_flag
            res.baud_rate = self._BAUD_FLAG_STR.get(baud_flag, f'unknown ({baud_flag})')
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_set_can_config(self, req: SetCanConfig.Request, res: SetCanConfig.Response):
        """
        Change CAN ID and / or baud rate.
          CAN ID   — immediate effect (comm type 7).
          Baud rate — saved to flash; takes effect after re-power (comm type 23).
        Pass 0 to leave a field unchanged.
        """
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res

        if req.baud_flag not in (0, 1, 2, 3, 4):
            res.success = False
            res.message = f'Invalid baud_flag {req.baud_flag}. Valid: 0=keep 1=1M 2=500K 3=250K 4=125K'
            return res

        if req.can_id > 127:
            res.success = False
            res.message = f'Invalid can_id {req.can_id}. Must be 0-127 (0=keep current).'
            return res

        changed = []
        failed  = []

        for name, motor in motors.items():
            with self._motor_locks[name]:
                try:
                    if req.can_id != 0:
                        motor.set_can_id(req.can_id)
                        changed.append(f'[{name}] CAN ID → {req.can_id}')
                    if req.baud_flag != 0:
                        motor.write_param_uint8(ParamIndex.MOTOR_BAUD, req.baud_flag)
                        motor.save_params()
                        changed.append(
                            f'[{name}] baud → {self._BAUD_FLAG_STR[req.baud_flag]} (re-power required)'
                        )
                except Exception as e:
                    failed.append(f'[{name}]: {e}')

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        elif not changed:
            res.success = True
            res.message = 'Nothing changed (both fields were 0)'
        else:
            res.success = True
            res.message = ', '.join(changed)
        return res

    def _srv_motor_param(self, req: MotorParam.Request, res: MotorParam.Response):
        motor = self._motors.get(req.name)
        if motor is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found. "all" is not supported.'
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
                with self._motor_locks[req.name]:
                    motor.write_param_float(_FIRMWARE_MOTOR_PARAMS[param], req.value)
            res.success = True
            res.message = f'{param} set to {req.value}'
            res.value   = req.value
        else:
            val = self._motor_cfg[req.name].get(param)
            res.success = True
            res.value   = float(val) if val is not None else float('nan')
            res.message = 'OK' if val is not None else f'{param} not set'

        return res

    def _set_timer_rate(self, hz: float) -> None:
        self._timer.cancel()
        self._timer = self.create_timer(
            1.0 / hz, self._update_cb, callback_group=self._cb_timer
        )

    def _srv_set_active_report(self, req: SetActiveReport.Request, res: SetActiveReport.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res

        if req.hz > 0.0:
            interval_ms = max(10, int(1000.0 / req.hz))
        else:
            interval_ms = self._report_interval_ms

        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    motor.enable_active_report(enable=req.enable, interval_ms=interval_ms)
            except Exception as e:
                failed.append(f'{name}: {e}')

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        else:
            if req.enable:
                publish_hz = 1000.0 / interval_ms
                self._set_timer_rate(publish_hz)
                state = f'enabled ({publish_hz:.1f} Hz / {interval_ms} ms)'
            else:
                self._set_timer_rate(self._update_rate_hz)
                state = 'disabled'
            res.success = True
            res.message = f'Active reporting {state} for {list(motors)}'
        return res

    def _srv_scan_motors(self, _req: Trigger.Request, res: Trigger.Response):
        """Query each configured motor for its MCU device ID and log the results."""
        lines = []
        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                uid = motor.get_device_id()
            if uid:
                uid_hex = uid.hex()
                self.get_logger().info(f'[{name}] CAN ID {motor.motor_id}  UID {uid_hex}')
                lines.append(f'{name}: CAN ID {motor.motor_id}  UID {uid_hex}')
            else:
                self.get_logger().warning(f'[{name}] CAN ID {motor.motor_id}  no response')
                lines.append(f'{name}: CAN ID {motor.motor_id}  no response')

        res.success = True
        res.message = '\n'.join(lines)
        return res

    def _srv_stop_all(self, _req: Trigger.Request, res: Trigger.Response):
        """Immediately disable every motor on the bus (comm type 4)."""
        failed = []

        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    motor.disable()
                    self._motor_enabled[name] = False
                except Exception as e:
                    failed.append(f'{name}: {e}')
                    self.get_logger().error(f'Stop failed for [{name}]: {e}')

        if failed:
            res.success = False
            res.message = 'Stop errors — ' + ', '.join(failed)
        else:
            res.success = True
            res.message = f'{len(self._motors)} motor(s) stopped'

        return res

    def _srv_homing(self, _req: Trigger.Request, res: Trigger.Response):
        """
        Stop all motors, switch to CSP position mode, then command position 0.0.
        Uses std_srvs/Trigger — no request fields required.
        """
        failed = []

        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    motor.disable()
                    self._motor_enabled[name] = False
                    motor.set_run_mode(RunMode.POSITION_CSP)
                    self._motor_mode[name] = RunMode.POSITION_CSP
                    motor.enable()
                    self._motor_enabled[name] = True
                    motor.set_position_csp(self._motor_pos(name, 0.0))
                except Exception as e:
                    failed.append(f'{name}: {e}')
                    self.get_logger().error(f'Homing failed for [{name}]: {e}')

        if failed:
            res.success = False
            res.message = 'Homing errors — ' + ', '.join(failed)
        else:
            res.success = True
            res.message = f'Homing commanded for {len(self._motors)} motor(s)'

        return res

    # ── Actions ───────────────────────────────────────────────────────────────

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
            motor.set_run_mode(RunMode.POSITION_CSP)
            self._motor_mode[req.name] = RunMode.POSITION_CSP
            motor.enable()
            self._motor_enabled[req.name] = True
            motor.set_position_csp(req.target_position, speed_limit_rad_s=speed)

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

        current_limit = req.current_limit if req.current_limit > 0.0 else None
        accel         = req.acceleration  if req.acceleration  > 0.0 else 20.0
        # Deceleration: use explicit value if provided, else fall back to accel
        decel         = req.deceleration  if req.deceleration  > 0.0 else accel

        with self._motor_locks[req.name]:
            motor.set_run_mode(RunMode.VELOCITY)
            self._motor_mode[req.name] = RunMode.VELOCITY
            motor.enable()
            self._motor_enabled[req.name] = True
            motor.set_velocity(req.target_velocity,
                               current_limit_a=current_limit,
                               acceleration_rad_s2=accel)

        start     = time.monotonic()
        vel_sum   = 0.0
        vel_count = 0
        feedback  = SetVelocityAction.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._motor_locks[req.name]:
                    # Apply deceleration ramp, then command stop
                    motor.set_velocity(0.0, acceleration_rad_s2=decel)
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
                    motor.set_velocity(0.0, acceleration_rad_s2=decel)
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

    # ── Message builders ──────────────────────────────────────────────────────

    def _build_state_msg(self, name: str, fb: MotorFeedback, user_pos: float, stamp) -> MotorState:
        msg              = MotorState()
        msg.header.stamp = stamp
        msg.name         = name
        msg.position     = user_pos
        msg.velocity     = fb.velocity
        msg.torque       = fb.torque
        msg.temperature  = float(fb.temperature)
        msg.mode         = fb.mode
        msg.fault        = fb.fault
        msg.enabled      = self._motor_enabled.get(name, False)
        return msg

    def _build_fault_msg(self, name: str, fb: MotorFeedback, stamp) -> MotorFault:
        msg              = MotorFault()
        msg.header.stamp = stamp
        msg.name         = name
        msg.fault_code   = fb.fault
        msg.warning_code = fb.warning
        msg.over_temp      = bool(fb.fault & FaultBit.OVER_TEMP)
        msg.driver_ic      = bool(fb.fault & FaultBit.DRIVER_IC)
        msg.undervoltage   = bool(fb.fault & FaultBit.UNDERVOLTAGE)
        msg.overvoltage    = bool(fb.fault & FaultBit.OVERVOLTAGE)
        msg.b_phase_oc     = bool(fb.fault & FaultBit.B_PHASE_OC)
        msg.c_phase_oc     = bool(fb.fault & FaultBit.C_PHASE_OC)
        msg.encoder_uncal  = bool(fb.fault & FaultBit.ENCODER_UNCAL)
        msg.hw_id_fault    = bool(fb.fault & FaultBit.HW_ID_FAULT)
        msg.pos_init_fault = bool(fb.fault & FaultBit.POS_INIT_FAULT)
        msg.stall_overload = bool(fb.fault & FaultBit.STALL_OVERLOAD)
        msg.a_phase_oc     = bool(fb.fault & FaultBit.A_PHASE_OC)
        msg.over_temp_warning = bool(fb.warning & WarnBit.OVER_TEMP_WARN)
        return msg

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        print('[motor_node] Shutting down — disabling motors …')
        for name, motor in self._motors.items():
            try:
                motor.disable()
            except Exception as e:
                print(f'[motor_node] Could not disable [{name}]: {e}')
        for key, bus in self._buses.items():
            try:
                bus.close()
            except Exception as e:
                print(f'[motor_node] Could not close bus {key}: {e}')
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    # Read node_name from config.toml before constructing MotorNode, because
    # Node.__init__ requires the name before any parameters can be declared.
    _tmp = rclpy.create_node(f'_robstride_cfg_reader_{os.getpid()}')
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
