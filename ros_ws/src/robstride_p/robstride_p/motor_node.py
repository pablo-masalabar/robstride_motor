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
import time
import threading
from typing import Dict, Optional, Tuple

import tomllib

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from custom_interfaces.msg import MotorCommand, MotorFault, MotorState
from custom_interfaces.srv import (
    EnableMotor,
    GetCanConfig,
    Help,
    ReadParam,
    SetCanConfig,
    SetRunMode,
    SetZeroPosition,
    WriteParam,
)
from custom_interfaces.action import MoveToPosition
from custom_interfaces.action import SetVelocity as SetVelocityAction

from .comms import CANComms
from .motor_base import FaultBit, MotorFeedback, ParamIndex, RunMode


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

        self._cb_group = ReentrantCallbackGroup()

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

        update_rate     = float(self._defaults['update_rate_hz'])
        report_interval = int(self._defaults['active_report_interval_ms'])
        use_node_prefix = bool(self._defaults.get('use_node_name_as_topic_base', True))
        self._ns        = '~' if use_node_prefix else ''

        self.get_logger().info(
            f'Config loaded from {config_path} ({len(config)} motors) '
            f'— {update_rate:.0f} Hz, report interval {report_interval} ms'
        )

        # ── State tracking ─────────────────────────────────────────────────
        self._buses:         Dict[Tuple, CANComms] = {}
        self._motors:        Dict[str, object]     = {}
        self._motor_enabled: Dict[str, bool]       = {}
        self._last_fault:    Dict[str, int]        = {}
        self._motor_locks:   Dict[str, threading.Lock] = {}

        self._init_motors(config, report_interval)

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

        # ── Subscribers ────────────────────────────────────────────────────
        self._cmd_subs: Dict[str, object] = {}
        for name in self._motors:
            self._cmd_subs[name] = self.create_subscription(
                MotorCommand,
                self._topic(f'motors/{name}/command'),
                lambda msg, n=name: self._on_command(msg, n),
                10,
                callback_group=self._cb_group,
            )

        # ── Services ───────────────────────────────────────────────────────
        self.create_service(EnableMotor,     self._topic('enable_motor'),      self._srv_enable_motor,      callback_group=self._cb_group)
        self.create_service(SetRunMode,      self._topic('set_run_mode'),      self._srv_set_run_mode,      callback_group=self._cb_group)
        self.create_service(SetZeroPosition, self._topic('set_zero_position'), self._srv_set_zero_position, callback_group=self._cb_group)
        self.create_service(ReadParam,       self._topic('read_param'),        self._srv_read_param,        callback_group=self._cb_group)
        self.create_service(WriteParam,      self._topic('write_param'),       self._srv_write_param,       callback_group=self._cb_group)
        self.create_service(Help,            self._topic('help'),              self._srv_help,              callback_group=self._cb_group)
        self.create_service(Trigger,         self._topic('homing'),            self._srv_homing,            callback_group=self._cb_group)
        self.create_service(Trigger,         self._topic('stop_all'),          self._srv_stop_all,          callback_group=self._cb_group)
        self.create_service(GetCanConfig,    self._topic('get_can_config'),    self._srv_get_can_config,    callback_group=self._cb_group)
        self.create_service(SetCanConfig,    self._topic('set_can_config'),    self._srv_set_can_config,    callback_group=self._cb_group)

        # ── Action servers ─────────────────────────────────────────────────
        self._move_action = ActionServer(
            self,
            MoveToPosition,
            self._topic('move_to_position'),
            execute_callback=self._execute_move_to_position,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self._vel_action = ActionServer(
            self,
            SetVelocityAction,
            self._topic('set_velocity'),
            execute_callback=self._execute_set_velocity,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        # ── Timer ──────────────────────────────────────────────────────────
        self._timer = self.create_timer(
            1.0 / update_rate,
            self._update_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'MotorNode ready — {len(self._motors)} motor(s), {update_rate:.0f} Hz'
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
            )
            bus.start_listener()
            self._buses[key] = bus
        return self._buses[key]

    def _init_motors(self, config: dict, report_interval_ms: int) -> None:
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
                motor.enable_active_report(enable=True, interval_ms=report_interval_ms)
                self._motors[name]        = motor
                self._motor_enabled[name] = False
                self._last_fault[name]    = -1
                self._motor_locks[name]   = threading.Lock()
                self.get_logger().info(
                    f'  [{name}]  type={motor_type}  motor_id={cfg["motor_id"]}  '
                    f'channel={cfg.get("channel", self._defaults["channel"])}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to initialise [{name}]: {e}')

    # ── Timer callback ───────────────────────────────────────────────────────

    def _update_cb(self) -> None:
        if not self._motors:
            return

        now = self.get_clock().now().to_msg()
        js  = JointState()
        js.header.stamp = now

        for name, motor in self._motors.items():
            motor.spin_once()
            fb = motor.feedback

            js.name.append(name)
            js.position.append(fb.position)
            js.velocity.append(fb.velocity)
            js.effort.append(fb.torque)

            self._state_pubs[name].publish(self._build_state_msg(name, fb, now))

            if fb.fault != self._last_fault[name]:
                self._fault_pubs[name].publish(self._build_fault_msg(name, fb, now))
                self._last_fault[name] = fb.fault

        self._joint_state_pub.publish(js)

    # ── Command subscriber ────────────────────────────────────────────────────

    def _on_command(self, msg: MotorCommand, name: str) -> None:
        motor = self._motors.get(name)
        if motor is None:
            return

        cmd = msg.command_type.lower()

        with self._motor_locks[name]:
            try:
                if cmd == 'position':
                    motor.set_position_csp(msg.value)
                elif cmd == 'velocity':
                    motor.set_velocity(msg.value)
                elif cmd == 'current':
                    motor.set_current(msg.value)
                elif cmd == 'torque':
                    motor.set_torque_limit(msg.value)
                elif cmd == 'operation':
                    motor.set_operation_control(
                        position  = msg.position,
                        velocity  = msg.velocity,
                        torque_ff = msg.torque_ff,
                        kp        = msg.kp,
                        kd        = msg.kd,
                    )
                else:
                    self.get_logger().warning(
                        f'[{name}] Unknown command_type: {msg.command_type!r}. '
                        'Expected: position | velocity | current | torque | operation'
                    )
            except Exception as e:
                self.get_logger().error(f'[{name}] Command error: {e}')

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
                    motor.set_run_mode(mode)
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
                    motor.enable()
                    self._motor_enabled[name] = True
                    motor.set_position_csp(0.0)
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

            motor.spin_once()
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

            motor.spin_once()
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

    def _build_state_msg(self, name: str, fb: MotorFeedback, stamp) -> MotorState:
        msg              = MotorState()
        msg.header.stamp = stamp
        msg.name         = name
        msg.position     = fb.position
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
        msg.over_temp      = bool(fb.fault & FaultBit.OVER_TEMP)
        msg.driver_ic      = bool(fb.fault & FaultBit.DRIVER_IC)
        msg.undervoltage   = bool(fb.fault & FaultBit.UNDERVOLTAGE)
        msg.overvoltage    = bool(fb.fault & FaultBit.OVERVOLTAGE)
        msg.c_phase_oc     = bool(fb.fault & FaultBit.C_PHASE_OC)
        msg.encoder_uncal  = bool(fb.fault & FaultBit.ENCODER_UNCAL)
        msg.hw_id_fault    = bool(fb.fault & FaultBit.HW_ID_FAULT)
        msg.pos_init_fault = bool(fb.fault & FaultBit.POS_INIT_FAULT)
        msg.stall_overload = bool(fb.fault & FaultBit.STALL_OVERLOAD)
        msg.a_phase_oc     = bool(fb.fault & FaultBit.A_PHASE_OC)
        return msg

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down — disabling motors …')
        for name, motor in self._motors.items():
            try:
                motor.disable()
            except Exception as e:
                self.get_logger().warning(f'Could not disable [{name}]: {e}')
        for key, bus in self._buses.items():
            try:
                bus.close()
            except Exception as e:
                self.get_logger().warning(f'Could not close bus {key}: {e}')
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    # Read node_name from config.toml before constructing MotorNode, because
    # Node.__init__ requires the name before any parameters can be declared.
    _tmp = rclpy.create_node('_robstride_cfg_reader')
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
