"""
motor_node.py – ROS2 node for EZmotion PCN/SCN C2 series motors (CANopen DS402).

Topics
------
  Published:
    ~/joint_states                    sensor_msgs/JointState
    ~/motors/{name}/state             custom_interfaces/MotorState
    ~/motors/{name}/fault             custom_interfaces/MotorFault  (on change only)
  Subscribed:
    ~/motors/{name}/cmd_position_pp   custom_interfaces/PositionPPCommand  (PP mode)
    ~/motors/{name}/cmd_position_csp  custom_interfaces/PositionCSPCommand (CSP mode)
    ~/motors/{name}/cmd_velocity      custom_interfaces/VelocityCommand    (PV mode)

Services
--------
  ~/enable_motor       custom_interfaces/EnableMotor
  ~/set_run_mode       custom_interfaces/SetRunMode
  ~/set_active_report  custom_interfaces/SetActiveReport
  ~/read_param         custom_interfaces/ReadParam    (SDO read by OD index)
  ~/write_param        custom_interfaces/WriteParam   (SDO write by OD index)
  ~/stop_all           std_srvs/Trigger
  ~/save_params        std_srvs/Trigger
Actions
-------
  ~/homing             custom_interfaces/EZMotionHoming

Actions
-------
  ~/move_to_position   custom_interfaces/MoveToPosition
  ~/set_velocity       custom_interfaces/SetVelocity

Config
------
  [defaults]
    node_name           str    ROS2 node name
    channel             str    SocketCAN interface (e.g. "can0")
    bustype             str    python-can bus type (default "socketcan")
    bitrate             int    baud rate in bps (default 1000000)
    rx_timeout          float  SDO reply timeout in seconds (default 0.1)
    update_rate_hz      float  joint_states publish rate
    operation_mode      str    default DS402 mode: PP PV PT HM CSP CSV CST

  [MotorName]
    type                str    motor class name (e.g. "MMS760400_48_C2_1")
    node_id             int    CANopen node ID (1–127)
    operation_mode      str    per-motor override (optional)
    joint_limit_min     float  rad (optional)
    joint_limit_max     float  rad (optional)
    profile_velocity    float  rad/s written to 6081h at startup (optional)
    profile_acceleration float  rad/s² written to 6083h (optional)
    profile_deceleration float  rad/s² written to 6084h (optional)
    max_torque          float  N·m written to 6072h (optional)
    homing_method             int    DS402 6098h (default -3 = torque hard-stop upward)
    homing_max_torque_permil int    max torque % during homing — 2070h sub1 (default 1000)
    homing_speed_rps          float  homing speed rad/s — 6099h sub1+sub2 (default 1.0)
    homing_acceleration_rps2  float  homing accel rad/s² — 609Ah (default 50.0)
    homing_offset_rotations   float  zero offset from hard stop in rotations — 607Ch (default 1.0)
    homing_timeout            float  seconds to wait for bit10 (default 30.0)

Notes
-----
  - Feedback is received via TPDO3 (position) and TPDO4 (velocity) automatically
    once the node is in NMT Operational state.
  - read_param/write_param use req.index as the 16-bit SDO object dictionary index
    (sub-index 0 assumed).
  - set_active_report configures the TPDO event timers (1802h sub5, 1803h sub5)
    to control how often the motor pushes feedback.
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
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


from custom_interfaces.msg import MotorFault, MotorState, PositionCSPCommand, PositionPPCommand, VelocityCommand
from custom_interfaces.srv import EnableMotor, ReadParam, SetActiveReport, SetRunMode, WriteParam
from custom_interfaces.action import EZMotionHoming, MoveToPosition
from custom_interfaces.action import SetVelocity as SetVelocityAction

from .comms import EZMotionCANComms
from .motor_base import (
    DriveState, MotorFeedback, OD, OperationMode,
    COUNTS_PER_REV,
    _rad_s_to_counts_s,
)


# ── Linear actuator conversion constants ──────────────────────────────────────
# Mechanical coupling from ref.py: 2.54 motor rotations per 1 cm of height.
# Increasing height (moving up) = motor turns in the negative direction.
_ROTATIONS_PER_CM = 2.54
_RAD_PER_MM       = _ROTATIONS_PER_CM * 2.0 * math.pi / 10.0  # ≈ 1.596 rad/mm
_HEIGHT_DIR       = -1.0   # positive height delta → negative motor position delta


# ── Motor class registry ───────────────────────────────────────────────────────

_MOTOR_CLASS_MAP = {
    'MMS760400_48_C2_1': 'ezmotion_p.mms760400_48_c2_1.MMS760400_48_C2_1',
}

# DS402 OperationMode string → enum
_MODE_STR_MAP = {m.name: m for m in OperationMode}

# SetRunMode integer → OperationMode (req.mode uses DS402 values directly)
_MODE_INT_MAP = {m.value: m for m in OperationMode}

# TPDO communication parameter objects (for event timer sub5)
_TPDO3_COMM = 0x1802
_TPDO4_COMM = 0x1803


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
            self.get_logger().fatal(f'[defaults] section missing from {config_path}')
            raise SystemExit(1)

        self._defaults       = config.pop('defaults')
        self._update_rate_hz = float(self._defaults['update_rate_hz'])
        node_name_cfg        = self._defaults.get('node_name', '')
        self._ns             = f'/{node_name_cfg}' if node_name_cfg else '~'

        self._buses:         Dict[Tuple, EZMotionCANComms] = {}
        self._motors:        Dict[str, object]             = {}
        self._motor_enabled: Dict[str, bool]               = {}
        self._motor_mode:    Dict[str, Optional[OperationMode]] = {}
        self._last_fault:    Dict[str, bool]               = {}
        self._motor_locks:   Dict[str, threading.Lock]     = {}
        self._motor_cfg:     Dict[str, dict]               = {}

        self._init_motors(config)

        # ── Publishers ─────────────────────────────────────────────────────
        self._joint_state_pub = self.create_publisher(JointState, self._topic('joint_states'), 10)
        self._state_pubs:       Dict[str, object] = {}
        self._fault_pubs:       Dict[str, object] = {}
        self._processed_state_pubs: Dict[str, object] = {}

        for name in self._motors:
            self._state_pubs[name] = self.create_publisher(
                MotorState, self._topic(f'motors/{name}/state'), 10
            )
            self._fault_pubs[name] = self.create_publisher(
                MotorFault, self._topic(f'motors/{name}/fault'), 10
            )
            self._processed_state_pubs[name] = self.create_publisher(
                MotorState, self._topic(f'motors/{name}/processed_state'), 10
            )
            self._motors[name].set_feedback_callback(
                lambda fb, n=name: self._on_feedback(n, fb)
            )

        # ── Subscribers ────────────────────────────────────────────────────
        _mode_subs = [
            (PositionPPCommand,  'cmd_position_pp',  self._on_cmd_position_pp),
            (PositionCSPCommand, 'cmd_position_csp', self._on_cmd_position_csp),
            (VelocityCommand,    'cmd_velocity',     self._on_cmd_velocity),
            (Float64,            'go_to',            self._on_go_to),
            (Float64,            'safe_vel',         self._on_safe_vel),
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
        self.create_service(EnableMotor,    self._topic('enable_motor'),      self._srv_enable_motor,      callback_group=self._cb_services)
        self.create_service(SetRunMode,     self._topic('set_run_mode'),      self._srv_set_run_mode,      callback_group=self._cb_services)
        self.create_service(SetActiveReport,self._topic('set_active_report'), self._srv_set_active_report, callback_group=self._cb_services)
        self.create_service(ReadParam,      self._topic('read_param'),        self._srv_read_param,        callback_group=self._cb_services)
        self.create_service(WriteParam,     self._topic('write_param'),       self._srv_write_param,       callback_group=self._cb_services)
        self.create_service(Trigger,        self._topic('stop_all'),          self._srv_stop_all,          callback_group=self._cb_services)
        self.create_service(Trigger,        self._topic('save_params'),       self._srv_save_params,       callback_group=self._cb_services)

        # ── Actions ────────────────────────────────────────────────────────
        self._homing_action = ActionServer(
            self, EZMotionHoming, self._topic('homing'),
            execute_callback=self._execute_homing,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_actions,
        )
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
            1.0 / self._update_rate_hz, self._update_cb, callback_group=self._cb_timer
        )

        self.get_logger().info(
            f'EZMotion MotorNode ready — {len(self._motors)} motor(s), {self._update_rate_hz:.0f} Hz'
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
            cfg.get('channel', self._defaults['channel']),
            cfg.get('bustype', self._defaults.get('bustype', 'socketcan')),
            int(cfg.get('bitrate', self._defaults.get('bitrate', 1_000_000))),
        )

    def _get_or_create_bus(self, cfg: dict) -> EZMotionCANComms:
        key = self._bus_key(cfg)
        if key not in self._buses:
            channel, bustype, bitrate = key
            self.get_logger().info(f'Opening CAN bus  channel={channel}  bitrate={bitrate}')
            bus = EZMotionCANComms(
                channel=channel,
                bustype=bustype,
                bitrate=bitrate,
                on_error=lambda exc, ch=channel: self.get_logger().error(
                    f'CAN bus error on {ch}: {exc}'
                ),
            )
            bus.start_listener()
            self._buses[key] = bus
        return self._buses[key]

    def _resolve_mode(self, cfg: dict) -> Optional[OperationMode]:
        mode_str = cfg.get('operation_mode') or self._defaults.get('operation_mode')
        if not mode_str:
            return None
        mode = _MODE_STR_MAP.get(mode_str.upper())
        if mode is None:
            self.get_logger().warning(
                f'Unknown operation_mode "{mode_str}" — valid: {list(_MODE_STR_MAP)}'
            )
        return mode

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
                    node_id    = int(cfg['node_id']),
                    comms      = bus,
                    rx_timeout = float(cfg.get('rx_timeout', self._defaults.get('rx_timeout', 0.1))),
                )
                self._motors[name]        = motor
                self._motor_enabled[name] = False
                self._motor_mode[name]    = None
                self._last_fault[name]    = False
                self._motor_locks[name]   = threading.Lock()
                self._motor_cfg[name]     = {
                    'joint_limit_min':    cfg.get('joint_limit_min'),
                    'joint_limit_max':    cfg.get('joint_limit_max'),
                    'max_vel':            cfg.get('max_vel'),
                    'max_torque':         cfg.get('max_torque'),
                    'profile_velocity':   cfg.get('profile_velocity'),
                    'profile_acceleration': cfg.get('profile_acceleration'),
                    'profile_deceleration': cfg.get('profile_deceleration'),
                    # Mode to restore after homing (resolved below)
                    'configured_mode': None,
                    # Linear actuator height limits (mm) for go_to / safe_vel
                    'top_height_mm':    cfg.get('top_height_mm',    600.0),
                    'bottom_height_mm': cfg.get('bottom_height_mm',  50.0),
                    # Physical height (mm) at the homing position (motor pos = 0).
                    # Defaults to top_height_mm since we home to the hard stop at the top.
                    'homing_height_mm': cfg.get('homing_height_mm', cfg.get('top_height_mm', 600.0)),
                    # Homing parameters
                    'homing_method':             cfg.get('homing_method', -3),
                    'homing_max_torque_permil': cfg.get('homing_max_torque_permil', 1000),
                    'homing_speed_rps':          cfg.get('homing_speed_rps', 1.0),
                    'homing_acceleration_rps2':  cfg.get('homing_acceleration_rps2', 50.0),
                    'homing_offset_rotations':   cfg.get('homing_offset_rotations', 1.0),
                    'homing_timeout':            cfg.get('homing_timeout', 30.0),
                }

                # NMT: bring node to Operational so PDOs/SDOs are active
                motor.nmt_start()
                time.sleep(0.05)  # allow boot-up frame to arrive

                # Write profile params via SDO
                mcfg = self._motor_cfg[name]
                if mcfg.get('profile_velocity') is not None:
                    motor.set_profile_velocity(float(mcfg['profile_velocity']))
                if mcfg.get('profile_acceleration') is not None:
                    motor.set_profile_acceleration(float(mcfg['profile_acceleration']))
                if mcfg.get('profile_deceleration') is not None:
                    motor.set_profile_deceleration(float(mcfg['profile_deceleration']))
                if mcfg.get('max_torque') is not None:
                    motor.set_max_torque(float(mcfg['max_torque']))

                # Set operation mode:
                # proper fault reset (0→1 rising edge) → shutdown → set mode →
                # enable → wait for state to settle → confirm → disable
                target_mode = self._resolve_mode(cfg)
                mode_label  = 'none'
                if target_mode is not None:
                    self._motor_cfg[name]['configured_mode'] = target_mode
                    motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0000)  # ensure bit7 = 0
                    motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0080)  # rising edge → fault reset
                    motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0006)  # shutdown → Ready to Switch On
                    motor.set_operation_mode(target_mode)
                    motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x000F)  # enable → Operation Enabled
                    time.sleep(0.1)                                    # allow state + mode display to settle
                    confirmed = motor.read_operation_mode()
                    motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0000)  # disable voltage
                    if confirmed == int(target_mode):
                        self._motor_mode[name] = target_mode
                        mode_label = target_mode.name
                    else:
                        actual = str(confirmed) if confirmed is not None else 'no response'
                        self.get_logger().error(
                            f'[{name}] mode mismatch — wrote {target_mode.name}, '
                            f'motor reports {actual}'
                        )

                # Configure TPDO event timer (active report)
                active_hz = float(
                    cfg.get('active_report_hz',
                            self._defaults.get('active_report_hz', 0.0))
                )
                if active_hz > 0.0:
                    timer_ms = max(1, int(1000.0 / active_hz))
                    motor.write_sdo_u16(_TPDO3_COMM, 0x05, timer_ms)
                    motor.write_sdo_u16(_TPDO4_COMM, 0x05, timer_ms)
                    self.get_logger().info(
                        f'  [{name}]  active_report={active_hz:.1f} Hz ({timer_ms} ms)'
                    )

                self.get_logger().info(
                    f'  [{name}]  type={motor_type}  node_id={cfg["node_id"]}  '
                    f'channel={self._bus_key(cfg)[0]}  mode={mode_label}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to init [{name}]: {e}')

    # ── Feedback ───────────────────────────────────────────────────────────────

    def _on_feedback(self, name: str, fb: MotorFeedback) -> None:
        """Called from motor_base on every TPDO3 (position) frame."""
        now      = self.get_clock().now().to_msg()
        user_pos = self._user_pos(name, fb.position)
        self._state_pubs[name].publish(self._build_state_msg(name, fb, user_pos, now))

        # Height / velocity in mm units — same MotorState msg, position=mm, velocity=mm/s
        h_msg              = MotorState()
        h_msg.header.stamp = now
        h_msg.name         = name
        h_msg.position     = self._motor_rad_to_height(name, fb.position)
        h_msg.velocity     = fb.velocity / _RAD_PER_MM * (-1.0 / _HEIGHT_DIR)
        h_msg.torque       = fb.torque
        h_msg.mode         = fb.op_mode
        h_msg.fault        = int(fb.fault)
        h_msg.enabled      = fb.enabled
        self._processed_state_pubs[name].publish(h_msg)

        # Fault change detection
        fault_now = fb.fault
        if fault_now != self._last_fault[name]:
            if fault_now:
                self.get_logger().error(f'[{name}] Fault — statusword=0x{fb.statusword:04X}')
                self._motor_enabled[name] = False
            else:
                self.get_logger().info(f'[{name}] Fault cleared')
            self._fault_pubs[name].publish(self._build_fault_msg(name, fb, now))
            self._last_fault[name] = fault_now

        # Keep enabled tracking in sync with what the motor reports
        if fb.enabled != self._motor_enabled.get(name, False):
            self._motor_enabled[name] = fb.enabled

    def _update_cb(self) -> None:
        if not self._motors:
            return
        now = self.get_clock().now().to_msg()
        js  = JointState()
        js.header.stamp = now
        for name, motor in self._motors.items():
            fb = motor.feedback
            js.name.append(name)
            js.position.append(self._user_pos(name, fb.position))
            js.velocity.append(fb.velocity)
            js.effort.append(fb.torque)
        self._joint_state_pub.publish(js)

    # ── Position / command helpers ─────────────────────────────────────────────

    def _user_pos(self, name: str, motor_pos: float) -> float:
        return motor_pos

    def _motor_pos(self, name: str, cmd_pos: float) -> float:
        return cmd_pos

    def _check_mode(self, name: str, required: OperationMode) -> bool:
        current = self._motor_mode.get(name)
        if current == required:
            if not self._motor_enabled.get(name, False):
                self.get_logger().warning(f'[{name}] Motor not enabled')
            return True
        self.get_logger().error(
            f'[{name}] Mode mismatch: in {current.name if current else "None"}, '
            f'expected {required.name}'
        )
        return False

    def _check_joint_limits(self, name: str, motor_pos: float) -> bool:
        lo = self._motor_cfg[name].get('joint_limit_min')
        hi = self._motor_cfg[name].get('joint_limit_max')
        if lo is not None and motor_pos < lo:
            self.get_logger().error(f'[{name}] Rejected: {motor_pos:.4f} < limit_min {lo:.4f}')
            return False
        if hi is not None and motor_pos > hi:
            self.get_logger().error(f'[{name}] Rejected: {motor_pos:.4f} > limit_max {hi:.4f}')
            return False
        return True

    def _clamp_vel(self, name: str, v: float) -> float:
        lim = self._motor_cfg[name].get('max_vel')
        if lim is not None and abs(v) > lim:
            clamped = lim if v > 0.0 else -lim
            self.get_logger().warning(f'[{name}] Velocity clamped to {clamped:.3f} rad/s')
            return clamped
        return v

    # ── Height / velocity conversion helpers ──────────────────────────────────

    def _height_to_motor_rad(self, name: str, height_mm: float) -> float:
        """Absolute height (mm from bottom) → motor position (rad, relative to homing zero)."""
        homing_height = self._motor_cfg[name]['homing_height_mm']
        delta_mm = height_mm - homing_height
        return _HEIGHT_DIR * delta_mm * _RAD_PER_MM

    def _motor_rad_to_height(self, name: str, pos_rad: float) -> float:
        """Motor position (rad) → absolute height (mm from bottom)."""
        homing_height = self._motor_cfg[name]['homing_height_mm']
        return homing_height + (_HEIGHT_DIR * pos_rad / _RAD_PER_MM)

    # ── go_to / safe_vel subscribers ──────────────────────────────────────────

    def _on_go_to(self, msg: Float64, name: str) -> None:
        """PP mode: move to absolute height in mm from bottom."""
        height_mm  = msg.data
        top        = self._motor_cfg[name]['top_height_mm']
        bottom     = self._motor_cfg[name]['bottom_height_mm']

        if height_mm > top:
            self.get_logger().error(
                f'[{name}] go_to {height_mm:.1f} mm > top_height {top:.1f} mm — rejected'
            )
            return
        if height_mm < bottom:
            self.get_logger().error(
                f'[{name}] go_to {height_mm:.1f} mm < bottom_height {bottom:.1f} mm — rejected'
            )
            return

        if not self._check_mode(name, OperationMode.PP):
            return

        motor_pos = self._height_to_motor_rad(name, height_mm)
        with self._motor_locks[name]:
            try:
                motor = self._motors[name]
                profile_vel = self._motor_cfg[name].get('profile_velocity')
                if profile_vel is not None:
                    motor.set_profile_velocity(float(profile_vel))
                motor.trigger_move_pp(motor_pos)
            except Exception as e:
                self.get_logger().error(f'[{name}] go_to error: {e}')

    def _on_safe_vel(self, msg: Float64, name: str) -> None:
        """PP mode: set velocity (rad/s) toward the appropriate height limit as target."""
        vel_rad_s = msg.data
        top       = self._motor_cfg[name]['top_height_mm']
        bottom    = self._motor_cfg[name]['bottom_height_mm']

        if not self._check_mode(name, OperationMode.PP):
            return

        with self._motor_locks[name]:
            try:
                motor = self._motors[name]
                if vel_rad_s == 0.0:
                    profile_vel = self._motor_cfg[name].get('profile_velocity')
                    if profile_vel is not None:
                        motor.set_profile_velocity(float(profile_vel))
                    motor.trigger_move_pp(motor.feedback.position)
                    return
                motor.set_profile_velocity(abs(vel_rad_s))
                if vel_rad_s > 0:
                    current_height = self._motor_rad_to_height(name, motor.feedback.position)
                    if current_height >= top:
                        return
                    motor.trigger_move_pp(self._height_to_motor_rad(name, top))
                else:
                    current_height = self._motor_rad_to_height(name, motor.feedback.position)
                    if current_height <= bottom:
                        return
                    motor.trigger_move_pp(self._height_to_motor_rad(name, bottom))
            except Exception as e:
                self.get_logger().error(f'[{name}] safe_vel error: {e}')

    # ── Command subscribers ────────────────────────────────────────────────────

    def _on_cmd_position_pp(self, msg: PositionPPCommand, name: str) -> None:
        if not self._check_mode(name, OperationMode.PP):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        with self._motor_locks[name]:
            try:
                motor = self._motors[name]
                if msg.speed > 0.0:
                    motor.set_profile_velocity(self._clamp_vel(name, msg.speed))
                if msg.acceleration > 0.0:
                    motor.set_profile_acceleration(msg.acceleration)
                if msg.deceleration > 0.0:
                    motor.set_profile_deceleration(msg.deceleration)
                motor.trigger_move_pp(motor_pos)
            except Exception as e:
                self.get_logger().error(f'[{name}] cmd_position_pp error: {e}')

    def _on_cmd_position_csp(self, msg: PositionCSPCommand, name: str) -> None:
        if not self._check_mode(name, OperationMode.CSP):
            return
        motor_pos = self._motor_pos(name, msg.position)
        if not self._check_joint_limits(name, motor_pos):
            return
        with self._motor_locks[name]:
            try:
                self._motors[name].set_cyclic_position(motor_pos)
            except Exception as e:
                self.get_logger().error(f'[{name}] cmd_position_csp error: {e}')

    def _on_cmd_velocity(self, msg: VelocityCommand, name: str) -> None:
        mode = self._motor_mode.get(name)
        if mode not in (OperationMode.PV, OperationMode.CSV):
            self.get_logger().error(
                f'[{name}] cmd_velocity requires PV or CSV mode, currently {mode}'
            )
            return
        vel = self._clamp_vel(name, msg.velocity)
        with self._motor_locks[name]:
            try:
                if mode == OperationMode.PV:
                    self._motors[name].set_target_velocity_pv(vel)
                else:
                    self._motors[name].set_cyclic_velocity(vel)
            except Exception as e:
                self.get_logger().error(f'[{name}] cmd_velocity error: {e}')

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
                        if req.clear_fault:
                            motor.fault_reset()
                        fb = motor.disable()
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

        mode = _MODE_INT_MAP.get(req.mode)
        if mode is None:
            res.success = False
            res.message = (
                f'Invalid mode {req.mode}. '
                f'Valid DS402 values: {sorted(_MODE_INT_MAP)}'
            )
            return res

        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    if req.automatic_enable_disable:
                        motor.disable()
                        self._motor_enabled[name] = False
                    motor.set_operation_mode(mode)
                    confirmed = motor.read_operation_mode()
                    if confirmed != int(mode):
                        actual = str(confirmed) if confirmed is not None else 'no response'
                        failed.append(f'{name}: mode mismatch — wrote {mode.name}, got {actual}')
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
            res.message = f'Mode set to {mode.name}'
        return res

    def _srv_set_active_report(self, req: SetActiveReport.Request, res: SetActiveReport.Response):
        """
        Configure TPDO3/TPDO4 event timers so the motor pushes feedback at the
        requested rate, then ensure the node is in NMT Operational state.

        TPDO3 (position+status): 1802h sub5 = event timer ms (uint16)
        TPDO4 (velocity+status): 1803h sub5 = event timer ms (uint16)
        """
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res

        if req.enable and req.hz <= 0.0:
            res.success = False
            res.message = f'hz must be > 0 (got {req.hz})'
            return res

        timer_ms = max(1, int(1000.0 / req.hz)) if req.enable else 0

        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    motor.write_sdo_u16(_TPDO3_COMM, 0x05, timer_ms)
                    motor.write_sdo_u16(_TPDO4_COMM, 0x05, timer_ms)
                    if req.enable:
                        motor.nmt_start()
                    else:
                        motor.nmt_pre_operational()
            except Exception as e:
                failed.append(f'{name}: {e}')

        if failed:
            res.success = False
            res.message = 'Errors — ' + ', '.join(failed)
        else:
            res.success = True
            state = f'enabled @ {req.hz:.1f} Hz ({timer_ms} ms)' if req.enable else 'disabled'
            res.message = f'Active report {state} for {list(motors)}'
        return res

    def _srv_read_param(self, req: ReadParam.Request, res: ReadParam.Response):
        motor = self._motors.get(req.name)
        if motor is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found. "all" not supported for read_param.'
            return res
        size = req.size if req.size in (1, 2, 4) else 4
        try:
            with self._motor_locks[req.name]:
                if size == 1:
                    raw = motor.read_sdo_u8(req.index, req.sub_index)
                elif size == 2:
                    raw = motor.read_sdo_u16(req.index, req.sub_index)
                else:
                    raw = motor.read_sdo_u32(req.index, req.sub_index)
            if raw is None:
                res.success = False
                res.message = f'No SDO response for 0x{req.index:04X}:{req.sub_index:02X}'
            else:
                res.success = True
                res.message = 'OK'
                res.value   = int(raw)
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

        size   = req.size if req.size in (1, 2, 4) else 4
        failed = []
        for name, motor in motors.items():
            try:
                with self._motor_locks[name]:
                    if size == 1:
                        motor.write_sdo_u8(req.index, req.sub_index, int(req.value) & 0xFF)
                    elif size == 2:
                        motor.write_sdo_u16(req.index, req.sub_index, int(req.value) & 0xFFFF)
                    else:
                        motor.write_sdo_u32(req.index, req.sub_index, int(req.value) & 0xFFFFFFFF)
                    if req.persist:
                        motor.save_params()
            except Exception as e:
                failed.append(f'{name}: {e}')

        res.success = not bool(failed)
        res.message = ('OK' + (' (persisted)' if req.persist else '')) if not failed \
                      else 'Errors — ' + ', '.join(failed)
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
        res.message = f'{len(self._motors)} motor(s) stopped' if not failed \
                      else 'Errors — ' + ', '.join(failed)
        return res

    def _srv_save_params(self, _req: Trigger.Request, res: Trigger.Response):
        failed = []
        for name, motor in self._motors.items():
            with self._motor_locks[name]:
                try:
                    was_enabled = self._motor_enabled[name]
                    if was_enabled:
                        motor.disable()
                        self._motor_enabled[name] = False
                    motor.save_params()
                    time.sleep(0.05)
                    if was_enabled:
                        motor.enable()
                        self._motor_enabled[name] = True
                except Exception as e:
                    failed.append(f'{name}: {e}')
        res.success = not bool(failed)
        res.message = 'Parameters saved' if not failed else 'Errors — ' + ', '.join(failed)
        return res

    def _home_motor(self, name: str, motor) -> tuple:
        """
        Execute DS402 homing sequence for one motor.

        Sequence (matching ref.py):
          1. Enable          (6040h = 0x000F)
          2. Shutdown        (6040h = 0x0006) → Ready to Switch On
          3. Set mode = HM   (6060h = 6)
          4. Enable          (6040h = 0x000F) → Operation Enabled in HM
          5. Homing method   (6098h)           default -3 = torque-limit hard stop upward
          6. Max homing torque (2070h sub1)    default 1000 = 100% rated
          7. Homing speed (search) (6099h sub1)
          8. Homing speed (zero)   (6099h sub2)
          9. Homing acceleration   (609Ah)
         10. Homing offset         (607Ch)     shift zero away from hard stop
         11. Trigger               (6040h = 0x001F)

        Polls statusword bit 12 (homing attained) until done or timeout.
        Returns (success, message).
        """
        mcfg = self._motor_cfg[name]

        homing_method   = int(mcfg['homing_method'])
        torque_pct      = int(mcfg['homing_max_torque_permil'])
        speed_counts    = abs(int(_rad_s_to_counts_s(float(mcfg['homing_speed_rps']))))
        accel_counts    = abs(int(_rad_s_to_counts_s(float(mcfg['homing_acceleration_rps2']))))
        offset_counts   = int(float(mcfg['homing_offset_rotations']) * COUNTS_PER_REV)
        timeout         = float(mcfg['homing_timeout'])

        # Steps 1–4: transition to Operation Enabled in Homing mode
        motor.write_sdo_u16(OD.CTRL_WORD,    0x00, 0x000F)              # Enable
        motor.write_sdo_u16(OD.CTRL_WORD,    0x00, 0x0006)              # Shutdown
        motor.write_sdo_u8( OD.MODES_OF_OP,  0x00, OperationMode.HM.value & 0xFF)
        motor.write_sdo_u16(OD.CTRL_WORD,    0x00, 0x000F)              # Enable

        # Steps 5–10: homing parameters
        motor.write_sdo_u8( OD.HOMING_METHOD,  0x00, homing_method & 0xFF)
        motor.write_sdo_u16(OD.HOMING_TORQUE,  0x01, torque_pct)
        motor.write_sdo_u32(OD.HOMING_SPEED,   0x01, speed_counts)     # search speed
        motor.write_sdo_u32(OD.HOMING_SPEED,   0x02, speed_counts)     # zero speed
        motor.write_sdo_u32(OD.HOMING_ACCEL,   0x00, accel_counts)
        motor.write_sdo_s32(OD.HOME_OFFSET,    0x00, offset_counts)

        # Step 11: trigger homing (bit 4 = new setpoint / start homing)
        motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x001F)

        self._motor_mode[name]    = OperationMode.HM
        self._motor_enabled[name] = True
        self.get_logger().info(f'[{name}] Homing started (method={homing_method})')

        # Poll statusword bit 12 (homing attained) — updated via TPDO
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sw = motor.feedback.statusword
            if sw & (1 << 13):                        # homing error (bit 13)
                return False, f'Homing error (statusword=0x{sw:04X})'
            if sw & (1 << 3):                         # generic fault
                return False, f'Fault during homing (statusword=0x{sw:04X})'
            if sw & (1 << 12):                        # homing attained (bit 12)
                return True, 'Homing complete'
            time.sleep(0.05)

        return False, f'Homing timeout after {timeout:.1f}s'

    def _execute_homing(self, goal_handle) -> EZMotionHoming.Result:
        """
        DS402 homing action. goal.motor_name = specific motor, or '' for all.

        For each motor: Enable → Shutdown → HM mode → Enable → configure →
        trigger → poll bit 10. Publishes feedback per motor.
        """
        req    = goal_handle.request
        result = EZMotionHoming.Result()

        motors = self._resolve_motors(req.motor_name if req.motor_name else 'all')
        if motors is None:
            result.success = False
            result.message = f'Motor {req.motor_name!r} not found'
            result.homed_motors = []
            goal_handle.abort()
            return result

        homed  = []
        failed = []
        start  = time.monotonic()

        for name, motor in motors.items():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success      = False
                result.message      = 'Cancelled'
                result.homed_motors = homed
                return result

            fb = EZMotionHoming.Feedback()
            fb.motor_name   = name
            fb.phase        = 'starting'
            fb.elapsed_time = time.monotonic() - start
            fb.statusword   = motor.feedback.statusword
            goal_handle.publish_feedback(fb)

            with self._motor_locks[name]:
                try:
                    ok, msg = self._home_motor(name, motor)
                    if ok:
                        # Restore configured operation mode using same reliable sequence as init
                        configured = self._motor_cfg[name].get('configured_mode')
                        if configured is not None:
                            motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0000)  # ensure bit7 = 0
                            motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0080)  # fault reset
                            motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x0006)  # shutdown → Ready to Switch On
                            motor.set_operation_mode(configured)
                            motor.write_sdo_u16(OD.CTRL_WORD, 0x00, 0x000F)  # enable → Operation Enabled
                            time.sleep(0.1)
                            confirmed = motor.read_operation_mode()
                            if confirmed == int(configured):
                                self._motor_mode[name]    = configured
                                self._motor_enabled[name] = True
                                self.get_logger().info(
                                    f'[{name}] Restored mode {configured.name} after homing'
                                )
                            else:
                                actual = str(confirmed) if confirmed is not None else 'no response'
                                self.get_logger().error(
                                    f'[{name}] Mode restore failed after homing — '
                                    f'wrote {configured.name}, motor reports {actual}'
                                )
                except Exception as e:
                    ok, msg = False, str(e)

            fb.phase        = 'complete' if ok else 'error'
            fb.elapsed_time = time.monotonic() - start
            fb.statusword   = motor.feedback.statusword
            goal_handle.publish_feedback(fb)

            if ok:
                homed.append(name)
                self.get_logger().info(f'[{name}] Homing complete')
            else:
                failed.append(f'{name}: {msg}')
                self.get_logger().error(f'[{name}] Homing failed: {msg}')

        result.homed_motors = homed
        if failed:
            result.success = False
            result.message = 'Errors — ' + ', '.join(failed)
            goal_handle.abort()
        else:
            result.success = True
            result.message = f'{len(homed)} motor(s) homed'
            goal_handle.succeed()

        return result

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
            motor.set_operation_mode(OperationMode.PP)
            self._motor_mode[req.name] = OperationMode.PP
            motor.enable()
            self._motor_enabled[req.name] = True
            if speed > 0.0:
                motor.set_profile_velocity(speed)
            motor.trigger_move_pp(req.target_position)

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
            error   = abs(req.target_position - self._user_pos(req.name, fb.position))

            feedback.current_position = self._user_pos(req.name, fb.position)
            feedback.position_error   = error
            feedback.elapsed_time     = elapsed
            goal_handle.publish_feedback(feedback)

            if error <= req.tolerance:
                goal_handle.succeed()
                result.success        = True
                result.message        = 'Target reached'
                result.final_position = self._user_pos(req.name, fb.position)
                result.elapsed_time   = elapsed
                return result

            if req.timeout > 0.0 and elapsed >= req.timeout:
                goal_handle.abort()
                result.success        = False
                result.message        = f'Timeout after {elapsed:.2f}s, error={error:.4f} rad'
                result.final_position = self._user_pos(req.name, fb.position)
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
            motor.set_operation_mode(OperationMode.PV)
            self._motor_mode[req.name] = OperationMode.PV
            motor.enable()
            self._motor_enabled[req.name] = True
            motor.set_target_velocity_pv(req.target_velocity)

        start     = time.monotonic()
        vel_sum   = 0.0
        vel_count = 0
        feedback  = SetVelocityAction.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._motor_locks[req.name]:
                    motor.set_target_velocity_pv(0.0)
                goal_handle.canceled()
                result.success          = False
                result.message          = 'Cancelled'
                result.average_velocity = vel_sum / max(vel_count, 1)
                result.elapsed_time     = time.monotonic() - start
                return result

            fb        = motor.feedback
            elapsed   = time.monotonic() - start
            vel_sum   += fb.velocity
            vel_count += 1

            feedback.current_velocity = fb.velocity
            feedback.current_torque   = fb.torque
            feedback.elapsed_time     = elapsed
            goal_handle.publish_feedback(feedback)

            if req.duration > 0.0 and elapsed >= req.duration:
                with self._motor_locks[req.name]:
                    motor.set_target_velocity_pv(0.0)
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
        msg.temperature  = 0.0             # not available via default TPDOs
        msg.mode         = fb.op_mode      # DS402 modes of operation display (6061h)
        msg.fault        = int(fb.fault)   # statusword bit 3
        msg.enabled      = fb.enabled
        return msg

    def _build_fault_msg(self, name: str, fb: MotorFeedback, stamp) -> MotorFault:
        msg              = MotorFault()
        msg.header.stamp = stamp
        msg.name         = name
        msg.fault_code   = fb.statusword   # raw statusword as fault code
        msg.warning_code = 0
        # DS402 statusword bit 3 = generic fault. Per-fault details require reading
        # the manufacturer error status register (0x200B sub 0x0B) via SDO on demand.
        msg.over_temp      = False
        msg.driver_ic      = False
        msg.undervoltage   = False
        msg.overvoltage    = False
        msg.b_phase_oc     = False
        msg.c_phase_oc     = False
        msg.a_phase_oc     = False
        msg.encoder_uncal  = False
        msg.hw_id_fault    = False
        msg.pos_init_fault = False
        msg.stall_overload = False
        msg.over_temp_warning = False
        return msg

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        print('[ezmotion_node] Shutting down — disabling motors …')
        for name, motor in self._motors.items():
            try:
                motor.disable()
            except Exception as e:
                print(f'[ezmotion_node] Could not disable [{name}]: {e}')
        for key, bus in self._buses.items():
            try:
                bus.close()
            except Exception as e:
                print(f'[ezmotion_node] Could not close bus {key}: {e}')
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    _tmp = rclpy.create_node(f'_ezmotion_cfg_reader_{os.getpid()}')
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
