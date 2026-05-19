#!/usr/bin/env python3
"""
sim_motor_node.py — Drop-in simulation replacement for motor_node.

Exposes the identical ROS2 interface (topics / services / actions) as the real
motor_node but routes everything through Gazebo instead of CAN hardware.

State:   reads  joint_state_broadcaster output  →  publishes MotorState / JointState
Commands: routes cmd_position_pp / cmd_position_csp / cmd_velocity
          →  JointTrajectory on the configured sim controller topic

Hardware-only services (CAN config, scan, read/write firmware params) respond
with a clear "sim mode" message so callers don't hang.
"""

import math
import threading
import time
import tomllib
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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


# ── Local RunMode (avoids importing hardware-dependent robstride_p) ───────────

class RunMode(IntEnum):
    OPERATION    = 0
    POSITION_PP  = 1
    VELOCITY     = 2
    CURRENT      = 3
    POSITION_CSP = 5


# ── Per-motor runtime state ────────────────────────────────────────────────────

@dataclass
class _State:
    position:    float = 0.0
    velocity:    float = 0.0
    torque:      float = 0.0
    initialized: bool  = False   # True once first sim joint state arrives


@dataclass
class _MotorCfg:
    joint_limit_min:  Optional[float] = None
    joint_limit_max:  Optional[float] = None
    motor_homing_pos: float           = 0.0
    max_vel:          Optional[float] = None
    max_accel:        Optional[float] = None
    max_decel:        Optional[float] = None


# ── Node ───────────────────────────────────────────────────────────────────────

class SimMotorNode(Node):
    """Simulated motor_node: same interface, Gazebo back-end."""

    def __init__(self, node_name: str = 'sim_motor_node'):
        super().__init__(node_name)

        self._cb_timer    = MutuallyExclusiveCallbackGroup()
        self._cb_services = ReentrantCallbackGroup()
        self._cb_actions  = ReentrantCallbackGroup()
        self._cb_subs     = ReentrantCallbackGroup()

        self.declare_parameter('config_path', '')
        cfg_path = self.get_parameter('config_path').value
        if not cfg_path:
            raise RuntimeError('config_path parameter is required')

        with open(cfg_path, 'rb') as f:
            raw = tomllib.load(f)

        defaults = raw.pop('defaults', {})
        self._update_rate_hz   = float(defaults.get('update_rate_hz', 100.0))
        use_prefix             = bool(defaults.get('use_node_name_as_topic_base', True))
        self._ns               = '~' if use_prefix else ''
        self._sim_js_topic     = defaults.get('sim_joint_states_topic', '/joint_states')
        self._sim_ctrl_topic   = defaults.get('sim_controller_topic', '')
        self._sim_ctrl_type    = defaults.get('sim_controller_type', 'trajectory')  # 'trajectory' | 'velocity'
        self._traj_ns          = int(float(defaults.get('trajectory_time_sec', 0.05)) * 1e9)

        # ── Per-motor state ─────────────────────────────────────────────────
        self._motors:        List[str]                   = list(raw.keys())
        self._state:         Dict[str, _State]           = {}
        self._desired_pos:   Dict[str, float]            = {}
        self._desired_vel:   Dict[str, float]            = {}   # velocity mode
        self._motor_cfg:     Dict[str, _MotorCfg]       = {}
        self._motor_enabled: Dict[str, bool]             = {}
        self._motor_mode:    Dict[str, Optional[RunMode]]= {}
        self._locks:         Dict[str, threading.Lock]  = {}

        for name, cfg in raw.items():
            self._state[name]         = _State()
            self._desired_pos[name]   = 0.0
            self._desired_vel[name]   = 0.0
            self._motor_enabled[name] = False
            self._motor_mode[name]    = None
            self._locks[name]         = threading.Lock()
            self._motor_cfg[name]     = _MotorCfg(
                joint_limit_min  = cfg.get('joint_limit_min'),
                joint_limit_max  = cfg.get('joint_limit_max'),
                motor_homing_pos = float(cfg.get('motor_homing_pos', 0.0)),
                max_vel          = cfg.get('max_vel'),
                max_accel        = cfg.get('max_accel'),
                max_decel        = cfg.get('max_decel'),
            )

        self.get_logger().info(
            f'SimMotorNode — {len(self._motors)} motors: {self._motors}'
        )

        # ── Sim state subscriber ────────────────────────────────────────────
        self._sim_js_sub = self.create_subscription(
            JointState, self._sim_js_topic, self._on_sim_joint_states, 10,
            callback_group=self._cb_subs,
        )

        # ── Sim controller publisher ────────────────────────────────────────
        if not self._sim_ctrl_topic:
            raise RuntimeError('defaults.sim_controller_topic is required in config')
        if self._sim_ctrl_type == 'velocity':
            self._vel_pub  = self.create_publisher(Float64MultiArray, self._sim_ctrl_topic, 10)
            self._traj_pub = None
        else:
            self._traj_pub = self.create_publisher(JointTrajectory, self._sim_ctrl_topic, 10)
            self._vel_pub  = None

        # ── Real-robot-compatible publishers ───────────────────────────────
        self._js_pub     = self.create_publisher(JointState, self._t('joint_states'), 10)
        self._state_pubs: Dict[str, object] = {}
        self._fault_pubs: Dict[str, object] = {}
        for name in self._motors:
            self._state_pubs[name] = self.create_publisher(
                MotorState, self._t(f'motors/{name}/state'), 10
            )
            self._fault_pubs[name] = self.create_publisher(
                MotorFault, self._t(f'motors/{name}/fault'), 10
            )

        # ── Command subscribers ─────────────────────────────────────────────
        _subs = [
            (OperationCommand,   'cmd_operation',    self._on_cmd_operation),
            (PositionPPCommand,  'cmd_position_pp',  self._on_cmd_position_pp),
            (VelocityCommand,    'cmd_velocity',     self._on_cmd_velocity),
            (CurrentCommand,     'cmd_current',      self._on_cmd_current),
            (PositionCSPCommand, 'cmd_position_csp', self._on_cmd_position_csp),
        ]
        for name in self._motors:
            for msg_type, suffix, cb in _subs:
                self.create_subscription(
                    msg_type, self._t(f'motors/{name}/{suffix}'),
                    lambda msg, n=name, fn=cb: fn(msg, n),
                    10, callback_group=self._cb_subs,
                )

        # ── Services ───────────────────────────────────────────────────────
        _srvs = [
            (EnableMotor,     'enable_motor',      self._srv_enable_motor),
            (SetRunMode,      'set_run_mode',      self._srv_set_run_mode),
            (SetZeroPosition, 'set_zero_position', self._srv_set_zero_position),
            (ReadParam,       'read_param',        self._srv_read_param),
            (WriteParam,      'write_param',       self._srv_write_param),
            (Help,            'help',              self._srv_help),
            (GetCanConfig,    'get_can_config',    self._srv_get_can_config),
            (SetCanConfig,    'set_can_config',    self._srv_set_can_config),
            (SetActiveReport, 'set_active_report', self._srv_set_active_report),
            (MotorParam,      'motor_param',       self._srv_motor_param),
        ]
        for srv_type, name, cb in _srvs:
            self.create_service(srv_type, self._t(name), cb,
                                callback_group=self._cb_services)
        self.create_service(Trigger, self._t('homing'),      self._srv_homing,    callback_group=self._cb_services)
        self.create_service(Trigger, self._t('stop_all'),    self._srv_stop_all,  callback_group=self._cb_services)
        self.create_service(Trigger, self._t('scan_motors'), self._srv_scan_motors, callback_group=self._cb_services)

        # ── Actions ─────────────────────────────────────────────────────────
        ActionServer(self, MoveToPosition,    self._t('move_to_position'),
                     execute_callback=self._execute_move_to_position,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self._cb_actions)
        ActionServer(self, SetVelocityAction, self._t('set_velocity'),
                     execute_callback=self._execute_set_velocity,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self._cb_actions)

        # ── Timer ───────────────────────────────────────────────────────────
        self._timer = self.create_timer(
            1.0 / self._update_rate_hz, self._update_cb, callback_group=self._cb_timer
        )

        self.get_logger().info(
            f'Ready — reading sim states from {self._sim_js_topic}, '
            f'commanding {self._sim_ctrl_topic}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _t(self, suffix: str) -> str:
        return f'{self._ns}/{suffix}'

    def _user_pos(self, name: str, pos: float) -> float:
        return pos - self._motor_cfg[name].motor_homing_pos

    def _motor_pos(self, name: str, user_pos: float) -> float:
        return user_pos + self._motor_cfg[name].motor_homing_pos

    def _resolve_motors(self, name: str) -> Optional[List[str]]:
        if name == 'all':
            return list(self._motors)
        if name not in self._motors:
            return None
        return [name]

    def _check_joint_limits(self, name: str, user_pos: float) -> bool:
        cfg = self._motor_cfg[name]
        if cfg.joint_limit_min is not None and user_pos < cfg.joint_limit_min:
            self.get_logger().error(
                f'[{name}] Rejected: {user_pos:.4f} < joint_limit_min {cfg.joint_limit_min:.4f}'
            )
            return False
        if cfg.joint_limit_max is not None and user_pos > cfg.joint_limit_max:
            self.get_logger().error(
                f'[{name}] Rejected: {user_pos:.4f} > joint_limit_max {cfg.joint_limit_max:.4f}'
            )
            return False
        return True

    def _clamp_vel(self, name: str, v: float) -> float:
        lim = self._motor_cfg[name].max_vel
        if lim is not None and abs(v) > lim:
            return math.copysign(lim, v)
        return v

    def _send_trajectory(self) -> None:
        """Publish desired positions or velocities to the sim controller."""
        if self._sim_ctrl_type == 'velocity':
            self._send_velocity_cmd()
            return

        traj              = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = list(self._motors)

        pt           = JointTrajectoryPoint()
        pt.positions = [self._motor_pos(n, self._desired_pos[n]) for n in self._motors]
        pt.time_from_start = Duration(
            sec     = self._traj_ns // 1_000_000_000,
            nanosec = self._traj_ns  % 1_000_000_000,
        )
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _send_velocity_cmd(self) -> None:
        """Publish Float64MultiArray velocities in motor list order."""
        cmd      = Float64MultiArray()
        cmd.data = [self._desired_vel[n] for n in self._motors]
        self._vel_pub.publish(cmd)

    # ── Sim state subscriber ──────────────────────────────────────────────────

    def _on_sim_joint_states(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for motor_name in self._motors:
            idx = name_to_idx.get(motor_name)
            if idx is None:
                continue
            s = self._state[motor_name]
            pos = msg.position[idx] if idx < len(msg.position) else 0.0
            vel = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
            eff = msg.effort[idx]   if idx < len(msg.effort)   else 0.0
            with self._locks[motor_name]:
                if not s.initialized:
                    # Seed desired so first command doesn't jump from 0
                    self._desired_pos[motor_name] = self._user_pos(motor_name, pos)
                    s.initialized = True
                s.position = self._user_pos(motor_name, pos)
                s.velocity = vel
                s.torque   = eff

    # ── Timer ─────────────────────────────────────────────────────────────────

    def _update_cb(self) -> None:
        now = self.get_clock().now().to_msg()
        js  = JointState()
        js.header.stamp = now

        for name in self._motors:
            s = self._state[name]

            # Velocity mode on trajectory controllers: advance desired_pos each tick.
            # Velocity controllers (wheels) handle velocity directly via _send_velocity_cmd.
            if (self._sim_ctrl_type == 'trajectory'
                    and self._motor_mode.get(name) == RunMode.VELOCITY
                    and self._motor_enabled.get(name, False)
                    and self._desired_vel[name] != 0.0):
                dt  = 1.0 / self._update_rate_hz
                vel = self._clamp_vel(name, self._desired_vel[name])
                new_pos = self._desired_pos[name] + vel * dt
                if self._check_joint_limits(name, new_pos):
                    self._desired_pos[name] = new_pos
                else:
                    self._desired_vel[name] = 0.0
                self._send_trajectory()

            js.name.append(name)
            js.position.append(s.position)
            js.velocity.append(s.velocity)
            js.effort.append(s.torque)

            state_msg              = MotorState()
            state_msg.header.stamp = now
            state_msg.name         = name
            state_msg.position     = s.position
            state_msg.velocity     = s.velocity
            state_msg.torque       = s.torque
            state_msg.temperature  = 0.0
            state_msg.mode         = int(self._motor_mode[name]) if self._motor_mode[name] is not None else 0
            state_msg.fault        = 0
            state_msg.enabled      = self._motor_enabled.get(name, False)
            self._state_pubs[name].publish(state_msg)

        self._js_pub.publish(js)

    # ── Command subscribers ───────────────────────────────────────────────────

    def _on_cmd_position_pp(self, msg: PositionPPCommand, name: str) -> None:
        if not self._check_joint_limits(name, msg.position):
            return
        self._desired_pos[name] = msg.position
        self._send_trajectory()

    def _on_cmd_position_csp(self, msg: PositionCSPCommand, name: str) -> None:
        if not self._check_joint_limits(name, msg.position):
            return
        self._desired_pos[name] = msg.position
        self._send_trajectory()

    def _on_cmd_velocity(self, msg: VelocityCommand, name: str) -> None:
        self._desired_vel[name] = self._clamp_vel(name, msg.velocity)
        if self._sim_ctrl_type == 'velocity':
            self._send_velocity_cmd()

    def _on_cmd_operation(self, msg: OperationCommand, name: str) -> None:
        if not self._check_joint_limits(name, msg.position):
            return
        self._desired_pos[name] = msg.position
        self._send_trajectory()

    def _on_cmd_current(self, _msg: CurrentCommand, name: str) -> None:
        self.get_logger().warning(f'[{name}] Current mode not supported in sim — ignored')

    # ── Services ─────────────────────────────────────────────────────────────

    def _srv_enable_motor(self, req: EnableMotor.Request, res: EnableMotor.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res
        for name in motors:
            self._motor_enabled[name] = req.enable
            if not req.enable:
                self._desired_vel[name] = 0.0
        res.success = True
        res.message = 'enabled' if req.enable else 'disabled'
        if req.name != 'all':
            s = self._state[req.name]
            res.position = s.position
            res.velocity = s.velocity
            res.torque   = s.torque
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
            res.message = f'Invalid mode {req.mode}'
            return res
        for name in motors:
            if req.automatic_enable_disable:
                self._motor_enabled[name] = False
            self._motor_mode[name] = mode
            if req.automatic_enable_disable:
                self._motor_enabled[name] = True
        res.success = True
        res.message = f'Mode set to {mode.name}'
        return res

    def _srv_set_zero_position(self, req: SetZeroPosition.Request, res: SetZeroPosition.Response):
        motors = self._resolve_motors(req.name)
        if motors is None:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            return res
        for name in motors:
            s = self._state[name]
            # Rebase homing_pos so current position becomes user 0
            self._motor_cfg[name].motor_homing_pos += s.position
            self._desired_pos[name] = 0.0
        res.success = True
        res.message = 'Zero position set (sim)'
        return res

    def _srv_motor_param(self, req: MotorParam.Request, res: MotorParam.Response):
        if req.name not in self._motors:
            res.success = False
            res.message = f'Motor {req.name!r} not found'
            res.value   = float('nan')
            return res
        cfg   = self._motor_cfg[req.name]
        param = req.param.lower()
        _map  = {
            'joint_limit_min':  'joint_limit_min',
            'joint_limit_max':  'joint_limit_max',
            'motor_homing_pos': 'motor_homing_pos',
            'max_vel':          'max_vel',
            'max_accel':        'max_accel',
            'max_decel':        'max_decel',
        }
        attr = _map.get(param)
        if attr is None:
            res.success = False
            res.message = f'Unknown param {req.param!r} (sim). Valid: {list(_map)}'
            res.value   = float('nan')
            return res
        if req.set:
            setattr(cfg, attr, req.value)
            res.value = req.value
        else:
            v = getattr(cfg, attr)
            res.value = float(v) if v is not None else float('nan')
        res.success = True
        res.message = 'OK'
        return res

    def _srv_homing(self, _req: Trigger.Request, res: Trigger.Response):
        for name in self._motors:
            self._desired_pos[name] = 0.0
        self._send_trajectory()
        res.success = True
        res.message = f'Homing commanded for {self._motors} (sim)'
        return res

    def _srv_stop_all(self, _req: Trigger.Request, res: Trigger.Response):
        for name in self._motors:
            self._desired_pos[name] = self._state[name].position
            self._desired_vel[name] = 0.0
            self._motor_enabled[name] = False
        self._send_trajectory()
        res.success = True
        res.message = f'{len(self._motors)} motor(s) stopped (sim — holding position)'
        return res

    def _srv_scan_motors(self, _req: Trigger.Request, res: Trigger.Response):
        lines = [f'{n}: sim (no CAN)' for n in self._motors]
        res.success = True
        res.message = '\n'.join(lines)
        return res

    def _srv_set_active_report(self, req: SetActiveReport.Request, res: SetActiveReport.Response):
        if req.hz > 0.0:
            self._timer.cancel()
            self._timer = self.create_timer(
                1.0 / req.hz, self._update_cb, callback_group=self._cb_timer
            )
        res.success = True
        res.message = f'Update rate set to {req.hz:.1f} Hz (sim)'
        return res

    # Hardware-only stubs ─────────────────────────────────────────────────────

    def _srv_read_param(self, _req: ReadParam.Request, res: ReadParam.Response):
        res.success = False
        res.message = 'read_param not supported in sim (no hardware)'
        res.value   = float('nan')
        return res

    def _srv_write_param(self, _req: WriteParam.Request, res: WriteParam.Response):
        res.success = False
        res.message = 'write_param not supported in sim (no hardware)'
        return res

    def _srv_help(self, _req: Help.Request, res: Help.Response):
        res.codes        = []
        res.names        = ['joint_limit_min', 'joint_limit_max', 'motor_homing_pos', 'max_vel']
        res.types        = ['float'] * 4
        res.access       = ['R_W'] * 4
        res.descriptions = [
            'Minimum joint position (rad)',
            'Maximum joint position (rad)',
            'Homing position offset (rad)',
            'Maximum velocity (rad/s)',
        ]
        return res

    def _srv_get_can_config(self, _req: GetCanConfig.Request, res: GetCanConfig.Response):
        res.success   = False
        res.message   = 'No CAN hardware in sim'
        res.can_id    = 0
        res.baud_flag = 0
        res.baud_rate = 'N/A (sim)'
        return res

    def _srv_set_can_config(self, _req: SetCanConfig.Request, res: SetCanConfig.Response):
        res.success = False
        res.message = 'No CAN hardware in sim'
        return res

    # ── Actions ───────────────────────────────────────────────────────────────

    def _execute_move_to_position(self, goal_handle) -> MoveToPosition.Result:
        req    = goal_handle.request
        result = MoveToPosition.Result()

        if req.name not in self._motors:
            result.success = False
            result.message = f'Motor {req.name!r} not found'
            goal_handle.abort()
            return result

        if not self._check_joint_limits(req.name, req.target_position):
            result.success = False
            result.message = f'Target {req.target_position:.4f} violates joint limits'
            goal_handle.abort()
            return result

        self._motor_mode[req.name]    = RunMode.POSITION_CSP
        self._motor_enabled[req.name] = True
        self._desired_pos[req.name]   = req.target_position
        self._send_trajectory()

        start    = time.monotonic()
        feedback = MoveToPosition.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success        = False
                result.message        = 'Cancelled'
                result.final_position = self._state[req.name].position
                result.elapsed_time   = time.monotonic() - start
                return result

            current = self._state[req.name].position
            elapsed = time.monotonic() - start
            error   = abs(req.target_position - current)

            feedback.current_position = current
            feedback.position_error   = error
            feedback.elapsed_time     = elapsed
            goal_handle.publish_feedback(feedback)

            if error <= (req.tolerance if req.tolerance > 0.0 else 0.01):
                goal_handle.succeed()
                result.success        = True
                result.message        = 'Target reached (sim)'
                result.final_position = current
                result.elapsed_time   = elapsed
                return result

            if req.timeout > 0.0 and elapsed >= req.timeout:
                goal_handle.abort()
                result.success        = False
                result.message        = f'Timeout after {elapsed:.2f}s, error={error:.4f} rad'
                result.final_position = current
                result.elapsed_time   = elapsed
                return result

            time.sleep(0.02)

        goal_handle.abort()
        result.success = False
        result.message = 'Node shutting down'
        return result

    def _execute_set_velocity(self, goal_handle) -> SetVelocityAction.Result:
        req    = goal_handle.request
        result = SetVelocityAction.Result()

        if req.name not in self._motors:
            result.success = False
            result.message = f'Motor {req.name!r} not found'
            goal_handle.abort()
            return result

        self._motor_mode[req.name]    = RunMode.VELOCITY
        self._motor_enabled[req.name] = True
        self._desired_vel[req.name]   = self._clamp_vel(req.name, req.target_velocity)

        start     = time.monotonic()
        vel_sum   = 0.0
        vel_count = 0
        feedback  = SetVelocityAction.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._desired_vel[req.name] = 0.0
                goal_handle.canceled()
                result.success          = False
                result.message          = 'Cancelled'
                result.average_velocity = vel_sum / max(vel_count, 1)
                result.elapsed_time     = time.monotonic() - start
                return result

            s       = self._state[req.name]
            elapsed = time.monotonic() - start
            vel_sum   += s.velocity
            vel_count += 1

            feedback.current_velocity = s.velocity
            feedback.current_torque   = s.torque
            feedback.elapsed_time     = elapsed
            goal_handle.publish_feedback(feedback)

            if req.duration > 0.0 and elapsed >= req.duration:
                self._desired_vel[req.name] = 0.0
                goal_handle.succeed()
                result.success          = True
                result.message          = f'Completed after {elapsed:.2f}s'
                result.average_velocity = vel_sum / max(vel_count, 1)
                result.elapsed_time     = elapsed
                return result

            time.sleep(0.02)

        goal_handle.abort()
        result.success = False
        result.message = 'Node shutting down'
        return result


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    _tmp = rclpy.create_node(f'_sim_cfg_reader')
    _tmp.declare_parameter('config_path', '')
    cfg_path = _tmp.get_parameter('config_path').value
    _tmp.destroy_node()

    node_name = 'sim_motor_node'
    if cfg_path:
        try:
            with open(cfg_path, 'rb') as f:
                _cfg = tomllib.load(f)
            node_name = _cfg.get('defaults', {}).get('node_name', node_name)
        except Exception:
            pass

    node     = SimMotorNode(node_name=node_name)
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
