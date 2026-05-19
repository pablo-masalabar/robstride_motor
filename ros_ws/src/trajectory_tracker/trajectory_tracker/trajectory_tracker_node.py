import csv
import os
import re
import signal
import threading
import time
import tomllib
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.action import Homing, RecordTrajectory, ReplayTrajectory, SimulateTrajectory
from custom_interfaces.msg import JointCommand, MotorState, PositionCSPCommand, PositionPPCommand
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
from trajectory_tracker import transforms as _transforms
from custom_interfaces.srv import CaptureHomingPose, EnableMotor, RecordArmPose, SetActiveReport, SetArmPose, SetRunMode, StopTrajectoryRecording, TrimTrajectory

_VALID_MODES = frozenset({'pp', 'csp'})
_VALID_ARMS  = frozenset({'left_arm', 'right_arm'})

_MODE_INT = {
    'pp':  1,   # POSITION_PP
    'csp': 5,   # POSITION_CSP
}


class TrajectoryTrackerNode(Node):

    def __init__(self):
        super().__init__('trajectory_tracker_node')

        self._cb_subs  = MutuallyExclusiveCallbackGroup()
        self._cb_srvs  = ReentrantCallbackGroup()
        self._cb_setup = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('config_path', '')
        self.declare_parameter('config_dir', '')

        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        config_dir = self.get_parameter('config_dir').value
        self._config_dir: str = config_dir if config_dir else str(Path(config_path).parent)

        cfg = self._load_config(config_path)

        self._left_arm_motors:  List[str] = list(cfg['left_arm_motors'])
        self._right_arm_motors: List[str] = list(cfg['right_arm_motors'])
        self._left_arm_prefix:  str       = cfg.get('left_arm_node_prefix', '')
        self._right_arm_prefix: str       = cfg.get('right_arm_node_prefix', '')

        _node_name: str = cfg.get('node_name', 'trajectory_tracker')
        self._ns: str   = f'{_node_name}/'

        self._active_report_hz:     float = float(cfg.get('active_report_hz',     50.0))
        self._trajectory_record_hz: float = float(cfg.get('trajectory_record_hz', 50.0))
        self._replay_hz:            float = float(cfg.get('replay_hz',            50.0))

        _pkg_path = Path(cfg['package_path'])
        self._export_path: str = str(_pkg_path / 'recorded_trajectories')
        self._poses_path:  str = str(_pkg_path / 'recorded_poses')

        # motor_map: explicit recording→replay mapping; falls back to parallel lists
        if 'motor_map' in cfg:
            self._motor_map: Dict[str, str] = dict(cfg['motor_map'])
        else:
            self._motor_map = {
                s: t for s, t in zip(self._left_arm_motors, self._right_arm_motors)
            }

        # Derive recording/replay arms from which arm's motors appear in motor_map keys
        map_keys = set(self._motor_map.keys())
        if map_keys.issubset(set(self._left_arm_motors)):
            self._recording_arm     = 'left_arm'
            self._recording_motors  = self._left_arm_motors
            self._recording_prefix  = self._left_arm_prefix
            self._replay_arm        = 'right_arm'
            self._replay_motors     = self._right_arm_motors
            self._replay_prefix     = self._right_arm_prefix
        else:
            self._recording_arm     = 'right_arm'
            self._recording_motors  = self._right_arm_motors
            self._recording_prefix  = self._right_arm_prefix
            self._replay_arm        = 'left_arm'
            self._replay_motors     = self._left_arm_motors
            self._replay_prefix     = self._left_arm_prefix

        self._transforms:         Dict[str, callable] = self._load_transforms(cfg, 'transform_map')
        self._inverse_transforms: Dict[str, callable] = self._load_transforms(cfg, 'inverse_transform_map')

        mode = cfg.get('replay_motor_mode', 'pp')
        if mode not in _VALID_MODES:
            raise RuntimeError(f'Invalid replay_motor_mode "{mode}". Valid: {_VALID_MODES}')
        self._target_mode: str = mode

        pp = cfg.get('pp_defaults', {})
        self._pp_defaults: Dict[str, float] = {
            'speed':        float(pp.get('speed',        5.0)),
            'acceleration': float(pp.get('acceleration', 10.0)),
            'deceleration': float(pp.get('deceleration', 10.0)),
            'torque_limit': float(pp.get('torque_limit', 0.0)),
        }

        csp = cfg.get('csp_defaults', {})
        self._csp_defaults: Dict[str, float] = {
            'speed_limit':   float(csp.get('speed_limit',   10.0)),
            'current_limit': float(csp.get('current_limit', 0.0)),
        }

        self._latest_states: Dict[str, MotorState] = {}

        self._is_recording:           bool                     = False
        self._recording_stop_event:   Optional[threading.Event] = None
        self._last_recording_file:    str                      = ''
        self._last_recording_samples: int                      = 0

        self._step_event:            Optional[threading.Event] = None
        self._step_cancel_requested: bool                      = False

        self._replay_active:      bool             = False
        self._replay_pause_event: threading.Event  = threading.Event()
        self._replay_pause_event.set()   # unpaused by default

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Active report clients — one per physical arm
        self._left_arm_report_client = self.create_client(
            SetActiveReport,
            f'{self._left_arm_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._right_arm_report_client = self.create_client(
            SetActiveReport,
            f'{self._right_arm_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )

        # State subscriptions for all motors on both arms so fault monitoring
        # works regardless of which arm is currently replaying.
        self._state_subs: Dict[str, object] = {}
        for name in self._left_arm_motors:
            topic = f'{self._left_arm_prefix}/motors/{name}/state'
            self._state_subs[name] = self.create_subscription(
                MotorState, topic,
                lambda msg, n=name: self._on_motor_state(n, msg),
                qos, callback_group=self._cb_subs,
            )
        for name in self._right_arm_motors:
            topic = f'{self._right_arm_prefix}/motors/{name}/state'
            self._state_subs[name] = self.create_subscription(
                MotorState, topic,
                lambda msg, n=name: self._on_motor_state(n, msg),
                qos, callback_group=self._cb_subs,
            )

        # Command publishers — created for ALL motors on both arms so any
        # recording→replay direction can be served without dynamic publisher creation.
        self._pp_pubs:  Dict[str, object] = {}
        self._csp_pubs: Dict[str, object] = {}
        for name in self._left_arm_motors:
            self._pp_pubs[name]  = self.create_publisher(
                PositionPPCommand,
                f'{self._left_arm_prefix}/motors/{name}/cmd_position_pp',
                qos,
            )
            self._csp_pubs[name] = self.create_publisher(
                PositionCSPCommand,
                f'{self._left_arm_prefix}/motors/{name}/cmd_position_csp',
                qos,
            )
        for name in self._right_arm_motors:
            self._pp_pubs[name]  = self.create_publisher(
                PositionPPCommand,
                f'{self._right_arm_prefix}/motors/{name}/cmd_position_pp',
                qos,
            )
            self._csp_pubs[name] = self.create_publisher(
                PositionCSPCommand,
                f'{self._right_arm_prefix}/motors/{name}/cmd_position_csp',
                qos,
            )

        self._run_mode_clients:     Dict[str, object] = {}
        self._enable_motor_clients: Dict[str, object] = {}

        self.create_service(
            Trigger,
            f'{self._ns}pause_resume_replay',
            self._srv_pause_resume_replay,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            CaptureHomingPose,
            f'{self._ns}capture_homing_pose',
            self._capture_homing_pose,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            RecordArmPose,
            f'{self._ns}record_arm_pose',
            self._record_arm_pose,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            SetArmPose,
            f'{self._ns}set_arm_pose',
            self._set_arm_pose,
            callback_group=self._cb_srvs,
        )

        self._homing_action_server = ActionServer(
            self,
            Homing,
            f'{self._ns}homing',
            self._execute_homing,
            callback_group=self._cb_srvs,
        )
        self._record_action_server = ActionServer(
            self,
            RecordTrajectory,
            f'{self._ns}record_trajectory',
            self._execute_record_trajectory,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            StopTrajectoryRecording,
            f'{self._ns}stop_trajectory_recording',
            self._stop_recording,
            callback_group=self._cb_srvs,
        )
        self.create_subscription(
            Bool,
            f'{self._ns}step_trajectory',
            self._on_step_trigger,
            10,
            callback_group=self._cb_subs,
        )
        self.create_service(
            TrimTrajectory,
            f'{self._ns}trim_trajectory',
            self._trim_trajectory,
            callback_group=self._cb_srvs,
        )
        self._replay_action_server = ActionServer(
            self,
            ReplayTrajectory,
            f'{self._ns}replay_trajectory',
            self._execute_replay_trajectory,
            callback_group=self._cb_srvs,
        )
        self._simulate_action_server = ActionServer(
            self,
            SimulateTrajectory,
            f'{self._ns}simulate_trajectory',
            self._execute_simulate_trajectory,
            callback_group=self._cb_srvs,
        )
        self._joint_cmd_pub = self.create_publisher(
            JointCommand,
            f'{self._ns}joint_command',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        self._setup_timer = self.create_timer(
            1.0, self._setup_once, callback_group=self._cb_setup
        )

        self.get_logger().info(
            f'TrajectoryTrackerNode init — '
            f'ns={self._ns}  '
            f'recording={self._recording_prefix} ({len(self._recording_motors)} motors)  '
            f'replay={self._replay_prefix} ({len(self._replay_motors)} motors)  '
            f'mode={self._target_mode}'
        )

    # ── Config ────────────────────────────────────────────────────────────────

    def _load_transforms(self, cfg: dict, key: str) -> Dict[str, callable]:
        """Load transform map keyed by base motor name (no L/R suffix)."""
        result: Dict[str, callable] = {}
        for base_name, fn_name in cfg.get(key, {}).items():
            fn = getattr(_transforms, fn_name, None)
            if fn is None:
                self.get_logger().warning(
                    f'[{base_name}] {key} "{fn_name}" not found in transforms.py — using passthrough'
                )
                result[base_name] = _transforms.passthrough
            else:
                result[base_name] = fn
        return result

    def _get_transform(self, motor_name: str, transform_map: Dict[str, callable]) -> callable:
        """Look up transform by base name (strip trailing L/R suffix)."""
        return transform_map.get(motor_name[:-1], _transforms.passthrough)

    def _prefix_for_motors(self, motor_names: List[str]) -> Optional[str]:
        name_set = set(motor_names)
        if name_set.issubset(set(self._left_arm_motors)):
            return self._left_arm_prefix
        if name_set.issubset(set(self._right_arm_motors)):
            return self._right_arm_prefix
        return None

    def _motors_for_arm(self, arm: str) -> List[str]:
        return self._left_arm_motors if arm == 'left_arm' else self._right_arm_motors

    def _prefix_for_arm(self, arm: str) -> str:
        return self._left_arm_prefix if arm == 'left_arm' else self._right_arm_prefix

    def _resolve_config(self, name: str) -> str:
        p = Path(name)
        if p.is_absolute():
            return str(p)
        return str(Path(self._config_dir) / name)

    def _resolve_homing(self, name: str) -> str:
        p = Path(name)
        if not p.suffix:
            stem = p.stem
            if not stem.endswith('_homing'):
                stem = f'{stem}_homing'
            p = p.with_name(f'{stem}.toml')
        if p.is_absolute():
            return str(p)
        return str(Path(self._poses_path) / p)

    def _load_config(self, path: str) -> dict:
        try:
            with open(path, 'rb') as f:
                return tomllib.load(f)
        except FileNotFoundError:
            self.get_logger().fatal(f'Config not found: {path}')
            raise
        except Exception as e:
            self.get_logger().fatal(f'Failed to parse config: {e}')
            raise

    # ── Startup ───────────────────────────────────────────────────────────────

    def _setup_once(self) -> None:
        self._setup_timer.cancel()
        self._setup_timer = None
        self._set_active_report(self._left_arm_report_client,  self._left_arm_prefix,  enable=True)
        self._set_active_report(self._right_arm_report_client, self._right_arm_prefix, enable=True)
        self.get_logger().info('Setup complete')

    def _set_active_report(self, client, prefix: str, enable: bool) -> None:
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{prefix}/set_active_report not available')
            return
        req        = SetActiveReport.Request()
        req.name   = 'all'
        req.enable = enable
        req.hz     = self._active_report_hz if enable else 0.0
        res = client.call(req)
        if res.success:
            self.get_logger().info(
                f'[{prefix}] active report {"enabled" if enable else "disabled"}: {res.message}'
            )
        else:
            self.get_logger().error(
                f'[{prefix}] active report {"enabled" if enable else "disabled"}: {res.message}'
            )

    # ── Replay pause/resume service ───────────────────────────────────────────

    def _srv_pause_resume_replay(self, _req: Trigger.Request, res: Trigger.Response):
        if not self._replay_active:
            res.success = False
            res.message = 'No replay in progress'
            return res

        if self._replay_pause_event.is_set():
            self._replay_pause_event.clear()
            res.success = True
            res.message = 'Replay paused'
        else:
            self._replay_pause_event.set()
            res.success = True
            res.message = 'Replay resumed'

        self.get_logger().info(res.message)
        return res

    # ── State callback ────────────────────────────────────────────────────────

    def _on_motor_state(self, motor_name: str, msg: MotorState) -> None:
        self._latest_states[motor_name] = msg

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_pp(self, target_name: str, position: float,
                    speed: float = 0.0, acceleration: float = 0.0,
                    deceleration: float = 0.0, torque_limit: float = 0.0) -> None:
        cmd              = PositionPPCommand()
        cmd.name         = target_name
        cmd.position     = position
        cmd.speed        = speed        or self._pp_defaults['speed']
        cmd.acceleration = acceleration or self._pp_defaults['acceleration']
        cmd.deceleration = deceleration or self._pp_defaults['deceleration']
        cmd.torque_limit = torque_limit or self._pp_defaults['torque_limit']
        self._pp_pubs[target_name].publish(cmd)

    def _publish_csp(self, target_name: str, position: float,
                     speed_limit: float = 0.0, current_limit: float = 0.0) -> None:
        cmd               = PositionCSPCommand()
        cmd.name          = target_name
        cmd.position      = position
        cmd.speed_limit   = speed_limit   or self._csp_defaults['speed_limit']
        cmd.current_limit = current_limit or self._csp_defaults['current_limit']
        self._csp_pubs[target_name].publish(cmd)

    def _publish(self, target_name: str, position: float, mode: str | None = None) -> None:
        m = mode or self._target_mode
        if m == 'pp':
            self._publish_pp(target_name, position)
        elif m == 'csp':
            self._publish_csp(target_name, position)

    # ── Replay trajectory action ──────────────────────────────────────────────

    def _execute_replay_trajectory(self, goal_handle) -> ReplayTrajectory.Result:
        def _abort(msg):
            goal_handle.abort()
            r = ReplayTrajectory.Result()
            r.success, r.message, r.frames_published = False, msg, 0
            return r

        traj_name   = goal_handle.request.trajectory_name.strip()
        goal_hz     = float(goal_handle.request.replay_hz)
        goal_mode   = goal_handle.request.target_mode.strip()

        pp_speed        = goal_handle.request.pp_speed        or self._pp_defaults['speed']
        pp_accel        = goal_handle.request.pp_acceleration or self._pp_defaults['acceleration']
        pp_decel        = goal_handle.request.pp_deceleration or self._pp_defaults['deceleration']
        pp_torque       = goal_handle.request.pp_torque_limit or self._pp_defaults['torque_limit']
        csp_speed_lim   = goal_handle.request.csp_speed_limit   or self._csp_defaults['speed_limit']
        csp_current_lim = goal_handle.request.csp_current_limit or self._csp_defaults['current_limit']

        export_dir = Path(self._export_path)
        file_path  = export_dir / f'{traj_name}.csv'

        if not file_path.exists():
            return _abort(f'File not found: {file_path}')

        try:
            metadata, header, rows = self._read_csv(file_path)
        except Exception as e:
            return _abort(f'Failed to read {file_path}: {e}')

        if not rows:
            return _abort('CSV has no data rows')

        if goal_hz > 0.0:
            hz = goal_hz
        elif 'replay_hz' in metadata:
            try:
                hz = float(metadata['replay_hz'])
            except ValueError:
                hz = self._replay_hz
        else:
            hz = self._replay_hz
        interval = 1.0 / hz

        mode = goal_mode if goal_mode in _VALID_MODES else self._target_mode

        _FIELDS  = ['position', 'velocity', 'torque', 'temperature', 'mode', 'fault', 'enabled']
        n_fields = len(_FIELDS)
        csv_motors = [
            col[: -(len('_position'))]
            for col in header[1:]
            if col.endswith('_position')
        ]

        req_left  = goal_handle.request.replay_left_arm
        req_right = goal_handle.request.replay_right_arm
        if not req_left and not req_right:
            return _abort('At least one of replay_left_arm or replay_right_arm must be true')

        left_set  = set(self._left_arm_motors)
        right_set = set(self._right_arm_motors)
        csv_set   = set(csv_motors)

        both_recorded = bool(csv_set & left_set) and bool(csv_set & right_set)

        # replay_entries: (col_base, rec_motor, replay_motor, transform_fn)
        # Publishers already exist for all motors on both arms.
        replay_entries: List[tuple] = []

        for i, rec_motor in enumerate(csv_motors):
            col_base = 1 + i * n_fields

            if rec_motor in left_set:
                rec_arm, opp_suffix = 'left_arm', 'R'
            elif rec_motor in right_set:
                rec_arm, opp_suffix = 'right_arm', 'L'
            else:
                continue

            base = rec_motor[:-1]

            if both_recorded:
                # CSV has both arms — only same-arm passthrough, discard the other
                if rec_arm == 'left_arm' and req_left:
                    replay_entries.append((col_base, rec_motor, rec_motor, _transforms.passthrough))
                elif rec_arm == 'right_arm' and req_right:
                    replay_entries.append((col_base, rec_motor, rec_motor, _transforms.passthrough))
            else:
                # Single arm recorded — support same-arm and cross-arm
                if req_left:
                    if rec_arm == 'left_arm':
                        replay_entries.append((col_base, rec_motor, rec_motor, _transforms.passthrough))
                    else:
                        replay_entries.append((
                            col_base, rec_motor, base + 'L',
                            self._get_transform(rec_motor, self._inverse_transforms),
                        ))
                if req_right:
                    if rec_arm == 'right_arm':
                        replay_entries.append((col_base, rec_motor, rec_motor, _transforms.passthrough))
                    else:
                        replay_entries.append((
                            col_base, rec_motor, base + 'R',
                            self._get_transform(rec_motor, self._transforms),
                        ))

        if not replay_entries:
            return _abort('No motors to replay — check arm filter flags and CSV contents')

        self.get_logger().info(
            f'ReplayTrajectory — {traj_name}  hz={hz}  mode={mode}  '
            f'replay_motors={list({e[2] for e in replay_entries})}'
        )

        # Set run mode and enable motors, grouped by prefix
        prefix_motors: Dict[str, List[str]] = {}
        for _, _, replay_motor, _ in replay_entries:
            prefix = self._left_arm_prefix if replay_motor in left_set else self._right_arm_prefix
            prefix_motors.setdefault(prefix, []).append(replay_motor)

        enable_clients: Dict[str, object] = {}
        for prefix, motors in prefix_motors.items():
            run_client = self._get_run_mode_client(prefix)
            if not run_client.wait_for_service(timeout_sec=3.0):
                return _abort(f'{prefix}/set_run_mode not available')
            for motor in motors:
                req = SetRunMode.Request()
                req.name = motor
                req.mode = _MODE_INT[mode]
                req.automatic_enable_disable = True
                run_client.call(req)

            en_client = self._get_enable_motor_client(prefix)
            if not en_client.wait_for_service(timeout_sec=3.0):
                return _abort(f'{prefix}/enable_motor not available')
            en_req             = EnableMotor.Request()
            en_req.name        = 'all'
            en_req.enable      = True
            en_req.clear_fault = False
            en_client.call(en_req)
            enable_clients[prefix] = en_client

        total        = len(rows)
        published    = 0
        start_mono   = time.monotonic()
        last_row     = None
        step_through  = bool(goal_handle.request.step_through)
        step_pct      = float(goal_handle.request.step_pct)
        step_frames   = max(1, int(total * step_pct / 100.0)) if step_through else 0
        replay_motors = {e[2] for e in replay_entries}

        self._replay_active = True
        self._replay_pause_event.set()   # ensure unpaused at start

        def _check_faults() -> Optional[str]:
            for motor in replay_motors:
                state = self._latest_states.get(motor)
                if state and state.fault:
                    return f'{motor} fault=0x{state.fault:08X}'
            return None

        def _wait_if_paused() -> bool:
            """Block while paused. Returns False if cancel or fault detected during wait."""
            while not self._replay_pause_event.is_set():
                if goal_handle.is_cancel_requested:
                    return False
                if _check_faults():
                    return False
                time.sleep(0.05)
            return True

        def _publish_row(row):
            nonlocal last_row, published
            loop_start = time.monotonic()
            for col_base, rec_motor, replay_motor, transform in replay_entries:
                try:
                    position = float(row[col_base])
                except (IndexError, ValueError):
                    continue
                pos = transform(position)
                if mode == 'pp':
                    self._publish_pp(replay_motor, pos, pp_speed, pp_accel, pp_decel, pp_torque)
                elif mode == 'csp':
                    self._publish_csp(replay_motor, pos, csp_speed_lim, csp_current_lim)
            last_row   = row
            published += 1
            fb                  = ReplayTrajectory.Feedback()
            fb.frames_published = published
            fb.frames_total     = total
            fb.elapsed_time     = time.monotonic() - start_mono
            fb.progress_pct     = round(published / total * 100.0, 1)
            goal_handle.publish_feedback(fb)
            sleep_for = interval - (time.monotonic() - loop_start)
            if sleep_for > 0:
                time.sleep(sleep_for)

        def _cancelled():
            goal_handle.canceled()
            r = ReplayTrajectory.Result()
            r.success, r.message, r.frames_published = False, 'Cancelled', published
            return r

        fault_desc: Optional[str] = None

        if step_through:
            self._step_event            = threading.Event()
            self._step_cancel_requested = False
            self.get_logger().info(
                f'Step-through mode — step={step_pct}% ({step_frames} frames). '
                f'Publish true to /step_trajectory to advance, false to cancel.'
            )
            frame_idx = 0
            cancelled = False
            while frame_idx < total:
                while not self._step_event.wait(timeout=0.1):
                    if goal_handle.is_cancel_requested:
                        cancelled = True
                        break
                    fault_desc = _check_faults()
                    if fault_desc:
                        break
                if cancelled or fault_desc:
                    break

                self._step_event.clear()

                if self._step_cancel_requested or goal_handle.is_cancel_requested:
                    cancelled = True
                    break

                batch_end = min(frame_idx + step_frames, total)
                for row in rows[frame_idx:batch_end]:
                    if goal_handle.is_cancel_requested or self._step_cancel_requested:
                        cancelled = True
                        break
                    fault_desc = _check_faults()
                    if fault_desc:
                        break
                    if not _wait_if_paused():
                        fault_desc = fault_desc or (_check_faults())
                        if not fault_desc:
                            cancelled = True
                        break
                    _publish_row(row)
                if cancelled or fault_desc:
                    break
                frame_idx = batch_end

            self._step_event            = None
            self._step_cancel_requested = False
            if fault_desc:
                pass   # handled below
            elif cancelled:
                self._replay_active = False
                return _cancelled()
        else:
            for row in rows:
                if goal_handle.is_cancel_requested:
                    self._replay_active = False
                    return _cancelled()
                fault_desc = _check_faults()
                if fault_desc:
                    break
                if not _wait_if_paused():
                    fault_desc = _check_faults()
                    if not fault_desc and goal_handle.is_cancel_requested:
                        self._replay_active = False
                        return _cancelled()
                    break
                _publish_row(row)

        self._replay_active = False
        self._replay_pause_event.set()   # ensure unpaused for next replay

        if fault_desc:
            msg = f'Motor fault detected after {published} frames — {fault_desc}. Motors left enabled at last pose.'
            self.get_logger().error(msg)
            goal_handle.canceled()
            result                  = ReplayTrajectory.Result()
            result.success          = False
            result.message          = msg
            result.frames_published = published
            return result

        if last_row is not None:
            for col_base, rec_motor, replay_motor, transform in replay_entries:
                try:
                    position = float(last_row[col_base])
                except (IndexError, ValueError):
                    continue
                pos = transform(position)
                if mode == 'pp':
                    self._publish_pp(replay_motor, pos, pp_speed, pp_accel, pp_decel, pp_torque)
                else:
                    self._publish_csp(replay_motor, pos, csp_speed_lim, csp_current_lim)
            time.sleep(1.0)

        goal_handle.succeed()
        result                  = ReplayTrajectory.Result()
        result.success          = True
        result.message          = f'Replayed {published}/{total} frames in {mode} mode at {hz} Hz. Motors left enabled at last pose.'
        result.frames_published = published
        self.get_logger().info(result.message)
        return result

    def _on_step_trigger(self, msg: Bool) -> None:
        if self._step_event is None:
            return
        if msg.data:
            self._step_event.set()
        else:
            self._step_cancel_requested = True
            self._step_event.set()

    # ── Record arm pose service ───────────────────────────────────────────────

    def _record_arm_pose(self, request, response):
        arm  = request.arm.strip().lower()
        name = request.name.strip()

        if arm not in _VALID_ARMS:
            response.success   = False
            response.message   = f'arm must be "left_arm" or "right_arm", got "{arm}"'
            response.file_path = ''
            return response

        motor_names   = self._motors_for_arm(arm)
        prefix        = self._prefix_for_arm(arm)
        state_pattern = f'{prefix}/motors/{{name}}/state'

        if not name:
            name = datetime.now().strftime('%H_%M_%S_%d_%m_%y')

        export_dir = Path(self._poses_path)
        export_dir.mkdir(parents=True, exist_ok=True)
        file_path = export_dir / f'{name}.csv'

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        captured: Dict[str, MotorState] = {}
        events:   Dict[str, threading.Event] = {}
        subs = []

        for motor in motor_names:
            ev = threading.Event()
            events[motor] = ev
            topic = state_pattern.format(name=motor)
            sub = self.create_subscription(
                MotorState,
                topic,
                lambda msg, n=motor, e=ev: (captured.update({n: msg}), e.set()),
                qos,
                callback_group=self._cb_subs,
            )
            subs.append(sub)

        timeout = 3.0
        for motor, ev in events.items():
            if not ev.wait(timeout=timeout):
                self.get_logger().warning(f'[{motor}] no state received within {timeout}s — skipping')

        for sub in subs:
            self.destroy_subscription(sub)

        if not captured:
            response.success   = False
            response.message   = 'No motor states received'
            response.file_path = ''
            return response

        _FIELDS = ['position', 'velocity', 'torque', 'temperature', 'mode', 'fault', 'enabled']
        with open(file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['motor_name'] + _FIELDS)
            for motor in motor_names:
                state = captured.get(motor)
                if state:
                    writer.writerow([
                        motor,
                        state.position,
                        state.velocity,
                        state.torque,
                        state.temperature,
                        state.mode,
                        state.fault,
                        int(state.enabled),
                    ])

        response.success   = True
        response.message   = f'Recorded {len(captured)}/{len(motor_names)} motors to {file_path}'
        response.file_path = str(file_path)
        self.get_logger().info(response.message)
        return response

    # ── Set arm pose service ──────────────────────────────────────────────────

    def _set_arm_pose(self, request, response):
        name = request.name.strip()
        mode = request.target_mode.strip()
        arm  = request.arm.strip().lower() if request.arm.strip() else self._replay_arm

        if arm not in _VALID_ARMS:
            response.success, response.message, response.motors_set = \
                False, f'arm must be "left_arm" or "right_arm", got "{arm}"', []
            return response

        if not name:
            response.success, response.message, response.motors_set = False, 'name is required', []
            return response

        if mode not in _VALID_MODES:
            response.success = False
            response.message = f'target_mode must be "pp" or "csp", got "{mode}"'
            response.motors_set = []
            return response

        file_path = Path(self._poses_path) / f'{name}.csv'
        if not file_path.exists():
            response.success, response.message, response.motors_set = \
                False, f'File not found: {file_path}', []
            return response

        motor_positions: Dict[str, float] = {}
        with open(file_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    motor_positions[row['motor_name']] = float(row['position'])
                except (KeyError, ValueError):
                    continue

        if not motor_positions:
            response.success, response.message, response.motors_set = \
                False, 'No valid motor positions found in CSV', []
            return response

        reverse_motor_map: Dict[str, str] = {v: k for k, v in self._motor_map.items()}

        is_recording_arm = (arm == self._recording_arm)
        is_replay_arm    = (arm == self._replay_arm)

        pp_speed    = request.pp_speed        or self._pp_defaults['speed']
        pp_accel    = request.pp_acceleration or self._pp_defaults['acceleration']
        pp_decel    = request.pp_deceleration or self._pp_defaults['deceleration']
        pp_torque   = request.pp_torque_limit or self._pp_defaults['torque_limit']
        csp_speed   = request.csp_speed_limit   or self._csp_defaults['speed_limit']
        csp_current = request.csp_current_limit or self._csp_defaults['current_limit']

        mode_int = _MODE_INT[mode]
        qos      = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        cmd_entries: List[tuple] = []
        for csv_motor, position in motor_positions.items():
            if csv_motor in self._motor_map:
                # CSV has a recording-arm motor
                if is_replay_arm:
                    cmd_motor = self._motor_map[csv_motor]
                    prefix    = self._replay_prefix
                    transform = self._get_transform(csv_motor, self._transforms)
                else:
                    cmd_motor = csv_motor
                    prefix    = self._recording_prefix
                    transform = _transforms.passthrough
            elif csv_motor in reverse_motor_map:
                # CSV has a replay-arm motor
                rec_motor = reverse_motor_map[csv_motor]
                if is_recording_arm:
                    cmd_motor = rec_motor
                    prefix    = self._recording_prefix
                    transform = self._get_transform(rec_motor, self._inverse_transforms)
                else:
                    cmd_motor = csv_motor
                    prefix    = self._replay_prefix
                    transform = _transforms.passthrough
            else:
                self.get_logger().warning(f'[{csv_motor}] not in motor_map — skipping')
                continue
            cmd_entries.append((csv_motor, cmd_motor, prefix, transform(position)))

        # Set run mode grouped by prefix
        prefix_motors: Dict[str, List[str]] = {}
        for _, cmd_motor, prefix, _ in cmd_entries:
            prefix_motors.setdefault(prefix, []).append(cmd_motor)

        for prefix, motors in prefix_motors.items():
            run_mode_client = self._get_run_mode_client(prefix)
            if not run_mode_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'{prefix}/set_run_mode not available')
                continue
            for motor in motors:
                req                          = SetRunMode.Request()
                req.name                     = motor
                req.mode                     = mode_int
                req.automatic_enable_disable = True
                res = run_mode_client.call(req)
                if not res.success:
                    self.get_logger().error(f'[{motor}] set_run_mode failed: {res.message}')

        motors_set: List[str] = []
        for csv_motor, cmd_motor, prefix, cmd_position in cmd_entries:
            if mode == 'pp':
                pub = self.create_publisher(
                    PositionPPCommand, f'{prefix}/motors/{cmd_motor}/cmd_position_pp', qos)
                cmd              = PositionPPCommand()
                cmd.name         = cmd_motor
                cmd.position     = cmd_position
                cmd.speed        = pp_speed
                cmd.acceleration = pp_accel
                cmd.deceleration = pp_decel
                cmd.torque_limit = pp_torque
            else:
                pub = self.create_publisher(
                    PositionCSPCommand, f'{prefix}/motors/{cmd_motor}/cmd_position_csp', qos)
                cmd               = PositionCSPCommand()
                cmd.name          = cmd_motor
                cmd.position      = cmd_position
                cmd.speed_limit   = csp_speed
                cmd.current_limit = csp_current

            pub.publish(cmd)
            motors_set.append(cmd_motor)
            self.get_logger().info(
                f'[{csv_motor}] → [{cmd_motor}] pose {cmd_position:.4f} rad ({mode})'
            )

        response.success    = True
        response.message    = f'Set {len(motors_set)}/{len(motor_positions)} motors to pose "{name}" in {mode} mode'
        response.motors_set = motors_set
        self.get_logger().info(response.message)
        return response

    # ── Capture homing pose service ───────────────────────────────────────────

    def _capture_homing_pose(self, request, response):
        arm = request.arm.strip().lower() if hasattr(request.arm, 'strip') else request.arm
        if arm not in _VALID_ARMS:
            response.success   = False
            response.message   = f'arm must be "left_arm" or "right_arm", got "{arm}"'
            response.motors    = []
            response.positions = []
            return response

        prefix      = self._prefix_for_arm(arm)
        motor_names = self._motors_for_arm(arm)

        config_path = self._resolve_homing(request.config_file)
        self.get_logger().info(f'CaptureHomingPose — arm: {arm}, config: {config_path}')

        if not Path(config_path).exists():
            template = self._find_homing_template()
            if template is None:
                response.success   = False
                response.message   = f'Config not found and no homing template available: {config_path}'
                response.motors    = []
                response.positions = []
                return response
            self._create_homing_config(template, config_path, motor_names)
            self.get_logger().info(f'Created homing config from template: {config_path}')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        captured: Dict[str, float] = {}
        events:   Dict[str, threading.Event] = {}
        subs = []

        for name in motor_names:
            ev = threading.Event()
            events[name] = ev
            topic = f'{prefix}/motors/{name}/state'
            sub = self.create_subscription(
                MotorState,
                topic,
                lambda msg, n=name, e=ev: (captured.update({n: msg.position}), e.set()),
                qos,
                callback_group=self._cb_subs,
            )
            subs.append(sub)

        timeout = 3.0
        for name, ev in events.items():
            if not ev.wait(timeout=timeout):
                self.get_logger().warning(f'[{name}] no state received within {timeout}s — skipping')

        for sub in subs:
            self.destroy_subscription(sub)

        if not captured:
            response.success   = False
            response.message   = 'No motor states received'
            response.motors    = []
            response.positions = []
            return response

        self._update_homing_pos_in_file(config_path, captured)

        response.success   = True
        response.message   = f'Captured {len(captured)}/{len(motor_names)} motors'
        response.motors    = list(captured.keys())
        response.positions = list(captured.values())
        self.get_logger().info(response.message)
        return response

    def _find_homing_template(self) -> Optional[str]:
        import glob as _glob
        matches = _glob.glob(os.path.join(self._poses_path, '*_homing.toml'))
        return matches[0] if matches else None

    def _create_homing_config(self, template_path: str, dest_path: str,
                               motor_names: List[str]) -> None:
        with open(template_path, 'rb') as f:
            tmpl = tomllib.load(f)

        pp = tmpl.get('pp_defaults', {
            'speed': 5.0, 'acceleration': 10.0, 'deceleration': 10.0, 'torque_limit': 0.0
        })

        lines = [
            '# homing configuration\n',
            '\n',
            '[homing_pos]\n',
        ]
        for name in motor_names:
            lines.append(f'{name} = 0.0\n')
        lines += [
            '\n',
            '[pp_defaults]\n',
            f'speed        = {pp.get("speed",        5.0)}\n',
            f'acceleration = {pp.get("acceleration", 10.0)}\n',
            f'deceleration = {pp.get("deceleration", 10.0)}\n',
            f'torque_limit = {pp.get("torque_limit",  0.0)}\n',
        ]
        Path(self._poses_path).mkdir(parents=True, exist_ok=True)
        Path(dest_path).parent.mkdir(parents=True, exist_ok=True)
        with open(dest_path, 'w') as f:
            f.writelines(lines)

    def _update_homing_pos_in_file(self, path: str, new_positions: Dict[str, float]) -> None:
        with open(path, 'r') as f:
            content = f.read()
        for motor, pos in new_positions.items():
            content = re.sub(
                rf'^({re.escape(motor)}\s*=\s*)[\d.+\-eE]+',
                rf'\g<1>{pos:.6f}',
                content,
                flags=re.MULTILINE,
            )
        with open(path, 'w') as f:
            f.write(content)
        self.get_logger().info(f'Updated {path} with {len(new_positions)} positions')

    # ── Record trajectory action & stop service ───────────────────────────────

    def _execute_record_trajectory(self, goal_handle) -> RecordTrajectory.Result:
        def _abort(msg):
            goal_handle.abort()
            r = RecordTrajectory.Result()
            r.success, r.message, r.file_path, r.samples_recorded = False, msg, '', 0
            return r

        if self._is_recording:
            return _abort('Already recording')

        use_left  = bool(goal_handle.request.left_arm_source)
        use_right = bool(goal_handle.request.right_arm_source)

        if not use_left and not use_right:
            return _abort('At least one of left_arm_source or right_arm_source must be true')

        # Build the ordered list of motors to record (left first, then right)
        motors_to_record: List[str] = []
        if use_left:
            motors_to_record += self._left_arm_motors
        if use_right:
            motors_to_record += self._right_arm_motors

        # Subscribe to any arms not already covered by the existing recording subs
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        extra_subs = []
        existing_subs = set(self._state_subs.keys())
        for motor in motors_to_record:
            if motor not in existing_subs:
                prefix = self._left_arm_prefix if motor in self._left_arm_motors else self._right_arm_prefix
                topic  = f'{prefix}/motors/{motor}/state'
                sub = self.create_subscription(
                    MotorState,
                    topic,
                    lambda msg, n=motor: self._on_motor_state(n, msg),
                    qos,
                    callback_group=self._cb_subs,
                )
                extra_subs.append(sub)

        self._is_recording         = True
        self._recording_stop_event = threading.Event()

        traj_name = goal_handle.request.trajectory_name.strip()
        if not traj_name:
            traj_name = datetime.now().strftime('%H_%M_%S_%d_%m_%y')

        export_dir = Path(self._export_path)
        export_dir.mkdir(parents=True, exist_ok=True)
        file_path = export_dir / f'{traj_name}.csv'
        self._last_recording_file    = str(file_path)
        self._last_recording_samples = 0

        _FIELDS = ['position', 'velocity', 'torque', 'temperature', 'mode', 'fault', 'enabled']
        header  = ['timestamp'] + [f'{m}_{f}' for m in motors_to_record for f in _FIELDS]

        start_time = time.monotonic()
        interval   = 1.0 / self._trajectory_record_hz
        samples    = 0

        recorded_at = datetime.now().strftime('%H_%M_%S_%d_%m_%y')
        arms_label  = '+'.join(
            ([' left_arm'] if use_left else []) + (['right_arm'] if use_right else [])
        )
        self.get_logger().info(f'Recording started ({arms_label}) → {file_path}')

        with open(file_path, 'w', newline='') as csvfile:
            csvfile.write(f'# recorded_at: {recorded_at}\n')
            csvfile.write(f'# replay_hz: {self._trajectory_record_hz}\n')
            writer = csv.writer(csvfile)
            writer.writerow(header)

            while not self._recording_stop_event.is_set() and not goal_handle.is_cancel_requested:
                loop_start = time.monotonic()
                ts = round(loop_start - start_time, 4)

                row: List = [ts]
                for motor in motors_to_record:
                    state = self._latest_states.get(motor)
                    if state:
                        row += [
                            state.position,
                            state.velocity,
                            state.torque,
                            state.temperature,
                            state.mode,
                            state.fault,
                            int(state.enabled),
                        ]
                    else:
                        row += [''] * len(_FIELDS)

                writer.writerow(row)
                samples += 1
                self._last_recording_samples = samples

                feedback                  = RecordTrajectory.Feedback()
                feedback.samples_recorded = samples
                feedback.elapsed_time     = ts
                goal_handle.publish_feedback(feedback)

                sleep_for = interval - (time.monotonic() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)

        for sub in extra_subs:
            self.destroy_subscription(sub)

        self._last_recording_file    = str(file_path)
        self._last_recording_samples = samples
        self._is_recording           = False

        self.get_logger().info(f'Recording stopped — {samples} samples saved to {file_path}')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result                  = RecordTrajectory.Result()
            result.success          = False
            result.message          = f'Cancelled — {samples} samples saved to {file_path}'
            result.file_path        = str(file_path)
            result.samples_recorded = samples
            return result

        goal_handle.succeed()
        result                  = RecordTrajectory.Result()
        result.success          = True
        result.message          = f'{samples} samples saved to {file_path}'
        result.file_path        = str(file_path)
        result.samples_recorded = samples
        return result

    def _stop_recording(self, request, response):
        if not self._is_recording or self._recording_stop_event is None:
            response.success          = False
            response.message          = 'No active recording'
            response.file_path        = self._last_recording_file
            response.samples_recorded = self._last_recording_samples
            return response

        self._recording_stop_event.set()
        response.success          = True
        response.message          = 'Stop signal sent'
        response.file_path        = self._last_recording_file
        response.samples_recorded = self._last_recording_samples
        return response

    # ── Homing action ─────────────────────────────────────────────────────────

    def _read_csv(self, file_path: Path):
        metadata, header, rows = {}, None, []
        with open(file_path, 'r', newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                if row[0].startswith('#'):
                    content = row[0][1:].strip()
                    if ':' in content:
                        k, v = content.split(':', 1)
                        metadata[k.strip()] = v.strip()
                    continue
                if header is None:
                    header = row
                else:
                    rows.append(row)
        return metadata, header or [], rows

    def _get_run_mode_client(self, prefix: str):
        if prefix not in self._run_mode_clients:
            self._run_mode_clients[prefix] = self.create_client(
                SetRunMode,
                f'{prefix}/set_run_mode',
                callback_group=self._cb_srvs,
            )
        return self._run_mode_clients[prefix]

    def _get_enable_motor_client(self, prefix: str):
        if prefix not in self._enable_motor_clients:
            self._enable_motor_clients[prefix] = self.create_client(
                EnableMotor,
                f'{prefix}/enable_motor',
                callback_group=self._cb_srvs,
            )
        return self._enable_motor_clients[prefix]

    def _execute_homing(self, goal_handle) -> Homing.Result:
        config_path = self._resolve_homing(goal_handle.request.config_path)
        self.get_logger().info(f'Homing action started — config: {config_path}')

        try:
            cfg = self._load_config(config_path)
        except Exception as e:
            goal_handle.abort()
            result = Homing.Result()
            result.success = False
            result.message = f'Failed to load config: {e}'
            result.homed_motors = []
            return result

        homing_pos: Dict[str, float] = dict(cfg.get('homing_pos', {}))
        pp         = cfg.get('pp_defaults', {})
        motor_names = list(homing_pos.keys())

        prefix = self._prefix_for_motors(motor_names)
        if prefix is None:
            goal_handle.abort()
            result = Homing.Result()
            result.success = False
            result.message = f'Motors {motor_names} do not match left_arm or right_arm motor lists'
            result.homed_motors = []
            return result
        total = len(motor_names)

        run_mode_client = self._get_run_mode_client(prefix)
        if not run_mode_client.wait_for_service(timeout_sec=3.0):
            goal_handle.abort()
            result = Homing.Result()
            result.success = False
            result.message = f'{prefix}/set_run_mode not available'
            result.homed_motors = []
            return result

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        pubs = {
            name: self.create_publisher(
                PositionPPCommand, f'{prefix}/motors/{name}/cmd_position_pp', qos
            )
            for name in motor_names
        }
        homed: List[str] = []

        try:
            for i, motor_name in enumerate(motor_names):
                feedback              = Homing.Feedback()
                feedback.motor_name   = motor_name
                feedback.motors_done  = i
                feedback.motors_total = total
                goal_handle.publish_feedback(feedback)

                req                          = SetRunMode.Request()
                req.name                     = motor_name
                req.mode                     = _MODE_INT['pp']
                req.automatic_enable_disable = True
                res = run_mode_client.call(req)
                if not res.success:
                    self.get_logger().error(f'[{motor_name}] set_run_mode failed: {res.message}')

                target_pos       = float(homing_pos[motor_name])
                cmd              = PositionPPCommand()
                cmd.name         = motor_name
                cmd.position     = target_pos
                cmd.speed        = float(pp.get('speed',        5.0))
                cmd.acceleration = float(pp.get('acceleration', 10.0))
                cmd.deceleration = float(pp.get('deceleration', 10.0))
                cmd.torque_limit = float(pp.get('torque_limit', 0.0))
                pubs[motor_name].publish(cmd)

                homed.append(motor_name)
                self.get_logger().info(f'[{motor_name}] homing command sent → {target_pos}')
        finally:
            for pub in pubs.values():
                self.destroy_publisher(pub)

        goal_handle.succeed()
        result              = Homing.Result()
        result.success      = True
        result.message      = f'Homed {len(homed)}/{total} motors'
        result.homed_motors = homed
        self.get_logger().info(result.message)
        return result

    # ── Simulate trajectory action ────────────────────────────────────────────

    def _execute_simulate_trajectory(self, goal_handle) -> SimulateTrajectory.Result:
        traj_name = goal_handle.request.trajectory_name.strip()
        replay_hz = float(goal_handle.request.replay_hz)

        export_dir = Path(self._export_path)
        file_path  = export_dir / f'{traj_name}.csv'

        if not file_path.exists():
            goal_handle.abort()
            result         = SimulateTrajectory.Result()
            result.success = False
            result.message = f'File not found: {file_path}'
            result.frames_published = 0
            return result

        try:
            _, header, rows = self._read_csv(file_path)
        except Exception as e:
            goal_handle.abort()
            result         = SimulateTrajectory.Result()
            result.success = False
            result.message = f'Failed to read {file_path}: {e}'
            result.frames_published = 0
            return result

        _FIELDS = ['position', 'velocity', 'torque', 'temperature', 'mode', 'fault', 'enabled']
        n_fields = len(_FIELDS)
        motor_names = [
            col[: -(len('_position'))]
            for col in header[1:]
            if col.endswith('_position')
        ]

        total      = len(rows)
        published  = 0
        start_mono = time.monotonic()
        interval   = 1.0 / replay_hz if replay_hz > 0.0 else None

        self.get_logger().info(
            f'SimulateTrajectory — {traj_name}  frames={total}  '
            f'replay_hz={"original" if interval is None else replay_hz}'
        )

        prev_ts: Optional[float] = None

        for i, row in enumerate(rows):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result                  = SimulateTrajectory.Result()
                result.success          = False
                result.message          = 'Cancelled'
                result.frames_published = published
                return result

            loop_start = time.monotonic()

            try:
                row_ts = float(row[0])
            except (IndexError, ValueError):
                continue

            msg              = JointCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.names        = motor_names
            positions, velocities, efforts = [], [], []

            for j, motor in enumerate(motor_names):
                base = 1 + j * n_fields
                try:
                    positions.append(float(row[base + 0]))
                    velocities.append(float(row[base + 1]))
                    efforts.append(float(row[base + 2]))
                except (IndexError, ValueError):
                    positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)

            msg.positions  = positions
            msg.velocities = velocities
            msg.efforts    = efforts
            self._joint_cmd_pub.publish(msg)
            published += 1

            feedback                  = SimulateTrajectory.Feedback()
            feedback.frames_published = published
            feedback.frames_total     = total
            feedback.elapsed_time     = time.monotonic() - start_mono
            goal_handle.publish_feedback(feedback)

            if interval is not None:
                sleep_for = interval - (time.monotonic() - loop_start)
            else:
                if prev_ts is not None:
                    sleep_for = (row_ts - prev_ts) - (time.monotonic() - loop_start)
                else:
                    sleep_for = 0.0

            if sleep_for > 0:
                time.sleep(sleep_for)

            prev_ts = row_ts

        goal_handle.succeed()
        result                  = SimulateTrajectory.Result()
        result.success          = True
        result.message          = f'Published {published}/{total} frames from {file_path}'
        result.frames_published = published
        self.get_logger().info(result.message)
        return result

    # ── Trim trajectory service ───────────────────────────────────────────────

    def _trim_trajectory(self, request, response):
        name = request.trajectory_name.strip()
        if not name:
            response.success = False
            response.message = 'trajectory_name is required'
            response.rows_before = response.rows_after = response.rows_removed = 0
            return response

        start_ts = list(request.start_ts)
        end_ts   = list(request.end_ts)

        if len(start_ts) != len(end_ts):
            response.success = False
            response.message = 'start_ts and end_ts must have the same length'
            response.rows_before = response.rows_after = response.rows_removed = 0
            return response

        ranges    = list(zip(start_ts, end_ts))
        file_path = Path(self._export_path) / f'{name}.csv'

        if not file_path.exists():
            response.success = False
            response.message = f'File not found: {file_path}'
            response.rows_before = response.rows_after = response.rows_removed = 0
            return response

        try:
            with open(file_path, 'r', newline='') as f:
                reader   = csv.reader(f)
                header   = next(reader)
                all_rows = list(reader)

            rows_before = len(all_rows)

            def _should_keep(row: list) -> bool:
                try:
                    ts = float(row[0])
                except (IndexError, ValueError):
                    return True
                for s, e in ranges:
                    if s < ts < e:
                        return False
                return True

            kept = [row for row in all_rows if _should_keep(row)]

            with open(file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(kept)

            rows_after   = len(kept)
            rows_removed = rows_before - rows_after

            response.success      = True
            response.message      = f'Removed {rows_removed} rows from {file_path}'
            response.rows_before  = rows_before
            response.rows_after   = rows_after
            response.rows_removed = rows_removed
            self.get_logger().info(response.message)

        except Exception as e:
            response.success      = False
            response.message      = f'Error trimming {file_path}: {e}'
            response.rows_before  = 0
            response.rows_after   = 0
            response.rows_removed = 0

        return response

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown_cleanup(self) -> None:
        print('[trajectory_tracker] Shutting down …')

        if self._is_recording and self._recording_stop_event is not None:
            print('[trajectory_tracker] Stopping active recording …')
            self._recording_stop_event.set()
            time.sleep(0.3)

        dis_req             = EnableMotor.Request()
        dis_req.name        = 'all'
        dis_req.enable      = False
        dis_req.clear_fault = False
        for prefix in (self._left_arm_prefix, self._right_arm_prefix):
            enable_client = self._get_enable_motor_client(prefix)
            if enable_client.service_is_ready():
                print(f'[trajectory_tracker] Disabling {prefix} motors …')
                done   = threading.Event()
                future = enable_client.call_async(dis_req)
                future.add_done_callback(lambda _: done.set())
                done.wait(timeout=2.0)

        for client, prefix in [
            (self._left_arm_report_client,  self._left_arm_prefix),
            (self._right_arm_report_client, self._right_arm_prefix),
        ]:
            if client.service_is_ready():
                self._set_active_report(client, prefix, enable=False)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node     = TrajectoryTrackerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    _stop = threading.Event()
    signal.signal(signal.SIGINT, lambda sig, frame: _stop.set())

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    _stop.wait()

    node.shutdown_cleanup()
    executor.shutdown()
    spin_thread.join(timeout=3.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
