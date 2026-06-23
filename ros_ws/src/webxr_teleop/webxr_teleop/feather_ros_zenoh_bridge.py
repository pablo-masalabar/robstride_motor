"""
webxr_teleop_cli.py — identical to webxr_teleop_node.py but loads config from a path
given on the command line instead of a ROS parameter.

Usage:
    ros2 run webxr_teleop webxr_teleop_cli -- --config /path/to/config.toml
"""

import argparse
import os
import signal
import threading
import tomllib
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import pinocchio as pin
import zenoh
from feather import feather_pb2
from feather.hw_joints_to_sim_joint_mapping import HW_TO_SIM, SIM_TO_HW

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.msg import MotorState, OperationCommand, PositionCSPCommand, PositionPPCommand, VelocityCommand
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from custom_interfaces.srv import EnableMotor, EnableMotors, Freeze, SetActiveReport, SetRunMode


_ROBSTRIDE_MODE_INT: Dict[str, int] = {
    'operation': 0,
    'pp':        1,
    'velocity':  2,
    'csp':       5,
}

_DAMIAO_MODE_INT: Dict[str, int] = {
    'mit':                   0,
    'position_velocity':     2,
    'velocity':              3,
    'force_position_hybrid': 4,
}

_EZMOTION_MODE_INT: Dict[str, int] = {
    'pp':  1,
    'pv':  3,
    'pt':  4,
    'hm':  6,
    'csp': 8,
    'csv': 9,
    'cst': 10,
}

# Matches GRIPPER_LIMIT_M in feather/arm.py — URDF prismatic upper limit (m).
# arm.py sends gripper position in metres [0, this]; normalise to [0, 1]
# before mapping to motor native units via _zenoh_to_gripper / _gripper_to_zenoh.
_GRIPPER_URDF_LIMIT_M = 0.0475

# Neck pitch (NpC) zenoh/URDF range — fixed by the robot URDF.
_NECK_PITCH_ZENOH_LOW  = -0.52   # rad
_NECK_PITCH_ZENOH_HIGH =  1.57   # rad


_CMD_TOPIC_MSG_TYPE = {
    'cmd_operation':    OperationCommand,
    'cmd_position_pp':  PositionPPCommand,
    'cmd_position_csp': PositionCSPCommand,
    'cmd_position_pv':  PositionPPCommand,
    'cmd_velocity':     VelocityCommand,
    'go_to':            Float64,
}

_DEFAULT_CONFIG = Path(__file__).parent / 'config.toml'


class WebXRTeleopNode(Node):

    def __init__(self, config_path: Path):
        super().__init__('webxr_teleop_node')

        self._cb_subs  = MutuallyExclusiveCallbackGroup()
        self._cb_srvs  = ReentrantCallbackGroup()
        self._cb_setup = MutuallyExclusiveCallbackGroup()

        cfg = self._load_config(config_path)

        self._robstride_motor_mode: str = cfg.get('robstride_motor_mode', 'pp').lower()
        self._damiao_motor_mode:    str = cfg.get('damiao_motor_mode', 'position_velocity').lower()
        self._ezmotion_motor_mode:  str = cfg.get('ezmotion_motor_mode', 'pp').lower()

        if self._robstride_motor_mode not in _ROBSTRIDE_MODE_INT:
            raise RuntimeError(
                f'Invalid robstride_motor_mode "{self._robstride_motor_mode}". '
                f'Valid: {sorted(_ROBSTRIDE_MODE_INT)}'
            )
        if self._damiao_motor_mode not in _DAMIAO_MODE_INT:
            raise RuntimeError(
                f'Invalid damiao_motor_mode "{self._damiao_motor_mode}". '
                f'Valid: {sorted(_DAMIAO_MODE_INT)}'
            )
        if self._ezmotion_motor_mode not in _EZMOTION_MODE_INT:
            raise RuntimeError(
                f'Invalid ezmotion_motor_mode "{self._ezmotion_motor_mode}". '
                f'Valid: {sorted(_EZMOTION_MODE_INT)}'
            )

        self._debug: bool = bool(cfg.get('debug', False))
        if self._debug:
            self.get_logger().warning('DEBUG mode enabled — commands will be logged but NOT published')

        self._enable_motors_on_startup: bool = bool(cfg.get('enable_motors_on_startup', False))
        self._forwarding_enabled: bool = bool(cfg.get('forwarding_enabled', False))

        self._global_robstride_hz: Optional[float] = (
            float(cfg['robstride_active_report_hz']) if 'robstride_active_report_hz' in cfg else None
        )
        self._global_damiao_hz: Optional[float] = (
            float(cfg['damiao_active_report_hz']) if 'damiao_active_report_hz' in cfg else None
        )
        self._global_ezmotion_hz: Optional[float] = (
            float(cfg['ezmotion_active_report_hz']) if 'ezmotion_active_report_hz' in cfg else None
        )

        op_sec = cfg.get('robstride_operation_defaults', {})
        self._robstride_operation_defaults = {
            'kp': float(op_sec.get('kp', 0.0)),
            'kd': float(op_sec.get('kd', 0.0)),
        }

        self._use_jd:    bool = bool(cfg.get('use_jd', False))
        self._ik_ready:  bool = False
        self._robstride_motors: list = list(cfg.get('arm_motors', []))
        if self._use_jd:
            _cfg_dir = os.path.dirname(os.path.abspath(config_path))
            def _resolve_urdf(p: str) -> str:
                return p if os.path.isabs(p) else os.path.join(_cfg_dir, p)
            self._left_arm_urdf_path:  str = _resolve_urdf(cfg.get('left_arm_urdf_path',  ''))
            self._right_arm_urdf_path: str = _resolve_urdf(cfg.get('right_arm_urdf_path', ''))
            self._left_ee_frame:       str = cfg.get('left_ee_frame',  'arm_to_gripperL')
            self._right_ee_frame:      str = cfg.get('right_ee_frame', 'arm_to_gripperR')
            try:
                self._init_jd()
                self._ik_ready = True
                self.get_logger().info('Pinocchio models loaded — gravity compensation enabled')
            except Exception as e:
                self.get_logger().error(f'Failed to load pinocchio models: {e}')
        self._robstride_pp_defaults = self._require_section(cfg, 'robstride_pp_defaults',
            ['speed', 'acceleration', 'deceleration', 'torque_limit'])
        self._ezmotion_pp_defaults = self._require_section(cfg, 'ezmotion_pp_defaults',
            ['speed', 'acceleration', 'deceleration'])
        self._damiao_pv_defaults = self._require_section(cfg, 'damiao_pv_defaults',
            ['speed'])

        grippers_cfg = cfg.get('grippers', {})
        for field in ('gripper_open_val', 'gripper_closed_val'):
            if field not in grippers_cfg:
                self.get_logger().fatal(f'Missing required config field "grippers.{field}"')
                self._shutdown()
                return
        self._gripper_open_val:   float = float(grippers_cfg['gripper_open_val'])
        self._gripper_closed_val: float = float(grippers_cfg['gripper_closed_val'])

        torso_cfg = cfg.get('torso', {})
        if 'torso_home_mm' not in torso_cfg:
            self.get_logger().fatal('Missing required config field "torso.torso_home_mm"')
            self._shutdown()
            return
        self._torso_home_mm: float = float(torso_cfg['torso_home_mm'])

        base_and_neck_cfg = cfg.get('base_and_neck', {})
        for field in ('neck_pitch_low', 'neck_pitch_high'):
            if field not in base_and_neck_cfg:
                self.get_logger().fatal(f'Missing required config field "base_and_neck.{field}"')
                self._shutdown()
                return
        self._neck_pitch_hw_low:  float = float(base_and_neck_cfg['neck_pitch_low'])
        self._neck_pitch_hw_high: float = float(base_and_neck_cfg['neck_pitch_high'])

        self._qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._latest_states: Dict[str, MotorState] = {}

        self._nodes: Dict[str, dict] = self._parse_nodes(cfg)

        self._motor_cmd_defaults: Dict[str, dict] = {
            motor_name: motor['cmd_defaults']
            for node in self._nodes.values()
            for motor_name, motor in node['motors'].items()
        }

        self._last_go_to: Dict[str, float] = {}

        self._feedback_subs: Dict[str, object] = {}
        self._cmd_pubs:      Dict[str, object] = {}

        self._active_report_clients: Dict[str, object] = {}
        self._run_mode_clients:      Dict[str, object] = {}
        self._enable_motor_clients:  Dict[str, object] = {}

        for node_name, node in self._nodes.items():
            prefix = node['prefix']

            if prefix not in self._active_report_clients:
                self._active_report_clients[prefix] = self.create_client(
                    SetActiveReport,
                    f'{prefix}/set_active_report',
                    callback_group=self._cb_srvs,
                )
            if prefix not in self._run_mode_clients:
                self._run_mode_clients[prefix] = self.create_client(
                    SetRunMode,
                    f'{prefix}/set_run_mode',
                    callback_group=self._cb_srvs,
                )
            enable_svc = node['enable_service']
            if enable_svc not in self._enable_motor_clients:
                self._enable_motor_clients[enable_svc] = self.create_client(
                    EnableMotor,
                    enable_svc,
                    callback_group=self._cb_srvs,
                )

            for motor_name, motor in node['motors'].items():
                self._feedback_subs[motor_name] = self.create_subscription(
                    MotorState,
                    motor['feedback_topic'],
                    lambda msg, n=motor_name: self._on_motor_state(n, msg),
                    self._qos,
                    callback_group=self._cb_subs,
                )

                cmd_topic = motor['cmd_topic']
                if cmd_topic not in self._cmd_pubs:
                    self._cmd_pubs[cmd_topic] = self.create_publisher(
                        self._msg_type_for_topic(cmd_topic),
                        cmd_topic,
                        self._qos,
                    )

        self._frozen: set = set()
        self._freeze_lock = threading.Lock()

        self.create_service(
            Freeze,
            '~/freeze',
            self._srv_freeze,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            EnableMotors,
            '~/enable_motors',
            self._srv_enable_motors,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            SetBool,
            '~/set_forwarding',
            self._srv_set_forwarding,
            callback_group=self._cb_srvs,
        )

        self._arm_poses: Dict[str, dict] = {
            pose_name: {
                'positions':    {k: float(v) for k, v in pose_vals.items()
                                 if not isinstance(v, dict) and k != 'cmd_defaults'},
                'cmd_defaults': {k: float(v)
                                 for k, v in cfg.get(pose_vals.get('cmd_defaults', ''), {}).items()},
            }
            for pose_name, pose_vals in cfg.get('arm_poses', {}).items()
            if isinstance(pose_vals, dict)
        }
        self.create_service(
            EnableMotors,
            '~/set_arm_pose',
            self._srv_set_arm_pose,
            callback_group=self._cb_srvs,
        )
        if self._arm_poses:
            self.get_logger().info(f'Arm poses configured: {list(self._arm_poses.keys())}')

        self._joints_state_publish_hz: float = self._resolve_publish_hz(cfg)
        self._zenoh_session = self._open_zenoh_session(cfg.get('zenoh_config'))

        self._group_motors:     Dict[str, list]            = {}
        self._group_motor_cmd:  Dict[str, Dict[str, str]]  = {}
        self._group_is_gripper: Dict[str, bool]            = {}
        self._group_is_torso:   Dict[str, bool]            = {}

        self._zenoh_feedback_pubs: Dict[str, object] = {}
        self._zenoh_cmd_subs:      Dict[str, object] = {}

        self._motor_cmd_topics: Dict[str, str] = {
            motor_name: motor['cmd_topic']
            for node in self._nodes.values()
            for motor_name, motor in node['motors'].items()
        }

        for group_name, motors in cfg.get('zenoh_motor_groups', {}).items():
            motors = list(motors)
            self._group_motors[group_name]     = motors
            self._group_motor_cmd[group_name]  = {m: self._motor_cmd_topics[m] for m in motors if m in self._motor_cmd_topics}
            self._group_is_gripper[group_name] = 'gripper' in group_name
            self._group_is_torso[group_name]   = 'torso'   in group_name

            feedback_key = cfg.get(f'{group_name}_zenoh_key')
            if feedback_key:
                self._zenoh_feedback_pubs[group_name] = self._zenoh_session.declare_publisher(feedback_key)
                self.get_logger().info(f'Zenoh feedback pub [{group_name}]: {feedback_key}')
            else:
                self.get_logger().warning(
                    f'[zenoh_motor_groups] group "{group_name}" has no '
                    f'"{group_name}_zenoh_key" in config — feedback skipped'
                )

            cmd_key = cfg.get(f'{group_name}_cmd_zenoh_key')
            if cmd_key:
                sub = self._zenoh_session.declare_subscriber(
                    cmd_key,
                    lambda sample, g=group_name: self._on_zenoh_cmd(sample, g),
                )
                self._zenoh_cmd_subs[group_name] = sub
                self.get_logger().info(f'Zenoh cmd sub [{group_name}]: {cmd_key}')

        # Dedicated per-gripper feedback publishers (AgL → left_gripper_zenoh_key, etc.)
        self._gripper_feedback_pubs: Dict[str, object] = {}
        for motor_name, key_field in (('AgL', 'left_gripper_zenoh_key'), ('AgR', 'right_gripper_zenoh_key')):
            key = cfg.get(key_field)
            if key:
                self._gripper_feedback_pubs[motor_name] = self._zenoh_session.declare_publisher(key)
                self.get_logger().info(f'Zenoh gripper feedback pub [{motor_name}]: {key}')

        self.create_timer(
            1.0 / self._joints_state_publish_hz,
            self._publish_zenoh_feedback,
            callback_group=self._cb_subs,
        )

        self._setup_timer = self.create_timer(
            1.0, self._setup_once, callback_group=self._cb_setup
        )

        total_motors = sum(len(n['motors']) for n in self._nodes.values())
        self.get_logger().info(
            f'WebXRTeleopNode init — {len(self._nodes)} nodes, {total_motors} motors  '
            f'robstride_mode={self._robstride_motor_mode}  damiao_mode={self._damiao_motor_mode}  '
            f'joints_state_publish_hz={self._joints_state_publish_hz}'
        )

    # ── Config ────────────────────────────────────────────────────────────────

    def _require_section(self, cfg: dict, section: str, fields: list) -> Dict[str, float]:
        if section not in cfg:
            self.get_logger().fatal(f'Missing required config section [{section}]')
            self._shutdown()
            return {}
        sec = cfg[section]
        result: Dict[str, float] = {}
        for field in fields:
            if field not in sec:
                self.get_logger().fatal(f'Missing required field "{field}" in [{section}]')
                self._shutdown()
                return {}
            result[field] = float(sec[field])
        return result

    def _load_config(self, path: Path) -> dict:
        try:
            with open(path, 'rb') as f:
                return tomllib.load(f)
        except FileNotFoundError:
            self.get_logger().fatal(f'Config not found: {path}')
            raise
        except Exception as e:
            self.get_logger().fatal(f'Failed to parse config: {e}')
            raise

    def _parse_nodes(self, cfg: dict) -> Dict[str, dict]:
        nodes: Dict[str, dict] = {}

        for section_key, section_val in cfg.items():
            if not isinstance(section_val, dict):
                continue

            motors = {
                k: v for k, v in section_val.items()
                if isinstance(v, dict) and 'motor_type' in v
            }
            if not motors:
                continue

            if 'active_report_hz' in section_val:
                report_hz: Optional[float] = float(section_val['active_report_hz'])
            else:
                motor_types = {m['motor_type'].lower() for m in motors.values()}
                if 'ezmotion' in motor_types and self._global_ezmotion_hz is not None:
                    report_hz = self._global_ezmotion_hz
                elif 'robstride' in motor_types and self._global_robstride_hz is not None:
                    report_hz = self._global_robstride_hz
                elif 'damiao' in motor_types and self._global_damiao_hz is not None:
                    report_hz = self._global_damiao_hz
                else:
                    report_hz = None

            prefix = f'/{section_key}'
            enable_svc = section_val.get('enable_disable_service_name') or f'{prefix}/enable_motor'

            cmd_pattern      = section_val.get('cmd_topic_pattern', '')
            feedback_pattern = section_val.get('feedback_topic_pattern', '')
            node_cmd_defaults = {k: float(v) for k, v in cfg.get(section_val.get('cmd_defaults', ''), {}).items()}

            nodes[section_key] = {
                'prefix':           prefix,
                'active_report_hz': report_hz,
                'enable_service':   enable_svc,
                'motors': {
                    motor_name: {
                        'cmd_topic':      (motor_val.get('cmd_topic_pattern') or cmd_pattern).format(motor_name=motor_name),
                        'feedback_topic': (motor_val.get('feedback_topic_pattern') or feedback_pattern).format(motor_name=motor_name),
                        'motor_type':     motor_val['motor_type'].lower(),
                        'mode':           motor_val.get('mode', '').lower(),
                        'cmd_defaults':   {k: float(v) for k, v in cfg.get(motor_val.get('cmd_defaults', ''), {}).items()} or node_cmd_defaults,
                    }
                    for motor_name, motor_val in motors.items()
                },
            }

        return nodes

    def _resolve_publish_hz(self, cfg: dict) -> float:
        if 'joints_state_publish_hz' in cfg:
            return float(cfg['joints_state_publish_hz'])
        if 'robstride_active_report_hz' in cfg:
            return float(cfg['robstride_active_report_hz'])
        if 'damiao_active_report_hz' in cfg:
            return float(cfg['damiao_active_report_hz'])
        for section_val in cfg.values():
            if isinstance(section_val, dict) and 'active_report_hz' in section_val:
                return float(section_val['active_report_hz'])
        self.get_logger().fatal(
            'joints_state_publish_hz not set and no active_report_hz found in config'
        )
        self._shutdown()
        return 30.0  # unreachable; satisfies type and prevents 1.0/None before shutdown completes

    def _open_zenoh_session(self, config_path: Optional[str]):
        if config_path and os.path.isfile(config_path):
            session = zenoh.open(zenoh.Config.from_file(config_path))
            self.get_logger().info(f'Zenoh session opened (config: {config_path})')
        else:
            session = zenoh.open(zenoh.Config())
            self.get_logger().info('Zenoh session opened (default config)')
        return session

    def _gripper_to_zenoh(self, motor_pos: float) -> float:
        # norm=0.0 → open (URDF 0.0), norm=1.0 → closed (URDF 0.0475)
        span = self._gripper_closed_val - self._gripper_open_val
        if span == 0.0:
            return 0.0
        return max(0.0, min(1.0, (motor_pos - self._gripper_open_val) / span))

    def _zenoh_to_gripper(self, cmd: float) -> float:
        # cmd norm=0.0 → open motor pos, norm=1.0 → closed motor pos
        cmd = max(0.0, min(1.0, cmd))
        return self._gripper_open_val + cmd * (self._gripper_closed_val - self._gripper_open_val)

    def _neck_pitch_zenoh_to_hw(self, zenoh_val: float) -> float:
        span = _NECK_PITCH_ZENOH_HIGH - _NECK_PITCH_ZENOH_LOW
        norm = max(0.0, min(1.0, (zenoh_val - _NECK_PITCH_ZENOH_LOW) / span))
        return self._neck_pitch_hw_low + norm * (self._neck_pitch_hw_high - self._neck_pitch_hw_low)

    def _neck_pitch_hw_to_zenoh(self, hw_val: float) -> float:
        span = self._neck_pitch_hw_high - self._neck_pitch_hw_low
        if span == 0.0:
            return _NECK_PITCH_ZENOH_LOW
        norm = max(0.0, min(1.0, (hw_val - self._neck_pitch_hw_low) / span))
        return _NECK_PITCH_ZENOH_LOW + norm * (_NECK_PITCH_ZENOH_HIGH - _NECK_PITCH_ZENOH_LOW)

    def _msg_type_for_topic(self, topic: str):
        for suffix, msg_type in _CMD_TOPIC_MSG_TYPE.items():
            if topic.endswith(suffix):
                return msg_type
        return PositionPPCommand

    # ── Startup ───────────────────────────────────────────────────────────────

    def _setup_once(self) -> None:
        self._setup_timer.cancel()
        self._setup_timer = None

        for node_name, node in self._nodes.items():
            prefix = node['prefix']

            if node['active_report_hz'] is not None:
                self._set_active_report(
                    self._active_report_clients[prefix],
                    prefix,
                    enable=True,
                    hz=node['active_report_hz'],
                )

            run_client = self._run_mode_clients[prefix]
            if not run_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'{prefix}/set_run_mode not available — skipping {node_name}')
                continue

            for motor_name, motor in node['motors'].items():
                motor_mode = motor['mode']
                if motor['motor_type'] == 'robstride':
                    mode_map = _ROBSTRIDE_MODE_INT
                    default  = self._robstride_motor_mode
                elif motor['motor_type'] == 'ezmotion':
                    mode_map = _EZMOTION_MODE_INT
                    default  = self._ezmotion_motor_mode
                else:
                    mode_map = _DAMIAO_MODE_INT
                    default  = self._damiao_motor_mode
                mode_int = mode_map.get(motor_mode) if motor_mode else None
                if mode_int is None:
                    mode_int = mode_map[default]

                req                          = SetRunMode.Request()
                req.name                     = motor_name
                req.mode                     = mode_int
                req.automatic_enable_disable = False
                res = run_client.call(req)
                if res.success:
                    self.get_logger().info(
                        f'[{node_name}/{motor_name}] run mode set to {mode_int}'
                    )
                else:
                    self.get_logger().error(
                        f'[{node_name}/{motor_name}] set_run_mode failed: {res.message}'
                    )

        if self._enable_motors_on_startup:
            req = EnableMotors.Request()
            req.name        = ''
            req.enable      = True
            req.clear_fault = False
            self._srv_enable_motors(req, EnableMotors.Response())
            self.get_logger().info('Motors enabled (enable_motors_on_startup)')

        if self._forwarding_enabled:
            self.get_logger().info('Command forwarding enabled (forwarding_enabled)')

        self.get_logger().info('Setup complete')

    def _set_active_report(self, client, prefix: str, enable: bool, hz: float) -> None:
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{prefix}/set_active_report not available')
            return
        req        = SetActiveReport.Request()
        req.name   = 'all'
        req.enable = enable
        req.hz     = hz if enable else 0.0
        res    = client.call(req)
        action = 'enabled' if enable else 'disabled'
        if res.success:
            self.get_logger().info(f'[{prefix}] active report {action} @ {hz} Hz: {res.message}')
        else:
            self.get_logger().error(f'[{prefix}] active report {action}: {res.message}')

    # ── Freeze service ────────────────────────────────────────────────────────

    def _motor_node_name(self, motor_name: str) -> Optional[str]:
        for node_name, node in self._nodes.items():
            if motor_name in node['motors']:
                return node_name
        return None

    def _is_frozen(self, motor_name: str) -> bool:
        with self._freeze_lock:
            if motor_name in self._frozen:
                return True
            node_name = self._motor_node_name(motor_name)
            return node_name is not None and node_name in self._frozen

    def _srv_freeze(self, req: Freeze.Request, res: Freeze.Response):
        name   = req.name.strip()
        freeze = req.freeze

        with self._freeze_lock:
            if not name:
                keys = set(self._nodes.keys())
                if freeze:
                    self._frozen |= keys
                    res.message = 'All nodes frozen'
                else:
                    self._frozen -= keys
                    res.message = 'All nodes unfrozen'
            elif name in self._nodes:
                if freeze:
                    self._frozen.add(name)
                    res.message = f'Node "{name}" frozen'
                else:
                    self._frozen.discard(name)
                    res.message = f'Node "{name}" unfrozen'
            elif self._motor_node_name(name) is not None:
                if freeze:
                    self._frozen.add(name)
                    res.message = f'Motor "{name}" frozen'
                else:
                    self._frozen.discard(name)
                    res.message = f'Motor "{name}" unfrozen'
            else:
                res.success = False
                res.frozen  = False
                res.message = f'Unknown node or motor: "{name}"'
                return res

        res.success = True
        res.frozen  = freeze
        self.get_logger().info(res.message)
        return res

    # ── Enable motors service ─────────────────────────────────────────────────

    def _srv_enable_motors(self, req: EnableMotors.Request, res: EnableMotors.Response):
        name = req.name.strip()

        calls: list = []
        if not name or name == 'all':
            for node in self._nodes.values():
                for motor_name in node['motors']:
                    calls.append((node['enable_service'], motor_name))
        elif name in self._nodes:
            node = self._nodes[name]
            for motor_name in node['motors']:
                calls.append((node['enable_service'], motor_name))
        else:
            node_name = self._motor_node_name(name)
            if node_name is None:
                res.success = False
                res.message = f'Unknown node or motor: "{name}"'
                return res
            calls.append((self._nodes[node_name]['enable_service'], name))

        action  = 'enabled' if req.enable else 'disabled'
        failed  = []

        for enable_svc, motor_name in calls:
            client = self._enable_motor_clients.get(enable_svc)
            if client is None or not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'{enable_svc} not available — skipping {motor_name}')
                failed.append(motor_name)
                continue

            en_req             = EnableMotor.Request()
            en_req.name        = motor_name
            en_req.enable      = req.enable
            en_req.clear_fault = req.clear_fault
            en_res = client.call(en_req)

            if en_res.success:
                self.get_logger().info(f'[{motor_name}] {action}: {en_res.message}')
            else:
                self.get_logger().error(f'[{motor_name}] enable_motor failed: {en_res.message}')
                failed.append(motor_name)

        if failed:
            res.success = False
            res.message = f'Partial failure — {action} failed for: {failed}'
        else:
            res.success = True
            res.message = f'{len(calls)} motor(s) {action}'

        return res

    # ── Forwarding control ────────────────────────────────────────────────────

    def _srv_set_forwarding(self, req: SetBool.Request, res: SetBool.Response):
        self._forwarding_enabled = req.data
        action = 'started' if req.data else 'stopped'
        res.success = True
        res.message = f'Command forwarding {action}'
        self.get_logger().info(res.message)
        return res

    # ── Set arm pose service ─────────────────────────────────────────────────

    def _srv_set_arm_pose(self, req: EnableMotors.Request, res: EnableMotors.Response):
        name = req.name.strip()
        if not name:
            res.success = False
            res.message = f'Pose name required. Available: {list(self._arm_poses.keys())}'
            return res

        pose = self._arm_poses.get(name)
        if pose is None:
            res.success = False
            res.message = f'Unknown pose: "{name}". Available: {list(self._arm_poses.keys())}'
            return res

        pose_positions    = pose['positions']
        pose_cmd_defaults = pose['cmd_defaults'] or None

        sent: list = []
        skipped: list = []
        for motor_name, position in pose_positions.items():
            cmd_topic = self._motor_cmd_topics.get(motor_name)
            if not cmd_topic or cmd_topic not in self._cmd_pubs:
                skipped.append(motor_name)
                self.get_logger().warning(f'[set_arm_pose/{name}] {motor_name}: no cmd publisher — skipped')
                continue
            self._publish_motor_cmd(motor_name, cmd_topic, position, cmd_defaults=pose_cmd_defaults)
            self.get_logger().info(f'[set_arm_pose/{name}] {motor_name} → {position:.4f}')
            sent.append(motor_name)

        res.success = True
        res.message = f'Pose "{name}": sent {len(sent)} motor(s): {sent}'
        if skipped:
            res.message += f'; skipped {skipped}'
        return res

    # ── Zenoh feedback (ROS → Zenoh) ──────────────────────────────────────────

    def _publish_zenoh_feedback(self) -> None:
        for group_name, pub in self._zenoh_feedback_pubs.items():
            is_gripper = self._group_is_gripper[group_name]
            is_torso   = self._group_is_torso[group_name]
            array = feather_pb2.JointStatesArray()
            for motor_name in self._group_motors[group_name]:
                state = self._latest_states.get(motor_name)
                if state is None:
                    continue
                joint          = array.joints.add()
                joint.id       = HW_TO_SIM.get(motor_name, motor_name)
                if motor_name in ('AgL', 'AgR'):
                    norm           = self._gripper_to_zenoh(float(state.position))
                    joint.position = norm * _GRIPPER_URDF_LIMIT_M
                elif motor_name == 'NpC':
                    joint.position = self._neck_pitch_hw_to_zenoh(float(state.position))
                elif is_torso:
                    joint.position = (float(state.position) - self._torso_home_mm) / 1000.0
                else:
                    joint.position = float(state.position)
                joint.velocity = float(state.velocity)
                joint.torque   = float(state.torque)

            if array.joints:
                pub.put(array.SerializeToString())

        for motor_name, pub in self._gripper_feedback_pubs.items():
            state = self._latest_states.get(motor_name)
            if state is None:
                continue
            array = feather_pb2.JointStatesArray()
            joint          = array.joints.add()
            joint.id       = HW_TO_SIM.get(motor_name, motor_name)
            norm           = self._gripper_to_zenoh(float(state.position))
            joint.position = norm * _GRIPPER_URDF_LIMIT_M
            joint.velocity = float(state.velocity)
            joint.torque   = float(state.torque)
            pub.put(array.SerializeToString())

    # ── Zenoh command (Zenoh → ROS) ───────────────────────────────────────────

    def _on_zenoh_cmd(self, sample, group_name: str) -> None:
        array = feather_pb2.JointControlArray()
        try:
            array.ParseFromString(sample.payload.to_bytes())
        except Exception as e:
            self.get_logger().error(f'[{group_name}] bad JointControlArray: {e}')
            return

        positions_by_id: Dict[str, float] = {
            SIM_TO_HW.get(j.id, j.id): float(j.command)
            for j in array.joints
            if j.cmd_type == feather_pb2.JointControl.POSITION
        }
        velocities_by_id: Dict[str, float] = {
            SIM_TO_HW.get(j.id, j.id): float(j.command)
            for j in array.joints
            if j.cmd_type == feather_pb2.JointControl.VELOCITY
        }

        if self._debug:
            joints_str = ', '.join(f'{k}={v:.4f}' for k, v in positions_by_id.items())
            vel_str    = ', '.join(f'{k}={v:.4f}' for k, v in velocities_by_id.items())
            self.get_logger().info(f'[DEBUG] cmd [{group_name}]: pos={joints_str} vel={vel_str}')

        if not self._forwarding_enabled:
            return

        is_torso = self._group_is_torso[group_name]

        # Gravity compensation — only for arm groups when use_jd is enabled
        gravity_torques: Dict[str, float] = {}
        if self._use_jd and self._ik_ready and not is_torso and not self._group_is_gripper[group_name]:
            sfx = 'L' if 'left' in group_name else ('R' if 'right' in group_name else None)
            if sfx is not None:
                try:
                    gravity_torques = self._compute_arm_gravity_from_positions(positions_by_id, sfx)
                except Exception as e:
                    self.get_logger().error(f'[{group_name}] gravity compensation failed: {e}')

        for motor_name in self._group_motors[group_name]:
            if self._is_frozen(motor_name):
                continue

            cmd_topic = self._group_motor_cmd[group_name].get(motor_name)
            if not cmd_topic or cmd_topic not in self._cmd_pubs:
                continue

            if motor_name in velocities_by_id and cmd_topic.endswith('cmd_velocity'):
                self._publish_motor_cmd(motor_name, cmd_topic, velocities_by_id[motor_name])
            elif motor_name in positions_by_id:
                value = positions_by_id[motor_name]
                if motor_name in ('AgL', 'AgR'):
                    norm  = max(0.0, min(1.0, value / _GRIPPER_URDF_LIMIT_M))
                    value = self._zenoh_to_gripper(norm)
                elif motor_name == 'NpC':
                    value = self._neck_pitch_zenoh_to_hw(value)
                elif is_torso:
                    value = self._torso_home_mm + value * 1000.0
                self._publish_motor_cmd(motor_name, cmd_topic, value,
                                        torque_ff=gravity_torques.get(motor_name))

    def _publish_motor_cmd(self, motor_name: str, cmd_topic: str, value: float,
                           cmd_defaults: Optional[dict] = None,
                           torque_ff: Optional[float] = None) -> None:
        pub = self._cmd_pubs[cmd_topic]
        d   = cmd_defaults if cmd_defaults is not None else self._motor_cmd_defaults.get(motor_name, {})
        if cmd_topic.endswith('cmd_operation'):
            cmd           = OperationCommand()
            cmd.name      = motor_name
            cmd.position  = value
            cmd.velocity  = 0.0
            cmd.torque_ff = torque_ff if torque_ff is not None else 0.0
            cmd.kp        = d.get('kp',        self._robstride_operation_defaults['kp'])
            cmd.kd        = d.get('kd',        self._robstride_operation_defaults['kd'])
        elif cmd_topic.endswith('cmd_velocity'):
            cmd               = VelocityCommand()
            cmd.name          = motor_name
            cmd.velocity      = value
            cmd.acceleration  = d.get('acceleration',  20.0)
            cmd.current_limit = d.get('current_limit', 23.0)
        elif cmd_topic.endswith('go_to'):
            if self._last_go_to.get(cmd_topic) == value:
                return
            self._last_go_to[cmd_topic] = value
            cmd      = Float64()
            cmd.data = value
        elif cmd_topic.endswith('cmd_position_csp'):
            cmd               = PositionCSPCommand()
            cmd.name          = motor_name
            cmd.position      = value
            cmd.speed_limit   = d.get('speed',         self._robstride_pp_defaults['speed'])
            cmd.current_limit = d.get('current_limit', 0.0)
        elif cmd_topic.endswith('cmd_position_pv'):
            cmd          = PositionPPCommand()
            cmd.name     = motor_name
            cmd.position = value
            cmd.speed    = d.get('speed', self._damiao_pv_defaults['speed'])
        else:
            cmd              = PositionPPCommand()
            cmd.name         = motor_name
            cmd.position     = value
            cmd.speed        = d.get('speed',        self._robstride_pp_defaults['speed'])
            cmd.acceleration = d.get('acceleration', self._robstride_pp_defaults['acceleration'])
            cmd.deceleration = d.get('deceleration', self._robstride_pp_defaults['deceleration'])
            cmd.torque_limit = d.get('torque_limit', self._robstride_pp_defaults['torque_limit'])
        if self._debug:
            self.get_logger().info(f'[DEBUG] {cmd_topic} → {motor_name} value={value:.4f}')
        else:
            pub.publish(cmd)

    # ── Motor state callback ──────────────────────────────────────────────────

    def _on_motor_state(self, motor_name: str, msg: MotorState) -> None:
        self._latest_states[motor_name] = msg

    # ── IK ───────────────────────────────────────────────────────────────
    def _init_jd(self):
        self.left_arm_pin_model  = pin.buildModelFromUrdf(self._left_arm_urdf_path)
        self.right_arm_pin_model = pin.buildModelFromUrdf(self._right_arm_urdf_path)

        self.left_arm_pin_data  = self.left_arm_pin_model.createData()
        self.right_arm_pin_data = self.right_arm_pin_model.createData()

        self.left_ee_id  = self.left_arm_pin_model.getFrameId(self._left_ee_frame)
        self.right_ee_id = self.right_arm_pin_model.getFrameId(self._right_ee_frame)

        self.left_arm_pin_q  = pin.neutral(self.left_arm_pin_model)
        self.right_arm_pin_q = pin.neutral(self.right_arm_pin_model)

        self._left_arm_joint_map  = self._build_joint_map(self.left_arm_pin_model,  'L')
        self._right_arm_joint_map = self._build_joint_map(self.right_arm_pin_model, 'R')

    def _build_joint_map(self, model, sfx: str) -> Dict[str, int]:
        """Return {base_motor_name: pinocchio_joint_id} for one arm."""
        result: Dict[str, int] = {}
        for base in self._robstride_motors:
            joint_name = base + sfx
            jid = model.getJointId(joint_name)
            if jid < len(model.joints):
                result[base] = jid
            else:
                self.get_logger().warning(f'Joint {joint_name!r} not found in {sfx} arm model (motor {joint_name})')
        return result
        
    def _compute_joint_dynamics(self, model, data, q):
        return pin.computeGeneralizedGravity(model, data, q)

    def _build_arm_q_v(self, state_dict: dict, motor_sfx: str, model, joint_map: Dict[str, int]):
        """Build pinocchio q and v arrays from a motor state dict for one arm."""
        q = pin.neutral(model)
        v = np.zeros(model.nv)
        for base in self._robstride_motors:
            jid = joint_map.get(base)
            if jid is None:
                continue
            ms = state_dict.get(base + motor_sfx)
            if ms is None:
                continue
            q[model.joints[jid].idx_q] = ms.position
            v[model.joints[jid].idx_v] = ms.velocity
        return q, v

    def _compute_arm_gravity_from_positions(self, positions_by_motor: Dict[str, float],
                                            sfx: str) -> Dict[str, float]:
        """Return {motor_name: gravity_torque} for one arm given position dict."""
        model = self.left_arm_pin_model  if sfx == 'L' else self.right_arm_pin_model
        data  = self.left_arm_pin_data   if sfx == 'L' else self.right_arm_pin_data
        jmap  = self._left_arm_joint_map if sfx == 'L' else self._right_arm_joint_map
        q = pin.neutral(model)
        for base, jid in jmap.items():
            pos = positions_by_motor.get(base + sfx)
            if pos is not None:
                q[model.joints[jid].idx_q] = pos
        tau = self._compute_joint_dynamics(model, data, q)
        return {base + sfx: float(tau[model.joints[jid].idx_v]) for base, jid in jmap.items()}


    # ── Shutdown ──────────────────────────────────────────────────────────────

    def _shutdown(self) -> None:
        print('[webxr_teleop] Shutting down …')

        seen_prefixes: set = set()
        for node in getattr(self, '_nodes', {}).values():
            prefix = node['prefix']
            if node['active_report_hz'] is None or prefix in seen_prefixes:
                continue
            seen_prefixes.add(prefix)
            client = self._active_report_clients.get(prefix)
            if not client or not client.service_is_ready():
                continue
            req        = SetActiveReport.Request()
            req.name   = 'all'
            req.enable = False
            req.hz     = 0.0
            done   = threading.Event()
            future = client.call_async(req)
            future.add_done_callback(lambda _: done.set())
            done.wait(timeout=2.0)
            print(f'[webxr_teleop] [{prefix}] active report disabled')

        for group_name, sub in getattr(self, '_zenoh_cmd_subs', {}).items():
            try:
                sub.undeclare()
            except Exception as e:
                print(f'[webxr_teleop] zenoh sub undeclare [{group_name}]: {e}')
        if hasattr(self, '_zenoh_cmd_subs'):
            self._zenoh_cmd_subs.clear()

        for group_name, pub in getattr(self, '_zenoh_feedback_pubs', {}).items():
            try:
                pub.undeclare()
            except Exception as e:
                print(f'[webxr_teleop] zenoh pub undeclare [{group_name}]: {e}')
        if hasattr(self, '_zenoh_feedback_pubs'):
            self._zenoh_feedback_pubs.clear()

        for motor_name, pub in getattr(self, '_gripper_feedback_pubs', {}).items():
            try:
                pub.undeclare()
            except Exception as e:
                print(f'[webxr_teleop] zenoh gripper pub undeclare [{motor_name}]: {e}')
        if hasattr(self, '_gripper_feedback_pubs'):
            self._gripper_feedback_pubs.clear()

        if hasattr(self, '_zenoh_session'):
            self._zenoh_session.close()
            print('[webxr_teleop] Zenoh session closed')

        rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    parser = argparse.ArgumentParser(description='WebXR teleop CLI')
    parser.add_argument(
        '--config',
        type=Path,
        required=True,
        help='Path to config TOML',
    )
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = WebXRTeleopNode(config_path=parsed.config)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    _stop = threading.Event()
    signal.signal(signal.SIGINT, lambda sig, frame: _stop.set())

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    _stop.wait()

    node._shutdown()
    executor.shutdown()
    spin_thread.join(timeout=3.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
