import os
import threading
import tomllib
from typing import Callable, Dict

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.msg import MotorState, OperationCommand, PositionPPCommand
from mimic import transforms as _transforms
from std_srvs.srv import SetBool
from custom_interfaces.srv import (
    EnableMimicMotors,
    EnableMotor,
    SetActiveReport,
    SetMimicMode,
    SetMimicParams,
    SetMimicTarget,
    SetRunMode,
)

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

_VALID_MODES   = frozenset({'operation'})
_VALID_TARGETS = frozenset({'left_arm', 'right_arm'})

_MODE_RUN_MODE_INT = {
    'operation': 0,  # OPERATION (MIT-style)
}

_DAMIAO_MODE_INT = {
    'mit':               1,
    'position_velocity': 2,
    'velocity':          3,
}


class MimicWDNode(Node):

    def __init__(self):
        super().__init__('mimic_wd_node')

        self._cb_subs  = MutuallyExclusiveCallbackGroup()
        self._cb_srvs  = ReentrantCallbackGroup()
        self._cb_setup = MutuallyExclusiveCallbackGroup()
        self._switch_lock = threading.Lock()

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        cfg = self._load_config(config_path)

        urdf_pkg = cfg.get('urdf_prefix', 'mimic_wd')
        _pkg_share = get_package_share_directory(urdf_pkg)
        self._left_arm_urdf_path:  str = os.path.join(_pkg_share, cfg.get('left_arm_urdf_path',  ''))
        self._right_arm_urdf_path: str = os.path.join(_pkg_share, cfg.get('right_arm_urdf_path', ''))
        self._left_ee_frame:       str = cfg.get('left_ee_frame',  'left_ee')
        self._right_ee_frame:      str = cfg.get('right_ee_frame', 'right_ee')
        self._base_frame:          str = cfg.get('base_frame',     'base_link')

        self._left_arm_prefix:      str = cfg['left_arm_node_prefix']
        self._right_arm_prefix:     str = cfg['right_arm_node_prefix']
        self._left_gripper_prefix:  str = cfg['left_gripper_node_prefix']
        self._right_gripper_prefix: str = cfg['right_gripper_node_prefix']
        self._robstride_motors:     list = cfg['robstride_motors']
        self._damiao_motors:        list = cfg.get('damiao_motors', [])

        self._forward_transforms: Dict[str, Callable] = self._load_transform_map(
            cfg.get('robstride_transform_map', {})
        )
        self._inverse_transforms: Dict[str, Callable] = self._load_transform_map(
            cfg.get('robstride_inverse_transform_map', {})
        )
        self._damiao_forward_transforms: Dict[str, Callable] = self._load_transform_map(
            cfg.get('damiao_transform_map', {})
        )
        self._damiao_inverse_transforms: Dict[str, Callable] = self._load_transform_map(
            cfg.get('damiao_inverse_transform_map', {})
        )

        self._debug:            bool  = bool(cfg.get('debug', False))
        self._visualize_rviz:   bool  = bool(cfg.get('visualize_rviz', False))
        self._active_report_hz: float = float(cfg.get('active_report_hz', 30.0))
        self._ik_hz:            float = float(cfg.get('ik_hz', 0.0))
        self._use_ik:           bool  = bool(cfg.get('use_ik', False))
        self._mode:             str   = cfg.get('robstride_mode', 'pp').lower()
        self._damiao_mode:      str   = cfg.get('damiao_mode', 'position_velocity').lower()

        self._robstride_operation_defaults: Dict[str, float] = {
            'velocity':  float(cfg.get('robstride_operation_defaults', {}).get('velocity',  0.0)),
            'torque_ff': float(cfg.get('robstride_operation_defaults', {}).get('torque_ff', 0.0)),
        }
        self._damiao_pv_defaults: Dict[str, float] = {
            'speed': float(cfg.get('damiao_pv_defaults', {}).get('speed', 2.0)),
        }
        self._damiao_mit_defaults: Dict[str, float] = {
            'velocity':  float(cfg.get('damiao_mit_defaults', {}).get('velocity',  0.0)),
            'torque_ff': float(cfg.get('damiao_mit_defaults', {}).get('torque_ff', 0.0)),
        }

        if self._mode not in _VALID_MODES:
            raise RuntimeError(f'Invalid mode {self._mode!r}. Valid: {sorted(_VALID_MODES)}')

        self._qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._ee_marker_pub = self.create_publisher(MarkerArray, '~/ee_markers', 10) if self._visualize_rviz else None

        # Mutable direction state — protected by _switch_lock during switches
        self._target_node:          str            = ''
        self._source_prefix:        str            = ''
        self._target_prefix:        str            = ''
        self._damiao_source_prefix: str            = ''
        self._damiao_target_prefix: str            = ''
        self._motor_map:            Dict[str, str] = {}
        self._transforms:           Dict[str, Callable] = {}
        self._latest_src_state:     Dict[str, MotorState] = {}
        self._latest_tgt_state:     Dict[str, MotorState] = {}
        self._ready:                bool           = False

        # Placeholders filled by _setup_direction
        self._state_subs:             Dict[str, object] = {}
        self._tgt_state_subs:         Dict[str, object] = {}
        self._operation_pubs:         Dict[str, object] = {}
        self._debug_operation_pubs:   Dict[str, object] = {}
        self._damiao_pv_pubs:          Dict[str, object] = {}
        self._damiao_debug_pv_pubs:    Dict[str, object] = {}
        self._damiao_mit_pubs:         Dict[str, object] = {}
        self._damiao_debug_mit_pubs:   Dict[str, object] = {}
        self._damiao_state_subs:          Dict[str, object] = {}
        self._tgt_damiao_state_subs:      Dict[str, object] = {}
        self._damiao_motor_map:           Dict[str, str] = {}
        self._damiao_transforms:          Dict[str, Callable] = {}
        self._latest_src_damiao_state:    Dict[str, MotorState] = {}
        self._latest_tgt_damiao_state:    Dict[str, MotorState] = {}
        self._active_report_src_client        = None
        self._active_report_tgt_client        = None
        self._set_run_mode_client             = None
        self._enable_motor_client             = None
        self._damiao_active_report_src_client = None
        self._damiao_active_report_tgt_client = None
        self._damiao_set_run_mode_client      = None
        self._damiao_enable_motor_client      = None

        # Pinocchio direction pointers — updated by _setup_direction
        self._src_pin_model  = None
        self._src_pin_data   = None
        self._src_ee_id      = None
        self._src_joint_map: Dict[str, int] = {}
        self._src_sfx:       str = ''
        self._tgt_pin_model  = None
        self._tgt_pin_data   = None
        self._tgt_ee_id      = None
        self._tgt_joint_map: Dict[str, int] = {}
        self._tgt_sfx:       str = ''

        self.create_service(
            SetMimicMode,
            '~/set_mode',
            self._srv_set_mode,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            SetMimicParams,
            '~/set_params',
            self._srv_set_params,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            EnableMimicMotors,
            '~/enable_motors',
            self._srv_enable_motors,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            SetMimicTarget,
            '~/switch_target',
            self._srv_switch_target,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            SetBool,
            '~/set_debug',
            self._srv_set_debug,
            callback_group=self._cb_srvs,
        )

        if self._ik_hz > 0.0:
            self.create_timer(
                1.0 / self._ik_hz, self._on_ik_timer, callback_group=self._cb_subs
            )

        initial_target = cfg.get('target_node', 'right_arm').lower()
        if initial_target not in _VALID_TARGETS:
            raise RuntimeError(f'Invalid target_node {initial_target!r}. Valid: {sorted(_VALID_TARGETS)}')

        self._init_jd()
        self._setup_direction(initial_target)

    # ── Config ────────────────────────────────────────────────────────────────

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

    def _load_transform_map(self, raw: dict) -> Dict[str, Callable]:
        result: Dict[str, Callable] = {}
        for base, fn_name in raw.items():
            fn = getattr(_transforms, fn_name, None)
            if fn is None:
                self.get_logger().warning(
                    f'Transform {fn_name!r} not found in transforms.py — using passthrough'
                )
                result[base] = _transforms.passthrough
            else:
                result[base] = fn
        return result

    # ── Direction setup / teardown ────────────────────────────────────────────

    def _setup_direction(self, target_node: str) -> None:
        """Wire up subs, pubs, and service clients for the given target direction."""
        self._ready = False

        # Tear down previous subscriptions
        for sub in self._state_subs.values():
            self.destroy_subscription(sub)
        self._state_subs.clear()
        for sub in self._tgt_state_subs.values():
            self.destroy_subscription(sub)
        self._tgt_state_subs.clear()
        for sub in self._damiao_state_subs.values():
            self.destroy_subscription(sub)
        self._damiao_state_subs.clear()
        for sub in self._tgt_damiao_state_subs.values():
            self.destroy_subscription(sub)
        self._tgt_damiao_state_subs.clear()
        self._latest_src_state.clear()
        self._latest_tgt_state.clear()
        self._latest_src_damiao_state.clear()
        self._latest_tgt_damiao_state.clear()

        # Disable active reporting on old source and target, disable old target motors
        if self._active_report_src_client is not None and self._active_report_src_client.service_is_ready():
            self._blocking_set_active_report(self._active_report_src_client, self._source_prefix, enable=False)
        if self._active_report_tgt_client is not None and self._active_report_tgt_client.service_is_ready():
            self._blocking_set_active_report(self._active_report_tgt_client, self._target_prefix, enable=False)
        if self._damiao_active_report_src_client is not None and self._damiao_active_report_src_client.service_is_ready():
            self._blocking_set_active_report(self._damiao_active_report_src_client, self._damiao_source_prefix, enable=False)
        if self._damiao_active_report_tgt_client is not None and self._damiao_active_report_tgt_client.service_is_ready():
            self._blocking_set_active_report(self._damiao_active_report_tgt_client, self._damiao_target_prefix, enable=False)
        if self._enable_motor_client is not None and self._enable_motor_client.service_is_ready():
            for tgt_motor in self._motor_map.values():
                self._blocking_enable_motors(self._enable_motor_client, self._target_prefix, enable=False, name=tgt_motor)
        if self._damiao_enable_motor_client is not None and self._damiao_enable_motor_client.service_is_ready():
            for tgt_motor in self._damiao_motor_map.values():
                self._blocking_enable_motors(self._damiao_enable_motor_client, self._damiao_target_prefix, enable=False, name=tgt_motor)

        # Tear down previous service clients
        for client in [
            self._active_report_src_client,
            self._active_report_tgt_client,
            self._set_run_mode_client,
            self._enable_motor_client,
            self._damiao_active_report_src_client,
            self._damiao_active_report_tgt_client,
            self._damiao_set_run_mode_client,
            self._damiao_enable_motor_client,
        ]:
            if client is not None:
                self.destroy_client(client)

        self._target_node = target_node

        if target_node == 'right_arm':
            src_prefix, tgt_prefix, src_sfx, tgt_sfx = (
                self._left_arm_prefix, self._right_arm_prefix, 'L', 'R'
            )
            active_transforms        = self._forward_transforms
            damiao_src_prefix        = self._left_gripper_prefix
            damiao_tgt_prefix        = self._right_gripper_prefix
            damiao_active_transforms = self._damiao_forward_transforms
            src_pin_model, src_pin_data = self.left_arm_pin_model,  self.left_arm_pin_data
            tgt_pin_model, tgt_pin_data = self.right_arm_pin_model, self.right_arm_pin_data
            src_ee_id     = self.left_ee_id
            tgt_ee_id     = self.right_ee_id
            src_joint_map = self._left_arm_joint_map
            tgt_joint_map = self._right_arm_joint_map
        else:
            src_prefix, tgt_prefix, src_sfx, tgt_sfx = (
                self._right_arm_prefix, self._left_arm_prefix, 'R', 'L'
            )
            active_transforms        = self._inverse_transforms
            damiao_src_prefix        = self._right_gripper_prefix
            damiao_tgt_prefix        = self._left_gripper_prefix
            damiao_active_transforms = self._damiao_inverse_transforms
            src_pin_model, src_pin_data = self.right_arm_pin_model, self.right_arm_pin_data
            tgt_pin_model, tgt_pin_data = self.left_arm_pin_model,  self.left_arm_pin_data
            src_ee_id     = self.right_ee_id
            tgt_ee_id     = self.left_ee_id
            src_joint_map = self._right_arm_joint_map
            tgt_joint_map = self._left_arm_joint_map

        self._source_prefix        = src_prefix
        self._target_prefix        = tgt_prefix
        self._damiao_source_prefix = damiao_src_prefix
        self._damiao_target_prefix = damiao_tgt_prefix
        self._src_pin_model        = src_pin_model
        self._src_pin_data         = src_pin_data
        self._src_ee_id            = src_ee_id
        self._src_joint_map        = src_joint_map
        self._src_sfx              = src_sfx
        self._tgt_pin_model        = tgt_pin_model
        self._tgt_pin_data         = tgt_pin_data
        self._tgt_ee_id            = tgt_ee_id
        self._tgt_joint_map        = tgt_joint_map
        self._tgt_sfx              = tgt_sfx

        self._motor_map  = {b + src_sfx: b + tgt_sfx for b in self._robstride_motors}
        self._transforms = {
            b + src_sfx: active_transforms.get(b, _transforms.passthrough)
            for b in self._robstride_motors
        }
        self._damiao_motor_map  = {b + src_sfx: b + tgt_sfx for b in self._damiao_motors}
        self._damiao_transforms = {
            b + src_sfx: damiao_active_transforms.get(b, _transforms.passthrough)
            for b in self._damiao_motors
        }
        # Robstride publishers
        self._operation_pubs       = {}
        self._debug_operation_pubs = {}

        for src_motor, tgt_motor in self._motor_map.items():
            self._operation_pubs[src_motor] = self.create_publisher(
                OperationCommand,
                f'{tgt_prefix}/motors/{tgt_motor}/cmd_operation',
                self._qos,
            )
            self._debug_operation_pubs[src_motor] = self.create_publisher(
                OperationCommand,
                f'~/mimic_wd/debug/motors/{tgt_motor}/cmd_operation',
                self._qos,
            )

        # Damiao publishers
        self._damiao_pv_pubs          = {}
        self._damiao_debug_pv_pubs    = {}
        self._damiao_mit_pubs         = {}
        self._damiao_debug_mit_pubs   = {}

        for src_motor, tgt_motor in self._damiao_motor_map.items():
            self._damiao_pv_pubs[src_motor] = self.create_publisher(
                PositionPPCommand,
                f'{damiao_tgt_prefix}/motors/{tgt_motor}/cmd_position_pv',
                self._qos,
            )
            self._damiao_debug_pv_pubs[src_motor] = self.create_publisher(
                PositionPPCommand,
                f'~/mimic_wd/debug/motors/{tgt_motor}/cmd_position_pv',
                self._qos,
            )
            self._damiao_mit_pubs[src_motor] = self.create_publisher(
                OperationCommand,
                f'{damiao_tgt_prefix}/motors/{tgt_motor}/cmd_mit',
                self._qos,
            )
            self._damiao_debug_mit_pubs[src_motor] = self.create_publisher(
                OperationCommand,
                f'~/mimic_wd/debug/motors/{tgt_motor}/cmd_mit',
                self._qos,
            )

        # Robstride source subscriptions
        for src_motor in self._motor_map:
            topic = f'{src_prefix}/motors/{src_motor}/state'
            self._state_subs[src_motor] = self.create_subscription(
                MotorState,
                topic,
                lambda msg, s=src_motor: self._on_motor_state(s, msg),
                self._qos,
                callback_group=self._cb_subs,
            )

        # Robstride target subscriptions
        for tgt_motor in self._motor_map.values():
            topic = f'{tgt_prefix}/motors/{tgt_motor}/state'
            self._tgt_state_subs[tgt_motor] = self.create_subscription(
                MotorState,
                topic,
                lambda msg, t=tgt_motor: self._on_target_state(t, msg),
                self._qos,
                callback_group=self._cb_subs,
            )

        # Damiao source subscriptions
        for src_motor in self._damiao_motor_map:
            topic = f'{damiao_src_prefix}/motors/{src_motor}/state'
            self._damiao_state_subs[src_motor] = self.create_subscription(
                MotorState,
                topic,
                lambda msg, s=src_motor: self._on_damiao_state(s, msg),
                self._qos,
                callback_group=self._cb_subs,
            )

        # Damiao target subscriptions
        for tgt_motor in self._damiao_motor_map.values():
            topic = f'{damiao_tgt_prefix}/motors/{tgt_motor}/state'
            self._tgt_damiao_state_subs[tgt_motor] = self.create_subscription(
                MotorState,
                topic,
                lambda msg, t=tgt_motor: self._on_target_damiao_state(t, msg),
                self._qos,
                callback_group=self._cb_subs,
            )

        # Robstride service clients
        self._active_report_src_client = self.create_client(
            SetActiveReport,
            f'{src_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._active_report_tgt_client = self.create_client(
            SetActiveReport,
            f'{tgt_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._set_run_mode_client = self.create_client(
            SetRunMode,
            f'{tgt_prefix}/set_run_mode',
            callback_group=self._cb_srvs,
        )
        self._enable_motor_client = self.create_client(
            EnableMotor,
            f'{tgt_prefix}/enable_motor',
            callback_group=self._cb_srvs,
        )

        # Damiao service clients
        self._damiao_active_report_src_client = self.create_client(
            SetActiveReport,
            f'{damiao_src_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._damiao_active_report_tgt_client = self.create_client(
            SetActiveReport,
            f'{damiao_tgt_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._damiao_set_run_mode_client = self.create_client(
            SetRunMode,
            f'{damiao_tgt_prefix}/set_run_mode',
            callback_group=self._cb_srvs,
        )
        self._damiao_enable_motor_client = self.create_client(
            EnableMotor,
            f'{damiao_tgt_prefix}/enable_motor',
            callback_group=self._cb_srvs,
        )

        self.get_logger().info(
            f'Direction: {src_prefix} → {tgt_prefix} '
            f'({len(self._motor_map)} robstride + {len(self._damiao_motor_map)} damiao motors, mode={self._mode})'
        )

        # Schedule deferred setup (active report + run mode)
        self._setup_timer = self.create_timer(
            1.0, self._setup_once, callback_group=self._cb_setup
        )

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
        
    def _compute_joint_dynamics(self, model, data, q, v):
        return pin.nonLinearEffects(model, data, q, v)

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

    def _solve_arm_fk(self, model, data, q, ee_id):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[ee_id]

    def _mirror_pose(self, pose: pin.SE3) -> pin.SE3:
        """Reflect an SE3 pose through the XZ plane (negate Y) for bilateral arm symmetry."""
        M = np.diag([1.0, -1.0, 1.0])
        return pin.SE3(M @ pose.rotation @ M, M @ pose.translation)

    def _publish_ee_visualization(self) -> None:
        if not self._visualize_rviz or self._src_pin_model is None or self._tgt_pin_model is None:
            return
        q_src, _ = self._build_arm_q_v(
            self._state_snapshot['source'], self._src_sfx, self._src_pin_model, self._src_joint_map,
        )
        q_tgt, _ = self._build_arm_q_v(
            self._state_snapshot['target'], self._tgt_sfx, self._tgt_pin_model, self._tgt_joint_map,
        )
        src_pose = self._solve_arm_fk(self._src_pin_model, self._src_pin_data, q_src, self._src_ee_id)
        tgt_pose = self._solve_arm_fk(self._tgt_pin_model, self._tgt_pin_data, q_tgt, self._tgt_ee_id)
        self._publish_ee_markers(src_pose, tgt_pose)

    def _publish_ee_markers(self, src_pose: pin.SE3, tgt_pose: pin.SE3) -> None:
        markers  = MarkerArray()
        stamp    = self.get_clock().now().to_msg()
        axis_rgb = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # X=red Y=green Z=blue
        length   = 0.1  # metres

        for ns, pose in (('source_ee', src_pose), ('target_ee', tgt_pose)):
            t = pose.translation
            for i, (r, g, b) in enumerate(axis_rgb):
                tip = t + pose.rotation[:, i] * length
                m              = Marker()
                m.header.frame_id = self._base_frame
                m.header.stamp    = stamp
                m.ns              = ns
                m.id              = i
                m.type            = Marker.ARROW
                m.action          = Marker.ADD
                m.points          = [
                    Point(x=float(t[0]),   y=float(t[1]),   z=float(t[2])),
                    Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])),
                ]
                m.scale.x = 0.008   # shaft diameter
                m.scale.y = 0.016   # head diameter
                m.scale.z = 0.0
                m.color.r = r
                m.color.g = g
                m.color.b = b
                m.color.a = 1.0
                markers.markers.append(m)

        self._ee_marker_pub.publish(markers)

    def _solve_arm_ik(self, model, data, q_init: np.ndarray, target_pose: pin.SE3, ee_id: int,
                      max_iter: int = 100, eps: float = 1e-4, damping: float = 1e-4) -> np.ndarray:
        """Damped least-squares IK; returns joint configuration reaching target_pose."""
        q = q_init.copy()
        for _ in range(max_iter):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            err = pin.log6(data.oMf[ee_id].inverse() * target_pose).vector
            if np.linalg.norm(err) < eps:
                break
            J   = pin.getFrameJacobian(model, data, ee_id, pin.LOCAL)
            dq  = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), err)
            q   = pin.integrate(model, q, dq)
        return q

    # ── Startup ───────────────────────────────────────────────────────────────

    def _setup_once(self) -> None:
        self._setup_timer.cancel()
        self._setup_timer = None

        self._blocking_set_active_report(self._active_report_src_client, self._source_prefix, enable=True)
        self._blocking_set_active_report(self._active_report_tgt_client, self._target_prefix, enable=True)
        self._blocking_set_run_mode(self._mode)
        for tgt_motor in self._motor_map.values():
            self._blocking_enable_motors(self._enable_motor_client, self._target_prefix, enable=True, name=tgt_motor)

        if self._damiao_motors:
            self._blocking_set_active_report(self._damiao_active_report_src_client, self._damiao_source_prefix, enable=True)
            self._blocking_set_active_report(self._damiao_active_report_tgt_client, self._damiao_target_prefix, enable=True)
            self._blocking_damiao_set_run_mode()
            for tgt_motor in self._damiao_motor_map.values():
                self._blocking_enable_motors(self._damiao_enable_motor_client, self._damiao_target_prefix, enable=True, name=tgt_motor)

        self._ready = True
        self.get_logger().info('Setup complete — forwarding commands')

    def _blocking_set_active_report(self, client, prefix: str, enable: bool) -> None:
        action = 'enable' if enable else 'disable'
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{prefix}/set_active_report not available')
            return
        req        = SetActiveReport.Request()
        req.name   = 'all'
        req.enable = enable
        req.hz     = self._active_report_hz
        res = client.call(req)
        if res.success:
            self.get_logger().info(f'[{prefix}] Active reporting {action}d: {res.message}')
        else:
            self.get_logger().error(f'[{prefix}] Active reporting {action}d: {res.message}')

    def _blocking_set_run_mode(self, mode: str) -> None:
        if not self._set_run_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._target_prefix}/set_run_mode not available')
            return
        req                          = SetRunMode.Request()
        req.name                     = 'all'
        req.mode                     = _MODE_RUN_MODE_INT[mode]
        req.automatic_enable_disable = False
        res = self._set_run_mode_client.call(req)
        if res.success:
            self.get_logger().info(f'Target motors run mode set to {mode.upper()}: {res.message}')
        else:
            self.get_logger().error(f'Target motors run mode set to {mode.upper()}: {res.message}')

    def _blocking_enable_motors(self, client, prefix: str, enable: bool, name: str = 'all') -> None:
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{prefix}/enable_motor not available')
            return
        req             = EnableMotor.Request()
        req.name        = name
        req.enable      = enable
        req.clear_fault = False
        res    = client.call(req)
        action = 'enabled' if enable else 'disabled'
        if res.success:
            self.get_logger().info(f'[{prefix}] {name} {action}: {res.message}')
        else:
            self.get_logger().error(f'[{prefix}] {name} {action}: {res.message}')

    def _blocking_damiao_set_run_mode(self) -> None:
        if not self._damiao_set_run_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._damiao_target_prefix}/set_run_mode not available')
            return
        req      = SetRunMode.Request()
        req.name = 'all'
        req.mode = _DAMIAO_MODE_INT.get(self._damiao_mode, 2)
        req.automatic_enable_disable = False
        res = self._damiao_set_run_mode_client.call(req)
        if res.success:
            self.get_logger().info(f'[{self._damiao_target_prefix}] Run mode set to {self._damiao_mode.upper()}')
        else:
            self.get_logger().error(f'[{self._damiao_target_prefix}] set_run_mode failed: {res.message}')

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_switch_target(self, req: SetMimicTarget.Request, res: SetMimicTarget.Response):
        target = req.target.lower()
        if target not in _VALID_TARGETS:
            res.success = False
            res.message = f'Invalid target {req.target!r}. Valid: {sorted(_VALID_TARGETS)}'
            return res

        if target == self._target_node:
            res.success = True
            res.message = f'Already targeting {target}'
            return res

        if not self._switch_lock.acquire(blocking=False):
            res.success = False
            res.message = 'Switch already in progress'
            return res

        try:
            self._setup_direction(target)
            res.success = True
            res.message = f'Switched target to {target}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        finally:
            self._switch_lock.release()

        return res

    def _srv_set_debug(self, req: SetBool.Request, res: SetBool.Response):
        if req.data == self._debug:
            res.success = True
            res.message = f'Debug already {"on" if self._debug else "off"}'
            return res

        self._debug = req.data
        res.message = (
            'Debug enabled — commands routed to ~/mimic_wd/debug/… topics only'
            if self._debug else
            'Debug disabled — commands forwarded to motors'
        )
        res.success = True
        self.get_logger().info(res.message)
        return res

    def _srv_set_mode(self, req: SetMimicMode.Request, res: SetMimicMode.Response):
        mode = req.mode.lower()
        if mode not in _VALID_MODES:
            res.success = False
            res.message = f'Unknown mode {req.mode!r}. Valid: {sorted(_VALID_MODES)}'
            return res

        self._ready = False
        self._mode  = mode
        self._blocking_set_run_mode(mode)
        for tgt_motor in self._motor_map.values():
            self._blocking_enable_motors(self._enable_motor_client, self._target_prefix, enable=True, name=tgt_motor)
        self._ready = True

        res.success = True
        res.message = f'Mode changed to {mode}'
        return res

    def _srv_enable_motors(self, req: EnableMimicMotors.Request, res: EnableMimicMotors.Response):
        if not self._enable_motor_client.wait_for_service(timeout_sec=2.0):
            res.success = False
            res.message = f'{self._target_prefix}/enable_motor not available'
            return res

        names = list(req.names)
        if not names:
            res.success = False
            res.message = 'names list is empty — pass motor names or ["all"]'
            return res

        if names == ['all']:
            self._send_enable(names[0], req.enable, req.clear_fault)
        else:
            for name in names:
                self._send_enable(name, req.enable, req.clear_fault)

        action = 'enabled' if req.enable else 'disabled'
        res.success = True
        res.message = f'{action}: {names}'
        return res

    def _send_enable(self, name: str, enable: bool, clear_fault: bool) -> None:
        fwd             = EnableMotor.Request()
        fwd.name        = name
        fwd.enable      = enable
        fwd.clear_fault = clear_fault
        future = self._enable_motor_client.call_async(fwd)
        future.add_done_callback(
            lambda f, n=name, e=enable: self.get_logger().info(
                f'[{n}] {"enabled" if e else "disabled"}: {f.result().message}'
            ) if f.result() else self.get_logger().error(f'[{n}] enable_motor call failed')
        )

    def _srv_set_params(self, req: SetMimicParams.Request, res: SetMimicParams.Response):
        mode = req.mode.lower()

        if mode == 'operation':
            # speed field → velocity default; torque_limit field → torque_ff default
            if req.speed        != 0.0: self._robstride_operation_defaults['velocity']  = req.speed
            if req.torque_limit != 0.0: self._robstride_operation_defaults['torque_ff'] = req.torque_limit
            res.success = True
            res.message = (
                f'Operation defaults — velocity={self._robstride_operation_defaults["velocity"]} '
                f'torque_ff={self._robstride_operation_defaults["torque_ff"]}'
            )

        elif mode == 'position_velocity':
            if req.speed > 0.0: self._damiao_pv_defaults['speed'] = req.speed
            res.success = True
            res.message = f'Damiao PV defaults — speed={self._damiao_pv_defaults["speed"]}'

        elif mode == 'mit':
            # speed field → velocity default; torque_limit field → torque_ff default
            if req.speed        != 0.0: self._damiao_mit_defaults['velocity']  = req.speed
            if req.torque_limit != 0.0: self._damiao_mit_defaults['torque_ff'] = req.torque_limit
            res.success = True
            res.message = (
                f'Damiao MIT defaults — velocity={self._damiao_mit_defaults["velocity"]} '
                f'torque_ff={self._damiao_mit_defaults["torque_ff"]}'
            )

        else:
            res.success = False
            res.message = f'Unknown mode {req.mode!r}. Valid: operation, position_velocity, mit'

        return res

    # ── Motor state callback ──────────────────────────────────────────────────

    def _on_motor_state(self, source: str, msg: MotorState) -> None:
        if not self._ready:
            return
        self._latest_src_state[source] = msg

    def _on_target_state(self, target: str, msg: MotorState) -> None:
        if not self._ready:
            return
        self._latest_tgt_state[target] = msg

    def _on_ik_timer(self) -> None:
        if not self._ready:
            return

        self._state_snapshot = {
            'source': {**self._latest_src_state, **self._latest_src_damiao_state},
            'target': {**self._latest_tgt_state, **self._latest_tgt_damiao_state},
        }

        if self._use_ik:
            self._publish_ik_commands()
        else:
            self._publish_direct_commands()

        self._publish_damiao_commands()
        self._publish_ee_visualization()

    def _publish_direct_commands(self) -> None:
        """Direct joint mirroring: apply transform, compute NLE at target state, publish."""
        tau_nle = None
        if self._tgt_pin_model is None:
            self.get_logger().warning('Pinocchio model not loaded — torque_ff falling back to config default')
        elif not self._tgt_joint_map:
            self.get_logger().warning('Target joint map is empty — torque_ff falling back to config default')
        else:
            q_tgt, v_tgt = self._build_arm_q_v(
                self._state_snapshot['target'], self._tgt_sfx,
                self._tgt_pin_model, self._tgt_joint_map,
            )
            tau_nle = self._compute_joint_dynamics(self._tgt_pin_model, self._tgt_pin_data, q_tgt, v_tgt)

        for source, state in self._state_snapshot['source'].items():
            if source in self._motor_map:
                target   = self._motor_map[source]
                tf       = self._transforms[source]
                position = tf(state.position)
                velocity = tf(state.velocity)

                base = source[:-1]
                jid  = self._tgt_joint_map.get(base)
                if tau_nle is not None and jid is not None:
                    torque_ff = float(tau_nle[self._tgt_pin_model.joints[jid].idx_v])
                else:
                    if tau_nle is not None and jid is None:
                        self.get_logger().warning(f'{source}: no joint mapping — torque_ff falling back to config default')
                    torque_ff = self._robstride_operation_defaults['torque_ff']

                cmd           = OperationCommand()
                cmd.name      = target
                cmd.position  = position
                cmd.velocity  = velocity
                cmd.torque_ff = torque_ff
                pub = self._debug_operation_pubs[source] if self._debug else self._operation_pubs[source]
                pub.publish(cmd)

    def _publish_ik_commands(self) -> None:
        """FK on source → mirror pose → IK on target → NLE at IK solution → publish."""
        if self._src_pin_model is None or self._tgt_pin_model is None:
            self.get_logger().warning('Pinocchio models not loaded — skipping IK publish')
            return
        if not self._src_joint_map or not self._tgt_joint_map:
            self.get_logger().warning('Joint maps not ready — skipping IK publish')
            return

        # Source FK
        q_src, _ = self._build_arm_q_v(
            self._state_snapshot['source'], self._src_sfx,
            self._src_pin_model, self._src_joint_map,
        )
        src_ee_pose = self._solve_arm_fk(self._src_pin_model, self._src_pin_data, q_src, self._src_ee_id)

        # Mirror EE pose around base_frame symmetry plane
        mirrored_pose = self._mirror_pose(src_ee_pose)

        # IK on target arm, warm-started from current target joint state
        q_tgt_init, _ = self._build_arm_q_v(
            self._state_snapshot['target'], self._tgt_sfx,
            self._tgt_pin_model, self._tgt_joint_map,
        )
        q_tgt_ik = self._solve_arm_ik(
            self._tgt_pin_model, self._tgt_pin_data, q_tgt_init, mirrored_pose, self._tgt_ee_id,
        )

        # NLE at IK solution (zero velocity → gravity compensation only)
        tau_nle = self._compute_joint_dynamics(
            self._tgt_pin_model, self._tgt_pin_data,
            q_tgt_ik, np.zeros(self._tgt_pin_model.nv),
        )

        # Publish one OperationCommand per robstride motor
        for src_motor, tgt_motor in self._motor_map.items():
            base = src_motor[:-1]
            jid  = self._tgt_joint_map.get(base)
            if jid is None:
                self.get_logger().warning(f'{src_motor}: no IK joint mapping — skipping')
                continue

            cmd           = OperationCommand()
            cmd.name      = tgt_motor
            cmd.position  = float(q_tgt_ik[self._tgt_pin_model.joints[jid].idx_q])
            cmd.velocity  = 0.0
            cmd.torque_ff = float(tau_nle[self._tgt_pin_model.joints[jid].idx_v])
            pub = self._debug_operation_pubs[src_motor] if self._debug else self._operation_pubs[src_motor]
            pub.publish(cmd)

    def _publish_damiao_commands(self) -> None:
        for source, state in self._state_snapshot['source'].items():
            if source not in self._damiao_motor_map:
                continue
            target   = self._damiao_motor_map[source]
            tf       = self._damiao_transforms[source]
            position = tf(state.position)
            velocity = tf(state.velocity)

            if self._damiao_mode == 'mit':
                cmd           = OperationCommand()
                cmd.name      = target
                cmd.position  = position
                cmd.velocity  = velocity
                cmd.torque_ff = self._damiao_mit_defaults['torque_ff']
                pub = self._damiao_debug_mit_pubs[source] if self._debug else self._damiao_mit_pubs[source]
            else:
                cmd          = PositionPPCommand()
                cmd.name     = target
                cmd.position = position
                cmd.speed    = abs(velocity)
                pub = self._damiao_debug_pv_pubs[source] if self._debug else self._damiao_pv_pubs[source]
            pub.publish(cmd)

    def _on_damiao_state(self, source: str, msg: MotorState) -> None:
        if not self._ready:
            return
        self._latest_src_damiao_state[source] = msg

    def _on_target_damiao_state(self, target: str, msg: MotorState) -> None:
        if not self._ready:
            return
        self._latest_tgt_damiao_state[target] = msg

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown_cleanup(self) -> None:
        print('[mimic_wd_node] Shutting down — disabling active reporting and target motors …')

        def _call(client, req, label):
            if not client or not client.service_is_ready():
                print(f'[mimic_wd_node] {label}: service not ready — skipping')
                return
            done   = threading.Event()
            future = client.call_async(req)
            future.add_done_callback(lambda _: done.set())
            done.wait(timeout=2.0)
            if future.done() and future.result() is not None:
                print(f'[mimic_wd_node] {label}: {future.result().message}')
            else:
                print(f'[mimic_wd_node] {label}: timed out')

        ar_off = self._make_active_report_req(False)
        _call(self._active_report_src_client,        ar_off,                         'robstride source active reporting disabled')
        _call(self._active_report_tgt_client,        ar_off,                         'robstride target active reporting disabled')
        _call(self._enable_motor_client,             self._make_disable_motors_req(), 'robstride target motors disabled')
        _call(self._damiao_active_report_src_client, ar_off,                         'damiao source active reporting disabled')
        _call(self._damiao_active_report_tgt_client, ar_off,                         'damiao target active reporting disabled')
        _call(self._damiao_enable_motor_client,      self._make_disable_motors_req(), 'damiao target motors disabled')

    def _make_active_report_req(self, enable: bool) -> SetActiveReport.Request:
        req        = SetActiveReport.Request()
        req.name   = 'all'
        req.enable = enable
        req.hz     = self._active_report_hz if enable else 0.0
        return req

    def _make_disable_motors_req(self) -> EnableMotor.Request:
        req             = EnableMotor.Request()
        req.name        = 'all'
        req.enable      = False
        req.clear_fault = False
        return req


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    import signal

    rclpy.init(args=args)
    node     = MimicWDNode()
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
