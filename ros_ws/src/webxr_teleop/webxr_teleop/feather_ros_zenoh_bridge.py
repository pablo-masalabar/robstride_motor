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
from typing import Dict, List, Optional

import zenoh
from feather import feather_pb2

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.msg import MotorState, PositionCSPCommand, PositionPPCommand
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from custom_interfaces.srv import EnableMotor, EnableMotors, Freeze, SetActiveReport, SetRunMode


_ROBSTRIDE_MODE_INT: Dict[str, int] = {
    'pp':  1,
    'csp': 5,
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

_CMD_TOPIC_MSG_TYPE = {
    'cmd_position_pp':  PositionPPCommand,
    'cmd_position_csp': PositionCSPCommand,
    'cmd_position_pv':  PositionPPCommand,
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

        self._debug: bool = bool(cfg.get('debug', False))
        if self._debug:
            self.get_logger().warning('DEBUG mode enabled — commands will be logged but NOT published')

        self._global_robstride_hz: Optional[float] = (
            float(cfg['robstride_active_report_hz']) if 'robstride_active_report_hz' in cfg else None
        )
        self._global_damiao_hz: Optional[float] = (
            float(cfg['damiao_active_report_hz']) if 'damiao_active_report_hz' in cfg else None
        )
        self._global_ezmotion_hz: Optional[float] = (
            float(cfg['ezmotion_active_report_hz']) if 'ezmotion_active_report_hz' in cfg else None
        )

        self._robstride_pp_defaults = self._require_section(cfg, 'robstride_pp_defaults',
            ['speed', 'acceleration', 'deceleration', 'torque_limit'])
        self._ezmotion_pp_defaults = self._require_section(cfg, 'ezmotion_pp_defaults',
            ['speed', 'acceleration', 'deceleration'])
        self._damiao_pv_defaults = self._require_section(cfg, 'damiao_pv_defaults',
            ['speed'])

        for field in ('gripper_expanded_val', 'gripper_contracted_val'):
            if field not in cfg:
                self.get_logger().fatal(f'Missing required config field "{field}"')
                self._shutdown()
                return
        self._gripper_expanded_val:   float = float(cfg['gripper_expanded_val'])
        self._gripper_contracted_val: float = float(cfg['gripper_contracted_val'])

        self._qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._latest_states: Dict[str, MotorState] = {}

        self._nodes: Dict[str, dict] = self._parse_nodes(cfg)

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

        self._forwarding_enabled: bool = False
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

        self._joints_state_publish_hz: float = self._resolve_publish_hz(cfg)
        self._zenoh_session = self._open_zenoh_session(cfg.get('zenoh_config'))

        self._zenoh_groups: Dict[str, dict] = self._parse_zenoh_groups(cfg)

        self._zenoh_cmd_subs: Dict[str, object] = {}
        for group_name, group in self._zenoh_groups.items():
            cmd_key = group['cmd_key']
            if cmd_key:
                sub = self._zenoh_session.declare_subscriber(
                    cmd_key,
                    lambda sample, g=group_name: self._on_zenoh_cmd(sample, g),
                )
                self._zenoh_cmd_subs[group_name] = sub
                self.get_logger().info(
                    f'Zenoh cmd sub [{group_name}]: {cmd_key}'
                )

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

            nodes[section_key] = {
                'prefix':           prefix,
                'active_report_hz': report_hz,
                'enable_service':   enable_svc,
                'motors': {
                    motor_name: {
                        'cmd_topic':      motor_val['cmd_topic_name'],
                        'feedback_topic': motor_val['feedback_topic_name'],
                        'motor_type':     motor_val['motor_type'].lower(),
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

    def _open_zenoh_session(self, config_path: Optional[str]):
        if config_path and os.path.isfile(config_path):
            session = zenoh.open(zenoh.Config.from_file(config_path))
            self.get_logger().info(f'Zenoh session opened (config: {config_path})')
        else:
            session = zenoh.open(zenoh.Config())
            self.get_logger().info('Zenoh session opened (default config)')
        return session

    def _parse_zenoh_groups(self, cfg: dict) -> Dict[str, dict]:
        raw = cfg.get('zenoh_motor_groups', {})
        groups: Dict[str, dict] = {}

        motor_cmd_topic: Dict[str, str] = {}
        for node in self._nodes.values():
            for motor_name, motor in node['motors'].items():
                motor_cmd_topic[motor_name] = motor['cmd_topic']

        for group_name, motors in raw.items():
            feedback_key = cfg.get(f'{group_name}_zenoh_key')
            cmd_key      = cfg.get(f'{group_name}_cmd_zenoh_key')
            if not feedback_key:
                self.get_logger().warning(
                    f'[zenoh_motor_groups] group "{group_name}" has no matching '
                    f'"{group_name}_zenoh_key" in config — feedback skipped'
                )
            groups[group_name] = {
                'motors':          list(motors),
                'feedback_key':    feedback_key,
                'cmd_key':         cmd_key,
                'motor_cmd_topic': {m: motor_cmd_topic[m] for m in motors if m in motor_cmd_topic},
            }

        return groups

    def _gripper_to_zenoh(self, motor_pos: float) -> float:
        span = self._gripper_expanded_val - self._gripper_contracted_val
        if span == 0.0:
            return 0.0
        return max(0.0, min(1.0, (motor_pos - self._gripper_contracted_val) / span))

    def _zenoh_to_gripper(self, cmd: float) -> float:
        cmd = max(0.0, min(1.0, cmd))
        return self._gripper_contracted_val + cmd * (self._gripper_expanded_val - self._gripper_contracted_val)

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
                if motor['motor_type'] == 'robstride':
                    mode_int = _ROBSTRIDE_MODE_INT[self._robstride_motor_mode]
                elif motor['motor_type'] == 'ezmotion':
                    mode_int = _EZMOTION_MODE_INT[self._ezmotion_motor_mode]
                else:
                    mode_int = _DAMIAO_MODE_INT[self._damiao_motor_mode]

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
        if not name:
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

    # ── Zenoh feedback (ROS → Zenoh) ──────────────────────────────────────────

    def _publish_zenoh_feedback(self) -> None:
        for group_name, group in self._zenoh_groups.items():
            feedback_key = group['feedback_key']
            if not feedback_key:
                continue

            is_gripper = 'gripper' in group_name
            array = feather_pb2.JointStatesArray()
            for motor_name in group['motors']:
                state = self._latest_states.get(motor_name)
                if state is None:
                    continue
                joint          = array.joints.add()
                joint.id       = motor_name
                joint.position = (
                    self._gripper_to_zenoh(float(state.position))
                    if is_gripper else float(state.position)
                )
                joint.velocity = float(state.velocity)
                joint.torque   = float(state.torque)

            if array.joints:
                self._zenoh_session.put(feedback_key, array.SerializeToString())

    # ── Zenoh command (Zenoh → ROS) ───────────────────────────────────────────

    def _on_zenoh_cmd(self, sample, group_name: str) -> None:
        if not self._forwarding_enabled:
            return

        group = self._zenoh_groups.get(group_name)
        if group is None:
            return

        array = feather_pb2.JointControlArray()
        try:
            array.ParseFromString(sample.payload.to_bytes())
        except Exception as e:
            self.get_logger().error(f'[{group_name}] bad JointControlArray: {e}')
            return

        positions_by_id: Dict[str, float] = {
            j.id: float(j.command)
            for j in array.joints
            if j.cmd_type == feather_pb2.JointControl.POSITION
        }

        is_gripper = 'gripper' in group_name

        for motor_name in group['motors']:
            if motor_name not in positions_by_id:
                continue
            if self._is_frozen(motor_name):
                continue

            cmd_topic = group['motor_cmd_topic'].get(motor_name)
            if not cmd_topic or cmd_topic not in self._cmd_pubs:
                continue

            position = positions_by_id[motor_name]
            if is_gripper:
                position = self._zenoh_to_gripper(position)

            self._publish_motor_cmd(motor_name, cmd_topic, position)

    def _publish_motor_cmd(self, motor_name: str, cmd_topic: str, position: float) -> None:
        pub = self._cmd_pubs[cmd_topic]
        if cmd_topic.endswith('go_to'):
            cmd      = Float64()
            cmd.data = position
        elif cmd_topic.endswith('cmd_position_csp'):
            cmd               = PositionCSPCommand()
            cmd.name          = motor_name
            cmd.position      = position
            cmd.speed_limit   = self._robstride_pp_defaults['speed']
            cmd.current_limit = 0.0
        elif cmd_topic.endswith('cmd_position_pv'):
            cmd          = PositionPPCommand()
            cmd.name     = motor_name
            cmd.position = position
            cmd.speed    = self._damiao_pv_defaults['speed']
        else:
            cmd              = PositionPPCommand()
            cmd.name         = motor_name
            cmd.position     = position
            cmd.speed        = self._robstride_pp_defaults['speed']
            cmd.acceleration = self._robstride_pp_defaults['acceleration']
            cmd.deceleration = self._robstride_pp_defaults['deceleration']
            cmd.torque_limit = self._robstride_pp_defaults['torque_limit']
        if self._debug:
            self.get_logger().info(f'[DEBUG] {cmd_topic} → {motor_name} pos={position:.4f}')
        else:
            pub.publish(cmd)

    # ── Motor state callback ──────────────────────────────────────────────────

    def _on_motor_state(self, motor_name: str, msg: MotorState) -> None:
        self._latest_states[motor_name] = msg

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
                print(f'[webxr_teleop] zenoh undeclare [{group_name}]: {e}')
        if hasattr(self, '_zenoh_cmd_subs'):
            self._zenoh_cmd_subs.clear()
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
