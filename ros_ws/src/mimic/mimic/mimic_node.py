import threading
import tomllib
from typing import Callable, Dict

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.msg import MotorState, PositionCSPCommand, PositionPPCommand
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


_VALID_MODES   = frozenset({'csp', 'pp'})
_VALID_TARGETS = frozenset({'left_arm', 'right_arm'})

_MODE_RUN_MODE_INT = {
    'csp': 5,  # POSITION_CSP
    'pp':  1,  # POSITION_PP
}


class MimicNode(Node):

    def __init__(self):
        super().__init__('mimic_node')

        self._cb_subs  = MutuallyExclusiveCallbackGroup()
        self._cb_srvs  = ReentrantCallbackGroup()
        self._cb_setup = MutuallyExclusiveCallbackGroup()
        self._switch_lock = threading.Lock()

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        cfg = self._load_config(config_path)

        self._left_arm_prefix:  str = cfg['left_arm_node_prefix']
        self._right_arm_prefix: str = cfg['right_arm_node_prefix']
        self._motors:           list = cfg['motors']   # base names, e.g. ["Sp", "Sr", ...]

        self._forward_transforms: Dict[str, Callable] = self._load_transform_map(
            cfg.get('transform_map', {})
        )
        self._inverse_transforms: Dict[str, Callable] = self._load_transform_map(
            cfg.get('inverse_transform_map', {})
        )

        self._debug:            bool  = bool(cfg.get('debug', False))
        self._active_report_hz: float = float(cfg.get('active_report_hz', 30.0))
        self._op_hz:            float = float(cfg.get('op_hz', 0.0))
        self._mode:             str   = cfg.get('mode', 'csp').lower()

        self._pp_defaults: Dict[str, float] = {
            'speed':        float(cfg.get('pp_defaults',  {}).get('speed',        0.0)),
            'acceleration': float(cfg.get('pp_defaults',  {}).get('acceleration', 0.0)),
            'deceleration': float(cfg.get('pp_defaults',  {}).get('deceleration', 0.0)),
            'torque_limit': float(cfg.get('pp_defaults',  {}).get('torque_limit', 0.0)),
        }
        self._csp_defaults: Dict[str, float] = {
            'speed_limit':   float(cfg.get('csp_defaults', {}).get('speed_limit',   0.0)),
            'current_limit': float(cfg.get('csp_defaults', {}).get('current_limit', 0.0)),
        }

        if self._mode not in _VALID_MODES:
            raise RuntimeError(f'Invalid mode {self._mode!r}. Valid: {sorted(_VALID_MODES)}')

        self._qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Mutable direction state — protected by _switch_lock during switches
        self._target_node:    str            = ''
        self._source_prefix:  str            = ''
        self._target_prefix:  str            = ''
        self._motor_map:      Dict[str, str] = {}
        self._transforms:     Dict[str, Callable] = {}
        self._latest_pos:     Dict[str, float] = {}
        self._ready:          bool           = False

        # Placeholders filled by _setup_direction
        self._state_subs:       Dict[str, object] = {}
        self._csp_pubs:         Dict[str, object] = {}
        self._pp_pubs:          Dict[str, object] = {}
        self._debug_csp_pubs:   Dict[str, object] = {}
        self._debug_pp_pubs:    Dict[str, object] = {}
        self._active_report_src_client  = None
        self._active_report_tgt_client  = None
        self._set_run_mode_client       = None
        self._enable_motor_client       = None

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

        if self._op_hz > 0.0:
            self.create_timer(
                1.0 / self._op_hz, self._publish_latest, callback_group=self._cb_subs
            )

        initial_target = cfg.get('target_node', 'right_arm').lower()
        if initial_target not in _VALID_TARGETS:
            raise RuntimeError(f'Invalid target_node {initial_target!r}. Valid: {sorted(_VALID_TARGETS)}')

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

        # Disable the current target motors before destroying the client
        if not self._debug and self._enable_motor_client is not None and self._enable_motor_client.service_is_ready():
            self._blocking_enable_motors(enable=False)

        # Tear down previous service clients
        for client in [
            self._active_report_src_client,
            self._active_report_tgt_client,
            self._set_run_mode_client,
            self._enable_motor_client,
        ]:
            if client is not None:
                self.destroy_client(client)

        self._target_node = target_node

        if target_node == 'right_arm':
            src_prefix, tgt_prefix, src_sfx, tgt_sfx = (
                self._left_arm_prefix, self._right_arm_prefix, 'L', 'R'
            )
            active_transforms = self._forward_transforms
        else:
            src_prefix, tgt_prefix, src_sfx, tgt_sfx = (
                self._right_arm_prefix, self._left_arm_prefix, 'R', 'L'
            )
            active_transforms = self._inverse_transforms

        self._source_prefix = src_prefix
        self._target_prefix = tgt_prefix
        self._motor_map     = {b + src_sfx: b + tgt_sfx for b in self._motors}
        self._transforms    = {
            b + src_sfx: active_transforms.get(b, _transforms.passthrough)
            for b in self._motors
        }
        self._latest_pos.clear()

        # Publishers (always recreate so topics match new direction)
        self._csp_pubs       = {}
        self._pp_pubs        = {}
        self._debug_csp_pubs = {}
        self._debug_pp_pubs  = {}

        for src_motor, tgt_motor in self._motor_map.items():
            self._csp_pubs[src_motor] = self.create_publisher(
                PositionCSPCommand,
                f'{tgt_prefix}/motors/{tgt_motor}/cmd_position_csp',
                self._qos,
            )
            self._pp_pubs[src_motor] = self.create_publisher(
                PositionPPCommand,
                f'{tgt_prefix}/motors/{tgt_motor}/cmd_position_pp',
                self._qos,
            )
            self._debug_csp_pubs[src_motor] = self.create_publisher(
                PositionCSPCommand,
                f'~/mimic/debug/motors/{tgt_motor}/cmd_position_csp',
                self._qos,
            )
            self._debug_pp_pubs[src_motor] = self.create_publisher(
                PositionPPCommand,
                f'~/mimic/debug/motors/{tgt_motor}/cmd_position_pp',
                self._qos,
            )

        # Subscriptions
        for src_motor in self._motor_map:
            topic = f'{src_prefix}/motors/{src_motor}/state'
            self._state_subs[src_motor] = self.create_subscription(
                MotorState,
                topic,
                lambda msg, s=src_motor: self._on_motor_state(s, msg),
                self._qos,
                callback_group=self._cb_subs,
            )

        # Service clients
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

        self.get_logger().info(
            f'Direction: {src_prefix} → {tgt_prefix} '
            f'({len(self._motor_map)} motors, mode={self._mode})'
        )

        # Schedule deferred setup (active report + run mode)
        self._setup_timer = self.create_timer(
            1.0, self._setup_once, callback_group=self._cb_setup
        )

    # ── Startup ───────────────────────────────────────────────────────────────

    def _setup_once(self) -> None:
        self._setup_timer.cancel()
        self._setup_timer = None

        self._blocking_set_active_report(self._active_report_src_client, self._source_prefix, enable=True)
        self._blocking_set_active_report(self._active_report_tgt_client, self._target_prefix, enable=True)
        self._blocking_set_run_mode(self._mode)
        if not self._debug:
            self._blocking_enable_motors(enable=True)

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

    def _blocking_enable_motors(self, enable: bool) -> None:
        if not self._enable_motor_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._target_prefix}/enable_motor not available')
            return
        req             = EnableMotor.Request()
        req.name        = 'all'
        req.enable      = enable
        req.clear_fault = False
        res = self._enable_motor_client.call(req)
        action = 'enabled' if enable else 'disabled'
        if res.success:
            self.get_logger().info(f'Target motors {action}: {res.message}')
        else:
            self.get_logger().error(f'Target motors {action}: {res.message}')

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

        self._ready = False
        self._debug = req.data

        if self._debug:
            # Entering debug — disable real target motors
            if self._enable_motor_client and self._enable_motor_client.service_is_ready():
                self._blocking_enable_motors(enable=False)
            res.message = 'Debug enabled — commands routed to ~/mimic/debug/… topics, target motors disabled'
        else:
            # Leaving debug — enable real target motors
            self._blocking_enable_motors(enable=True)
            res.message = 'Debug disabled — commands routed to real motors, target motors enabled'

        self._ready = True
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
        if not self._debug:
            self._blocking_enable_motors(enable=True)
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

        if mode == 'pp':
            if req.speed        > 0.0: self._pp_defaults['speed']        = req.speed
            if req.acceleration > 0.0: self._pp_defaults['acceleration'] = req.acceleration
            if req.deceleration > 0.0: self._pp_defaults['deceleration'] = req.deceleration
            if req.torque_limit > 0.0: self._pp_defaults['torque_limit'] = req.torque_limit
            res.success = True
            res.message = (
                f'PP defaults — speed={self._pp_defaults["speed"]} '
                f'accel={self._pp_defaults["acceleration"]} '
                f'decel={self._pp_defaults["deceleration"]} '
                f'torque_limit={self._pp_defaults["torque_limit"]}'
            )

        elif mode == 'csp':
            if req.speed_limit   > 0.0: self._csp_defaults['speed_limit']   = req.speed_limit
            if req.current_limit > 0.0: self._csp_defaults['current_limit'] = req.current_limit
            res.success = True
            res.message = (
                f'CSP defaults — speed_limit={self._csp_defaults["speed_limit"]} '
                f'current_limit={self._csp_defaults["current_limit"]}'
            )

        else:
            res.success = False
            res.message = f'Unknown mode {req.mode!r}. Valid: pp, csp'

        return res

    # ── Motor state callback ──────────────────────────────────────────────────

    def _on_motor_state(self, source: str, msg: MotorState) -> None:
        if not self._ready:
            return

        self._latest_pos[source] = self._transforms[source](msg.position)

        if self._op_hz <= 0.0:
            self._publish_for(source)

    def _publish_latest(self) -> None:
        if not self._ready:
            return
        for source in self._motor_map:
            if source in self._latest_pos:
                self._publish_for(source)

    def _publish_for(self, source: str) -> None:
        target   = self._motor_map[source]
        position = self._latest_pos[source]

        if self._mode == 'csp':
            cmd               = PositionCSPCommand()
            cmd.name          = target
            cmd.position      = position
            cmd.speed_limit   = self._csp_defaults['speed_limit']
            cmd.current_limit = self._csp_defaults['current_limit']
            pub = self._debug_csp_pubs[source] if self._debug else self._csp_pubs[source]
            pub.publish(cmd)

        elif self._mode == 'pp':
            cmd              = PositionPPCommand()
            cmd.name         = target
            cmd.position     = position
            cmd.speed        = self._pp_defaults['speed']
            cmd.acceleration = self._pp_defaults['acceleration']
            cmd.deceleration = self._pp_defaults['deceleration']
            cmd.torque_limit = self._pp_defaults['torque_limit']
            pub = self._debug_pp_pubs[source] if self._debug else self._pp_pubs[source]
            pub.publish(cmd)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown_cleanup(self) -> None:
        print('[mimic_node] Shutting down — disabling active reporting and target motors …')

        def _call(client, req, label):
            if not client or not client.service_is_ready():
                print(f'[mimic_node] {label}: service not ready — skipping')
                return
            done   = threading.Event()
            future = client.call_async(req)
            future.add_done_callback(lambda _: done.set())
            done.wait(timeout=2.0)
            if future.done() and future.result() is not None:
                print(f'[mimic_node] {label}: {future.result().message}')
            else:
                print(f'[mimic_node] {label}: timed out')

        ar_off = self._make_active_report_req(False)
        _call(self._active_report_src_client, ar_off, 'source active reporting disabled')
        _call(self._active_report_tgt_client, ar_off, 'target active reporting disabled')
        _call(self._enable_motor_client,      self._make_disable_motors_req(), 'target motors disabled')

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
    node     = MimicNode()
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
