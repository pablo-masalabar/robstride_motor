import tomllib
from typing import Callable, Dict

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.msg import MotorState, PositionCSPCommand, PositionPPCommand
from mimic import transforms as _transforms
from custom_interfaces.srv import (
    EnableMimicMotors,
    EnableMotor,
    SetActiveReport,
    SetMimicMode,
    SetMimicParams,
    SetRunMode,
)


_VALID_MODES = frozenset({'csp', 'pp'})

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

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        cfg = self._load_config(config_path)

        self._motor_map:        Dict[str, str]            = cfg['motor_map']
        self._transforms:       Dict[str, Callable]       = self._load_transforms(cfg)
        self._debug:            bool                      = bool(cfg.get('debug', False))
        self._ready:            bool                      = False

        self._pp_defaults:  Dict[str, float] = {
            'speed':        float(cfg.get('pp_defaults',  {}).get('speed',        0.0)),
            'acceleration': float(cfg.get('pp_defaults',  {}).get('acceleration', 0.0)),
            'deceleration': float(cfg.get('pp_defaults',  {}).get('deceleration', 0.0)),
            'torque_limit': float(cfg.get('pp_defaults',  {}).get('torque_limit', 0.0)),
        }
        self._csp_defaults: Dict[str, float] = {
            'speed_limit':   float(cfg.get('csp_defaults', {}).get('speed_limit',   0.0)),
            'current_limit': float(cfg.get('csp_defaults', {}).get('current_limit', 0.0)),
        }
        self._mode:             str            = cfg.get('mode', 'csp').lower()
        self._active_report_hz: float          = float(cfg.get('active_report_hz', 30.0))
        self._op_hz:            float          = float(cfg.get('op_hz', 0.0))
        self._source_prefix:    str            = cfg.get('source_node_prefix', '')
        self._target_prefix:    str            = cfg.get('target_node_prefix', '')

        # Latest transformed position per source motor; populated by _on_motor_state
        self._latest_pos: Dict[str, float] = {}

        if self._mode not in _VALID_MODES:
            raise RuntimeError(f'Invalid mode {self._mode!r}. Valid: {sorted(_VALID_MODES)}')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._csp_pubs:       Dict[str, object] = {}
        self._pp_pubs:        Dict[str, object] = {}
        self._debug_csp_pubs: Dict[str, object] = {}
        self._debug_pp_pubs:  Dict[str, object] = {}

        for source, target in self._motor_map.items():
            self._csp_pubs[source] = self.create_publisher(
                PositionCSPCommand,
                cfg['target_csp_topic_pattern'].format(name=target), qos,
            )
            self._pp_pubs[source] = self.create_publisher(
                PositionPPCommand,
                cfg['target_pp_topic_pattern'].format(name=target), qos,
            )
            self._debug_csp_pubs[source] = self.create_publisher(
                PositionCSPCommand,
                self._topic(f'mimic/debug/motors/{target}/cmd_position_csp'), qos,
            )
            self._debug_pp_pubs[source] = self.create_publisher(
                PositionPPCommand,
                self._topic(f'mimic/debug/motors/{target}/cmd_position_pp'), qos,
            )

        state_topic_pattern = cfg['source_motor_state_topic_pattern']
        self._state_subs: Dict[str, object] = {}
        for source in self._motor_map:
            topic = state_topic_pattern.format(name=source)
            self._state_subs[source] = self.create_subscription(
                MotorState,
                topic,
                lambda msg, s=source: self._on_motor_state(s, msg),
                qos,
                callback_group=self._cb_subs,
            )

        self._active_report_client = self.create_client(
            SetActiveReport,
            f'{self._source_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._target_active_report_client = self.create_client(
            SetActiveReport,
            f'{self._target_prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._set_run_mode_client = self.create_client(
            SetRunMode,
            f'{self._target_prefix}/set_run_mode',
            callback_group=self._cb_srvs,
        )
        self._enable_motor_client = self.create_client(
            EnableMotor,
            f'{self._target_prefix}/enable_motor',
            callback_group=self._cb_srvs,
        )

        self.create_service(
            SetMimicMode,
            self._topic('set_mode'),
            self._srv_set_mode,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            SetMimicParams,
            self._topic('set_params'),
            self._srv_set_params,
            callback_group=self._cb_srvs,
        )
        self.create_service(
            EnableMimicMotors,
            self._topic('enable_motors'),
            self._srv_enable_motors,
            callback_group=self._cb_srvs,
        )

        # Fire once after 1 s to give motor nodes time to start
        self._setup_timer = self.create_timer(
            1.0, self._setup_once, callback_group=self._cb_setup
        )

        if self._op_hz > 0.0:
            self.create_timer(
                1.0 / self._op_hz, self._publish_latest, callback_group=self._cb_subs
            )

        self.get_logger().info(
            f'MimicNode ready — {len(self._motor_map)} motor(s), '
            f'mode={self._mode}, active_report_hz={self._active_report_hz}, '
            f'op_hz={"timer@" + str(self._op_hz) if self._op_hz > 0.0 else "passthrough"}'
            + (' [DEBUG MODE — commands go to ~/mimic/debug/… topics]' if self._debug else '')
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _topic(self, suffix: str) -> str:
        """Return a node-namespaced topic / service name: ~/{suffix}."""
        return f'~/{suffix}'

    # ── Config ────────────────────────────────────────────────────────────────

    def _load_transforms(self, cfg: dict) -> Dict[str, Callable]:
        transform_map_cfg = cfg.get('transform_map', {})
        result: Dict[str, Callable] = {}
        for source in self._motor_map:
            fn_name = transform_map_cfg.get(source)
            if fn_name is None:
                result[source] = _transforms.passthrough
            else:
                fn = getattr(_transforms, fn_name, None)
                if fn is None:
                    self.get_logger().warning(
                        f'[{source}] Transform function {fn_name!r} not found in transforms.py '
                        f'— using passthrough'
                    )
                    result[source] = _transforms.passthrough
                else:
                    result[source] = fn
                    self.get_logger().info(f'[{source}] Transform: {fn_name}')
        return result

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

        self._blocking_set_active_report(self._active_report_client,       self._source_prefix, enable=True)
        self._blocking_set_active_report(self._target_active_report_client, self._target_prefix, enable=True)
        self._blocking_set_run_mode(self._mode)

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
        log = self.get_logger().info if res.success else self.get_logger().error
        log(f'[{prefix}] Active reporting {action}d: {res.message}')

    def _blocking_set_run_mode(self, mode: str) -> None:
        if not self._set_run_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._target_prefix}/set_run_mode not available')
            return
        req                          = SetRunMode.Request()
        req.name                     = 'all'
        req.mode                     = _MODE_RUN_MODE_INT[mode]
        req.automatic_enable_disable = True
        res = self._set_run_mode_client.call(req)
        log = self.get_logger().info if res.success else self.get_logger().error
        log(f'Target motors set to {mode.upper()}: {res.message}')

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_set_mode(self, req: SetMimicMode.Request, res: SetMimicMode.Response):
        mode = req.mode.lower()
        if mode not in _VALID_MODES:
            res.success = False
            res.message = f'Unknown mode {req.mode!r}. Valid: {sorted(_VALID_MODES)}'
            return res

        self._ready = False
        self._mode  = mode
        self._blocking_set_run_mode(mode)
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

        # 'all' as a single entry broadcasts to every motor in one call
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

    def shutdown_cleanup(self, executor) -> None:
        """
        Disable active reporting on both nodes and disable target motors.
        Called from main thread while the executor is still spinning in spin_thread.
        Uses threading.Event to wait for each async call — spin_until_future_complete
        cannot be used here because the executor is already spinning.
        """
        import threading
        print('[mimic_node] Shutting down — disabling active reporting and target motors …')
        calls = [
            (self._active_report_client,        self._make_active_report_req(False), f'[{self._source_prefix}] active reporting disabled'),
            (self._target_active_report_client,  self._make_active_report_req(False), f'[{self._target_prefix}] active reporting disabled'),
            (self._enable_motor_client,          self._make_disable_motors_req(),     'target motors disabled'),
        ]
        for client, req, label in calls:
            if not client.service_is_ready():
                print(f'[mimic_node] {client.srv_name} not ready — skipping')
                continue
            done = threading.Event()
            future = client.call_async(req)
            future.add_done_callback(lambda _: done.set())
            done.wait(timeout=2.0)
            if future.done() and future.result() is not None:
                print(f'[mimic_node] {label}: {future.result().message}')
            else:
                print(f'[mimic_node] {label}: timed out or no response')

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
    import threading

    rclpy.init(args=args)
    node     = MimicNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    _stop = threading.Event()

    # Override rclpy's SIGINT handler so we can do cleanup before the context
    # is invalidated.  rclpy.init() installs a handler that calls rcl_shutdown()
    # immediately, which would make any post-shutdown executor creation fail.
    signal.signal(signal.SIGINT, lambda sig, frame: _stop.set())

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    _stop.wait()  # block until SIGINT

    node.shutdown_cleanup(executor)

    executor.shutdown()
    spin_thread.join(timeout=3.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
