import signal
import threading
import time
import tomllib
from typing import Dict

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy
from custom_interfaces.msg import PositionPPCommand, VelocityCommand
from custom_interfaces.srv import EnableMotor, SetActiveReport, SetRunMode


_STEER_MOTORS = ('front_left_steer', 'front_right_steer', 'rear_steer')
_WHEEL_MOTORS = ('front_left_wheel', 'front_right_wheel', 'rear_wheel')

_RUN_MODE_PP       = 1   # POSITION_PP
_RUN_MODE_VELOCITY = 2   # VELOCITY


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        self._cb_subs  = MutuallyExclusiveCallbackGroup()
        self._cb_srvs  = ReentrantCallbackGroup()
        self._cb_setup = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        cfg = self._load_config(config_path)

        self._motors:           Dict[str, str]  = dict(cfg['motors'])
        self._prefix:           str             = cfg.get('motor_node_prefix', '')
        self._active_report_hz: float           = float(cfg.get('active_report_hz', 50.0))
        self._max_linear_vel: float = float(cfg.get('max_linear_vel', 1.0))
        self._wheel_radius:   float = float(cfg.get('wheel_radius',   0.1))
        self._joy_timeout:    float = float(cfg.get('joy_timeout',    0.5))
        self._joy_topic:        str             = cfg.get('joy_topic', '/joy')

        joy_axes                      = cfg.get('joy_axes', {})
        self._axis_wheel:         int = int(joy_axes.get('wheel_velocity', 3))
        self._axis_steer:         int = int(joy_axes.get('steering_angle', 0))
        self._max_steering_angle: float = float(cfg.get('max_steering_angle', 0.5))

        self._ipr_fl:   float = float(cfg.get('in_place_rotate_front_left_steer_angle',  -0.7853))
        self._ipr_fr:   float = float(cfg.get('in_place_rotate_front_right_steer_angle', 0.7853))
        self._ipr_rear: float = float(cfg.get('in_place_rotate_rear_steer_angle',        1.57))

        joy_buttons                      = cfg.get('joy_buttons', {})
        self._btn_enable_joy:        int = int(joy_buttons.get('enable_joystick',  4))
        self._btn_disable_joy:       int = int(joy_buttons.get('disable_joystick', 6))
        self._btn_enable_steer:      int = int(joy_buttons.get('enable_steer',     1))
        self._btn_disable_steer:     int = int(joy_buttons.get('disable_steer',    2))
        self._btn_enable_wheel:      int = int(joy_buttons.get('enable_wheel',     3))
        self._btn_disable_wheel:     int = int(joy_buttons.get('disable_wheel',    0))
        self._btn_low_speed:         int = int(joy_buttons.get('low_speed',               9))
        self._btn_high_speed:        int = int(joy_buttons.get('high_speed',              10))
        self._btn_enable_ipr:        int = int(joy_buttons.get('enable_in_place_rotate',  12))
        self._btn_disable_ipr:       int = int(joy_buttons.get('disable_in_place_rotate', 13))

        self._prev_buttons: list = []   # previous Joy.buttons for rising-edge detection

        pp = cfg.get('pp_defaults', {})
        self._pp_defaults: Dict[str, float] = {
            'speed':        float(pp.get('speed',        5.0)),
            'acceleration': float(pp.get('acceleration', 10.0)),
            'deceleration': float(pp.get('deceleration', 10.0)),
            'torque_limit': float(pp.get('torque_limit', 0.0)),
        }

        vd = cfg.get('velocity_defaults', {})
        self._vel_defaults: Dict[str, float] = {
            'current_limit': float(vd.get('current_limit', 0.0)),
            'acceleration':  float(vd.get('acceleration',  20.0)),
        }

        self._setup_done:           bool  = False
        self._enabled:              bool  = False
        self._ready:                bool  = False
        self._steer_enabled:        bool  = False
        self._wheel_enabled:        bool  = False
        _default_mode = cfg.get('speed_mode', 'low_speed')
        self._speed_scale:          float = 0.5 if _default_mode == 'low_speed' else 1.0
        self._last_joy_time:        float = 0.0   # monotonic timestamp of last Joy message
        self._joy_watchdog_tripped: bool  = False  # True while timed-out waiting for Joy
        self._in_place_rotate:      bool  = False  # True when in-place rotate mode is active

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers: one per motor
        self._pp_pubs:  Dict[str, object] = {}
        self._vel_pubs: Dict[str, object] = {}

        for role in _STEER_MOTORS:
            name = self._motors[role]
            self._pp_pubs[role] = self.create_publisher(
                PositionPPCommand,
                f'{self._prefix}/motors/{name}/cmd_position_pp',
                qos,
            )

        for role in _WHEEL_MOTORS:
            name = self._motors[role]
            self._vel_pubs[role] = self.create_publisher(
                VelocityCommand,
                f'{self._prefix}/motors/{name}/cmd_velocity',
                qos,
            )

        # Service clients
        self._active_report_client = self.create_client(
            SetActiveReport,
            f'{self._prefix}/set_active_report',
            callback_group=self._cb_srvs,
        )
        self._set_run_mode_client = self.create_client(
            SetRunMode,
            f'{self._prefix}/set_run_mode',
            callback_group=self._cb_srvs,
        )
        self._enable_motor_client = self.create_client(
            EnableMotor,
            f'{self._prefix}/enable_motor',
            callback_group=self._cb_srvs,
        )

        # Joystick subscriber
        self.create_subscription(
            Joy,
            self._joy_topic,
            self._on_joy,
            qos,
            callback_group=self._cb_subs,
        )

        # One-shot setup timer — fires after 1 s to let motor node start
        self._setup_timer = self.create_timer(
            1.0, self._setup_once, callback_group=self._cb_setup
        )

        # Watchdog: disable motors if no Joy message within joy_timeout seconds
        self._cb_watchdog = MutuallyExclusiveCallbackGroup()
        self.create_timer(
            min(self._joy_timeout / 2.0, 0.1),
            self._watchdog_check,
            callback_group=self._cb_watchdog,
        )

        self.get_logger().info(
            f'TeleopNode init — prefix={self._prefix}  steer=PP  wheel=VELOCITY  '
            f'joy={self._joy_topic}  max_vel={self._max_linear_vel} m/s  '
            f'max_steer={self._max_steering_angle} rad'
        )

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

    # ── Startup ───────────────────────────────────────────────────────────────

    def _setup_once(self) -> None:
        self._setup_timer.cancel()
        self._setup_timer = None

        self._blocking_set_active_report(enable=True)
        self._blocking_set_run_mode_steer()
        self._blocking_set_run_mode_wheel()
        self._blocking_enable_all()

        self._steer_enabled = True
        self._wheel_enabled = True
        self._setup_done = True
        self.get_logger().info(
            f'Setup complete — press button {self._btn_enable_joy} to enable commands'
        )

    def _blocking_set_active_report(self, enable: bool) -> None:
        if not self._active_report_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._prefix}/set_active_report not available')
            return
        req        = SetActiveReport.Request()
        req.name   = 'all'
        req.enable = enable
        req.hz     = self._active_report_hz if enable else 0.0
        res = self._active_report_client.call(req)
        if res.success:
            self.get_logger().info(f'Active reporting {"enabled" if enable else "disabled"}: {res.message}')
        else:
            self.get_logger().error(f'Active reporting {"enabled" if enable else "disabled"}: {res.message}')

    def _blocking_set_run_mode(self, name: str, mode: int) -> None:
        req                          = SetRunMode.Request()
        req.name                     = name
        req.mode                     = mode
        req.automatic_enable_disable = True
        res = self._set_run_mode_client.call(req)
        if res.success:
            self.get_logger().info(f'[{name}] run mode → {mode}: {res.message}')
        else:
            self.get_logger().error(f'[{name}] run mode → {mode}: {res.message}')

    def _blocking_set_run_mode_steer(self) -> None:
        if not self._set_run_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._prefix}/set_run_mode not available')
            return
        for role in _STEER_MOTORS:
            self._blocking_set_run_mode(self._motors[role], _RUN_MODE_PP)

    def _blocking_set_run_mode_wheel(self) -> None:
        if not self._set_run_mode_client.service_is_ready():
            return
        for role in _WHEEL_MOTORS:
            self._blocking_set_run_mode(self._motors[role], _RUN_MODE_VELOCITY)

    def _blocking_enable_all(self) -> None:
        if not self._enable_motor_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{self._prefix}/enable_motor not available')
            return
        req             = EnableMotor.Request()
        req.name        = 'all'
        req.enable      = True
        req.clear_fault = False
        res = self._enable_motor_client.call(req)
        if res.success:
            self.get_logger().info(f'Enable all motors: {res.message}')
        else:
            self.get_logger().error(f'Enable all motors: {res.message}')

    # ── Joystick callback ─────────────────────────────────────────────────────

    def _rising_edge(self, curr: list, prev: list, index: int) -> bool:
        c = curr[index] if index < len(curr) else 0
        p = prev[index] if index < len(prev) else 0
        return c == 1 and p == 0

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy_time = time.monotonic()

        if self._joy_watchdog_tripped and self._ready:
            self._joy_watchdog_tripped = False
            self.get_logger().info('Joy resumed — re-enabling motors')
            if self._steer_enabled:
                self._async_enable_motors(_STEER_MOTORS, enable=True)
            if self._wheel_enabled:
                self._async_enable_motors(_WHEEL_MOTORS, enable=True)

        curr = list(msg.buttons)
        prev = self._prev_buttons

        # ── Button actions (rising edge) ──────────────────────────────────────
        if self._rising_edge(curr, prev, self._btn_enable_joy) and self._setup_done:
            self._enabled = True
            self._ready   = True
            self.get_logger().info('Joystick ENABLED')

        if self._rising_edge(curr, prev, self._btn_disable_joy):
            self._enabled = False
            self._ready   = False
            self.get_logger().info('Joystick DISABLED')

        if self._rising_edge(curr, prev, self._btn_enable_steer):
            self._steer_enabled = True
            self._async_enable_motors(_STEER_MOTORS, enable=True)

        if self._rising_edge(curr, prev, self._btn_disable_steer):
            self._steer_enabled = False
            self._async_enable_motors(_STEER_MOTORS, enable=False)

        if self._rising_edge(curr, prev, self._btn_enable_wheel):
            self._wheel_enabled = True
            self._async_enable_motors(_WHEEL_MOTORS, enable=True)

        if self._rising_edge(curr, prev, self._btn_disable_wheel):
            self._wheel_enabled = False
            self._async_enable_motors(_WHEEL_MOTORS, enable=False)

        if self._rising_edge(curr, prev, self._btn_low_speed):
            self._speed_scale = 0.5
            self.get_logger().info('Speed mode: LOW (50% max vel)')

        if self._rising_edge(curr, prev, self._btn_high_speed):
            self._speed_scale = 1.0
            self.get_logger().info('Speed mode: HIGH (100% max vel)')

        if self._rising_edge(curr, prev, self._btn_enable_ipr):
            self._in_place_rotate = True
            self.get_logger().info('In-place rotate mode ENABLED')

        if self._rising_edge(curr, prev, self._btn_disable_ipr):
            self._in_place_rotate = False
            self.get_logger().info('In-place rotate mode DISABLED')

        self._prev_buttons = curr

        if not self._ready:
            return

        # ── Axis commands ─────────────────────────────────────────────────────
        linear          = self._axis_value(msg, self._axis_wheel) * self._max_linear_vel * self._speed_scale * -1.0
        wheel_vel_rad_s = linear / self._wheel_radius

        if self._in_place_rotate:
            self._publish_wheel('front_left_wheel',  wheel_vel_rad_s)
            self._publish_wheel('front_right_wheel', wheel_vel_rad_s)
            self._publish_wheel('rear_wheel',        wheel_vel_rad_s)
            self._publish_steer('front_left_steer',  self._ipr_fl)
            self._publish_steer('front_right_steer', self._ipr_fr)
            self._publish_steer('rear_steer',        self._ipr_rear)
        else:
            self._publish_wheel('front_left_wheel',  wheel_vel_rad_s)
            self._publish_wheel('front_right_wheel', wheel_vel_rad_s * -1.0)
            self._publish_wheel('rear_wheel',        wheel_vel_rad_s)
            steer_angle = self._axis_value(msg, self._axis_steer) * self._max_steering_angle
            self._publish_steer('front_left_steer',  steer_angle)
            self._publish_steer('front_right_steer', steer_angle)
            self._publish_steer('rear_steer',        steer_angle)

    def _watchdog_check(self) -> None:
        if not self._ready or self._last_joy_time == 0.0 or self._joy_watchdog_tripped:
            return
        if time.monotonic() - self._last_joy_time > self._joy_timeout:
            self._joy_watchdog_tripped = True
            self.get_logger().warning(
                f'No Joy message for >{self._joy_timeout}s — disabling motors'
            )
            self._async_enable_motors(_STEER_MOTORS + _WHEEL_MOTORS, enable=False)

    def _async_enable_motors(self, roles: tuple, enable: bool) -> None:
        if not self._enable_motor_client.service_is_ready():
            self.get_logger().warning('enable_motor service not ready')
            return
        for role in roles:
            req             = EnableMotor.Request()
            req.name        = self._motors[role]
            req.enable      = enable
            req.clear_fault = False
            future = self._enable_motor_client.call_async(req)
            future.add_done_callback(
                lambda f, n=self._motors[role], e=enable: (
                    self.get_logger().info(f'[{n}] {"enabled" if e else "disabled"}')
                    if f.result() is not None
                    else self.get_logger().error(f'[{n}] enable_motor call failed')
                )
            )

    def _axis_value(self, msg: Joy, index: int) -> float:
        if index < len(msg.axes):
            return float(msg.axes[index])
        return 0.0

    def _publish_wheel(self, role: str, velocity_rad_s: float) -> None:
        if not self._wheel_enabled:
            return
        cmd               = VelocityCommand()
        cmd.name          = self._motors[role]
        cmd.velocity      = velocity_rad_s
        cmd.current_limit = self._vel_defaults['current_limit']
        cmd.acceleration  = self._vel_defaults['acceleration']
        self._vel_pubs[role].publish(cmd)

    def _publish_steer(self, role: str, position_rad: float) -> None:
        if not self._steer_enabled:
            return
        cmd              = PositionPPCommand()
        cmd.name         = self._motors[role]
        cmd.position     = position_rad
        cmd.speed        = self._pp_defaults['speed']
        cmd.acceleration = self._pp_defaults['acceleration']
        cmd.deceleration = self._pp_defaults['deceleration']
        cmd.torque_limit = self._pp_defaults['torque_limit']
        self._pp_pubs[role].publish(cmd)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown_cleanup(self) -> None:
        print('[teleop_node] Shutting down — disabling motors and active reporting …')
        if self._enable_motor_client.service_is_ready():
            req             = EnableMotor.Request()
            req.name        = 'all'
            req.enable      = False
            req.clear_fault = False
            done = threading.Event()
            future = self._enable_motor_client.call_async(req)
            future.add_done_callback(lambda _: done.set())
            done.wait(timeout=2.0)

        if self._active_report_client.service_is_ready():
            self._blocking_set_active_report(enable=False)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node     = TeleopNode()
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
