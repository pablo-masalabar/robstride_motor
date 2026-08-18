"""
test_torso.py — interactive torso height tester.

Usage:
    ros2 run ezmotion_p test_torso -- \
        --target_topic /torso/motors/torso/go_to \
        --increment 1.0 \
        --start_val 600.0

Up arrow   → increment position by --increment
Down arrow → decrement position by --increment
q / Ctrl-C → quit

Works in both PP and CSP motor modes — the node publishes Float64 height (mm)
and the motor_node dispatches to trigger_move_pp or set_cyclic_position accordingly.

Service names are derived from --target_topic:
    /torso/motors/torso/go_to  →  prefix=/torso, motor=torso
    → /torso/enable_motor
    → /torso/set_active_report
"""

import argparse
import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from custom_interfaces.srv import EnableMotor, SetActiveReport


_HEIGHT_MIN    = 50.0
_HEIGHT_MAX    = 600.0
_ACTIVE_RPT_HZ = 60.0

_UP   = b'\x1b[A'
_DOWN = b'\x1b[B'


def _parse_topic(topic: str):
    """Extract (node_prefix, motor_name) from /prefix/motors/name/suffix."""
    parts = topic.split('/')
    try:
        idx        = parts.index('motors')
        prefix     = '/' + '/'.join(parts[1:idx])
        motor_name = parts[idx + 1]
        return prefix, motor_name
    except (ValueError, IndexError):
        return None, None


def _read_key() -> bytes:
    """Block until one keypress is available; return its raw bytes."""
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.buffer.read(1)
        if ch == b'\x1b':
            ch += sys.stdin.buffer.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _call_service(client, request, timeout: float = 3.0):
    """Call a service async and block via threading.Event (safe from any thread)."""
    done   = threading.Event()
    result = [None]

    def _cb(future):
        try:
            result[0] = future.result()
        except Exception:
            pass
        done.set()

    client.call_async(request).add_done_callback(_cb)
    done.wait(timeout=timeout)
    return result[0]


class TestTorsoNode(Node):

    def __init__(self, target_topic: str, increment: float, start_val: float,
                 publish_hz: float = 60.0):
        super().__init__('test_torso')

        self._increment  = increment
        self._position   = max(_HEIGHT_MIN, min(_HEIGHT_MAX, start_val))
        self._pub        = self.create_publisher(Float64, target_topic, 10)

        self.create_timer(1.0 / publish_hz, self._publish)

        prefix, motor_name = _parse_topic(target_topic)
        self._motor_name   = motor_name

        if prefix and motor_name:
            self._enable_client = self.create_client(EnableMotor,     f'{prefix}/enable_motor')
            self._report_client = self.create_client(SetActiveReport,  f'{prefix}/set_active_report')
        else:
            self.get_logger().warning(
                f'Could not parse prefix/motor from "{target_topic}" — '
                'enable_motor and set_active_report will not be called'
            )
            self._enable_client = None
            self._report_client = None

        self.get_logger().info(
            f'test_torso ready — topic={target_topic}  increment={increment} mm  '
            f'start={self._position} mm  publish_hz={publish_hz}  '
            f'range=[{_HEIGHT_MIN}, {_HEIGHT_MAX}] mm'
        )
        self.get_logger().info('Up arrow → up  |  Down arrow → down  |  q → quit')

    def setup(self) -> None:
        """Enable active report and motor. Called from main thread after spin starts."""
        self._set_active_report(enable=True)
        self._enable_motor(enable=True)

    def shutdown(self) -> None:
        """Disable active report and motor. Called from main thread before exit."""
        self._set_active_report(enable=False)
        self._enable_motor(enable=False)

    def _enable_motor(self, enable: bool) -> None:
        if self._enable_client is None:
            return
        if not self._enable_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('enable_motor service not available')
            return
        req             = EnableMotor.Request()
        req.name        = self._motor_name
        req.enable      = enable
        req.clear_fault = False
        res    = _call_service(self._enable_client, req)
        action = 'enabled' if enable else 'disabled'
        if res and res.success:
            self.get_logger().info(f'Motor {action}')
        else:
            self.get_logger().error(f'Failed to {action[:-1]} motor')

    def _set_active_report(self, enable: bool) -> None:
        if self._report_client is None:
            return
        if not self._report_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('set_active_report service not available')
            return
        req        = SetActiveReport.Request()
        req.name   = self._motor_name
        req.enable = enable
        req.hz     = _ACTIVE_RPT_HZ if enable else 0.0
        res    = _call_service(self._report_client, req)
        action = f'enabled @ {_ACTIVE_RPT_HZ} Hz' if enable else 'disabled'
        if res and res.success:
            self.get_logger().info(f'Active report {action}')
        else:
            self.get_logger().error(f'Failed to set active report ({action})')

    def _publish(self) -> None:
        msg      = Float64()
        msg.data = self._position
        self._pub.publish(msg)

    def step(self, direction: int) -> None:
        new_pos = self._position + direction * self._increment
        if new_pos > _HEIGHT_MAX:
            self.get_logger().warning(f'At max height ({_HEIGHT_MAX} mm) — ignoring')
            return
        if new_pos < _HEIGHT_MIN:
            self.get_logger().warning(f'At min height ({_HEIGHT_MIN} mm) — ignoring')
            return
        self._position = new_pos
        self.get_logger().info(f'go_to → {self._position:.1f} mm')


def main(args=None):
    parser = argparse.ArgumentParser(description='Interactive torso height tester')
    parser.add_argument('--target_topic', default='/torso/motors/torso/go_to',
                        help='go_to topic to publish on')
    parser.add_argument('--increment', type=float, default=10.0,
                        help='Position step per keypress (mm)')
    parser.add_argument('--start_val', type=float, default=300.0,
                        help='Starting height (mm)')
    parser.add_argument('--publish_hz', type=float, default=60.0,
                        help='Rate at which last command is republished (Hz)')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = TestTorsoNode(
        target_topic=parsed.target_topic,
        increment=parsed.increment,
        start_val=parsed.start_val,
        publish_hz=parsed.publish_hz,
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.setup()

    try:
        while rclpy.ok():
            key = _read_key()
            if key == _UP:
                node.step(+1)
            elif key == _DOWN:
                node.step(-1)
            elif key in (b'q', b'Q', b'\x03'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
