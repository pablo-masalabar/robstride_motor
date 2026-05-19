#!/usr/bin/env python3
"""
Subscribes to real-robot joint state topics and forwards positions to the
Gazebo sim controllers so the simulated robot mirrors the real one.

Each bridge entry in config maps one source JointState topic to one sim
controller topic. Sink type controls the output message format:
  trajectory  ->  trajectory_msgs/JointTrajectory  (arms, neck, torso, brackets)
  velocity    ->  std_msgs/Float64MultiArray        (wheel velocity controllers)
"""

import tomllib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MimicSimNode(Node):
    def __init__(self):
        super().__init__('mimic_sim')

        self.declare_parameter('config_path', '')
        cfg_path = self.get_parameter('config_path').value
        if not cfg_path:
            raise RuntimeError('config_path parameter is required')

        with open(cfg_path, 'rb') as f:
            cfg = tomllib.load(f)

        if cfg.get('node_name'):
            # rclpy does not support renaming after init; log the intended name
            self.get_logger().info(f"node_name from config: {cfg['node_name']}")

        traj_sec = cfg.get('trajectory_time_sec', 0.1)
        self._traj_ns = int(traj_sec * 1e9)

        self._subs: list = []

        for bridge in cfg.get('bridges', []):
            src   = bridge['source_topic']
            sink  = bridge['sink_topic']
            stype = bridge.get('sink_type', 'trajectory')

            if stype == 'trajectory':
                pub = self.create_publisher(JointTrajectory, sink, 10)
                sub = self.create_subscription(
                    JointState, src,
                    lambda msg, p=pub: self._forward_trajectory(msg, p),
                    10,
                )
            elif stype == 'velocity':
                pub = self.create_publisher(Float64MultiArray, sink, 10)
                sub = self.create_subscription(
                    JointState, src,
                    lambda msg, p=pub: self._forward_velocity(msg, p),
                    10,
                )
            else:
                self.get_logger().warn(f'Unknown sink_type "{stype}" for {src}, skipping')
                continue

            self._subs.append(sub)
            self.get_logger().info(f'Bridge: {src} → {sink} [{stype}]')

    def _forward_trajectory(self, msg: JointState, pub) -> None:
        if not msg.name or not msg.position:
            return

        traj             = JointTrajectory()
        traj.header      = msg.header
        traj.joint_names = list(msg.name)

        pt           = JointTrajectoryPoint()
        pt.positions = list(msg.position)
        if len(msg.velocity) == len(msg.name):
            pt.velocities = list(msg.velocity)
        pt.time_from_start = Duration(
            sec     = self._traj_ns // 1_000_000_000,
            nanosec = self._traj_ns  % 1_000_000_000,
        )

        traj.points = [pt]
        pub.publish(traj)

    def _forward_velocity(self, msg: JointState, pub) -> None:
        if not msg.velocity:
            return
        cmd      = Float64MultiArray()
        cmd.data = list(msg.velocity)
        pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MimicSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
