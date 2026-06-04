import rclpy
from rclpy.node import Node


class EzMotionNode(Node):
    def __init__(self):
        super().__init__('ezmotion_node')


def main(args=None):
    rclpy.init(args=args)
    node = EzMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
