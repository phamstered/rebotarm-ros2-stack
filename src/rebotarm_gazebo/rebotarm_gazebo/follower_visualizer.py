#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class FollowerVisualizer(Node):
    def __init__(self):
        super().__init__('follower_visualizer')
        self.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6']
        self.positions = [0.0] * 6

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_commands', self._cb, 10)
        self.create_timer(0.02, self._publish)

        self.get_logger().info('Follower visualizer started, waiting for /joint_commands...')

    def _cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                idx = self.joint_names.index(name)
                self.positions[idx] = msg.position[i]

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
