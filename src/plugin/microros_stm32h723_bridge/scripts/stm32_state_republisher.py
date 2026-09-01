#!/usr/bin/env python3
"""Republish STM32 feedback as an ordered, read-only ROS joint state."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))


class Stm32StateRepublisher(Node):
    def __init__(self):
        super().__init__('stm32_state_republisher')
        self.declare_parameter('input_topic', '/stm32/joint_states')
        self.declare_parameter('output_topic', '/joint_states')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            JointState, output_topic, output_qos)
        self._subscription = self.create_subscription(
            JointState, input_topic, self._state_callback, input_qos)
        self._missing_joints_logged = False
        self._first_state_logged = False
        self.get_logger().info(
            f'Read-only state relay: {input_topic} -> {output_topic}')

    def _state_callback(self, message):
        indices = {name: index for index, name in enumerate(message.name)}
        missing = [name for name in JOINT_NAMES if name not in indices]
        if missing:
            if not self._missing_joints_logged:
                self.get_logger().error(
                    f'Ignoring STM32 state missing joints: {missing}')
                self._missing_joints_logged = True
            return

        if len(message.position) != len(message.name):
            self.get_logger().error('Ignoring STM32 state with invalid positions')
            return

        ordered_indices = [indices[name] for name in JOINT_NAMES]
        output = JointState()
        output.header = message.header
        output.name = list(JOINT_NAMES)
        output.position = [message.position[index] for index in ordered_indices]
        if len(message.velocity) == len(message.name):
            output.velocity = [
                message.velocity[index] for index in ordered_indices]
        if len(message.effort) == len(message.name):
            output.effort = [message.effort[index] for index in ordered_indices]
        self._publisher.publish(output)

        self._missing_joints_logged = False
        if not self._first_state_logged:
            self.get_logger().info('Publishing ordered joint1-6 feedback')
            self._first_state_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = Stm32StateRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
