#!/usr/bin/env python3
"""Republish STM32 feedback as an ordered, read-only ROS joint state."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt32MultiArray
from feedback_clock import FeedbackClock


ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))
GRIPPER_JOINT_NAME = 'gripper_finger_left_joint'


class Stm32StateRepublisher(Node):
    def __init__(self):
        super().__init__('stm32_state_republisher')
        self.declare_parameter('input_topic', '/stm32/joint_states')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('include_gripper', False)
        self.declare_parameter('publish_tf_feedback', False)
        self.declare_parameter('gripper_position_scale', 1.0)
        self.declare_parameter('gripper_position_offset', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        include_gripper = self.get_parameter('include_gripper').value
        self._gripper_position_scale = self.get_parameter(
            'gripper_position_scale').value
        self._gripper_position_offset = self.get_parameter(
            'gripper_position_offset').value
        self._joint_names = list(ARM_JOINT_NAMES)
        if include_gripper:
            self._joint_names.append(GRIPPER_JOINT_NAME)
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
        self._feedback_clock = FeedbackClock()
        self._last_tf_at = None
        self._tf_publisher = None
        if self.get_parameter('publish_tf_feedback').value:
            self._tf_publisher = self.create_publisher(
                JointState, '/robot_feedback/tf_joint_states', input_qos)
        self._feedback_clock_status = self.create_publisher(
            String, '~/time_sync_status', 1)
        self._diagnostics = self.create_subscription(
            UInt32MultiArray, '/stm32/microros_diagnostics',
            lambda m: self._feedback_clock.firmware(m.data, time.monotonic_ns()), input_qos)
        self._feedback_clock_timer = self.create_timer(1., self._publish_clock_status)
        self._missing_joints_logged = False
        self._first_state_logged = False
        self.get_logger().info(
            f'Read-only state relay: {input_topic} -> {output_topic}')

    def _publish_clock_status(self):
        now = time.monotonic_ns()
        self._feedback_clock.refresh(now)
        status = String()
        status.data = json.dumps({
            'state': self._feedback_clock.state, 'message': self._feedback_clock.message,
            'generation': self._feedback_clock.generation,
            'source_offset_ms': self._feedback_clock.offset_ms,
            'feedback_age_ms': None if self._feedback_clock.last_received is None else
            (now-self._feedback_clock.last_received)/1000000,
        })
        self._feedback_clock_status.publish(status)

    def _state_callback(self, message):
        stamp = message.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
            self._feedback_clock.invalidate('反馈时间戳格式无效')
            return
        now = time.monotonic_ns()
        if not self._feedback_clock.observe(stamp.sec*1000000000+stamp.nanosec,
                                   self.get_clock().now().nanoseconds, now):
            return
        indices = {name: index for index, name in enumerate(message.name)}
        missing = [name for name in self._joint_names if name not in indices]
        if missing:
            if not self._missing_joints_logged:
                self.get_logger().error(
                    f'Ignoring STM32 state missing joints: {missing}')
                self._missing_joints_logged = True
            return

        if len(message.position) != len(message.name):
            self.get_logger().error('Ignoring STM32 state with invalid positions')
            return

        ordered_indices = [indices[name] for name in self._joint_names]
        output = JointState()
        output.header = message.header
        output.name = self._joint_names
        output.position = [message.position[index] for index in ordered_indices]
        if GRIPPER_JOINT_NAME in self._joint_names:
            gripper_index = self._joint_names.index(GRIPPER_JOINT_NAME)
            output.position[gripper_index] = (
                output.position[gripper_index] * self._gripper_position_scale +
                self._gripper_position_offset)
        if len(message.velocity) == len(message.name):
            output.velocity = [
                message.velocity[index] for index in ordered_indices]
            if GRIPPER_JOINT_NAME in self._joint_names:
                output.velocity[gripper_index] *= self._gripper_position_scale
        if len(message.effort) == len(message.name):
            output.effort = [message.effort[index] for index in ordered_indices]
        self._publisher.publish(output)
        if self._tf_publisher is not None and (
                self._last_tf_at is None or now-self._last_tf_at >= 50000000):
            self._tf_publisher.publish(output)
            self._last_tf_at = now

        self._missing_joints_logged = False
        if not self._first_state_logged:
            self.get_logger().info(
                f'Publishing ordered feedback: {self._joint_names}')
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
