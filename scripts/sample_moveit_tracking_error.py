#!/usr/bin/env python3
"""Sample MoveIt trajectory-controller tracking error and print statistics."""

import argparse
import math
import statistics
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from microros_stm32h723_bridge.msg import HardwareStatus, JointCommand


DEFAULT_STATE_TOPIC = "/joint1_trajectory_controller/controller_state"
DEFAULT_COMMAND_TOPIC = "/stm32/joint_commands"
DEFAULT_STATUS_TOPIC = "/stm32/status"


class TrackingErrorSampler(Node):
    def __init__(self, interval: float, settle: float) -> None:
        super().__init__("moveit_tracking_error_sampler")
        self.interval = interval
        self.settle_deadline = time.monotonic() + settle
        self.last_sample_time = float("-inf")
        self.joint_names = []
        self.samples = []
        self.command = None
        self.status = None
        best_effort_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            JointTrajectoryControllerState,
            DEFAULT_STATE_TOPIC,
            self._state_callback,
            10,
        )
        self.create_subscription(
            JointCommand, DEFAULT_COMMAND_TOPIC, self._command_callback,
            best_effort_qos)
        self.create_subscription(
            HardwareStatus, DEFAULT_STATUS_TOPIC, self._status_callback,
            best_effort_qos)

    def _state_callback(self, message: JointTrajectoryControllerState) -> None:
        now = time.monotonic()
        if now < self.settle_deadline or now - self.last_sample_time < self.interval:
            return
        size = len(message.joint_names)
        if not (
            len(message.desired.positions) == size
            and len(message.actual.positions) == size
            and len(message.error.positions) == size
        ):
            self.get_logger().warning("Ignored incomplete controller-state message")
            return
        if self.joint_names and list(message.joint_names) != self.joint_names:
            self.get_logger().warning("Ignored message with changed joint ordering")
            return
        self.joint_names = list(message.joint_names)
        self.samples.append(
            (
                list(message.desired.positions),
                list(message.actual.positions),
                list(message.error.positions),
            )
        )
        self.last_sample_time = now

    def _command_callback(self, message: JointCommand) -> None:
        self.command = message

    def _status_callback(self, message: HardwareStatus) -> None:
        self.status = message


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sample desired/actual joint positions and tracking errors.")
    parser.add_argument(
        "--duration", type=float, default=10.0,
        help="Sampling-window duration in seconds (default: 10).")
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="Minimum interval between retained samples (default: 0.5).")
    parser.add_argument(
        "--settle", type=float, default=1.0,
        help="Delay before retaining samples (default: 1).")
    return parser.parse_args()


def validate_args(args) -> None:
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.interval <= 0:
        raise ValueError("--interval must be positive")
    if args.settle < 0:
        raise ValueError("--settle cannot be negative")


def format_vector(values, count=6) -> str:
    return "[" + ", ".join(f"{value:.3f}" for value in values[:count]) + "]"


def print_environment(sampler: TrackingErrorSampler) -> None:
    print("\nRuntime state")
    if sampler.command is None:
        print("  STM32 command: not received")
    else:
        print(f"  mode: {sampler.command.mode}")
        print(f"  kp: {format_vector(sampler.command.kp)}")
        print(f"  kd: {format_vector(sampler.command.kd)}")
        print(f"  effort: {format_vector(sampler.command.effort)}")
    if sampler.status is None:
        print("  STM32 status: not received")
    else:
        print(f"  fault_code: {sampler.status.fault_code}")
        print(f"  active_joint_mask: 0x{sampler.status.active_joint_mask:02X}")
        print(f"  last_command_sequence: {sampler.status.last_command_sequence}")


def print_statistics(sampler: TrackingErrorSampler) -> None:
    print(f"\nSamples retained: {len(sampler.samples)}")
    print("Error definition: desired - actual")
    print(
        "\n| joint | desired mean (rad) | actual mean (rad) | "
        "error mean (rad) | mean abs (rad) | min (rad) | max (rad) | "
        "std (rad) | mean (deg) |")
    print(
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|")

    for index, name in enumerate(sampler.joint_names):
        desired = [sample[0][index] for sample in sampler.samples]
        actual = [sample[1][index] for sample in sampler.samples]
        errors = [sample[2][index] for sample in sampler.samples]
        error_mean = statistics.fmean(errors)
        print(
            f"| {name} | {statistics.fmean(desired):.6f} | "
            f"{statistics.fmean(actual):.6f} | {error_mean:+.6f} | "
            f"{statistics.fmean(abs(value) for value in errors):.6f} | "
            f"{min(errors):+.6f} | {max(errors):+.6f} | "
            f"{statistics.pstdev(errors):.6f} | "
            f"{math.degrees(error_mean):+.2f} |")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print(f"ERROR: {error}")
        return 2

    rclpy.init()
    sampler = TrackingErrorSampler(args.interval, args.settle)
    print(
        f"Sampling {DEFAULT_STATE_TOPIC} for {args.duration:.1f}s "
        f"after a {args.settle:.1f}s settle delay...")
    deadline = time.monotonic() + args.settle + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(sampler, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print_environment(sampler)
    if not sampler.samples:
        print(f"\nERROR: no messages received from {DEFAULT_STATE_TOPIC}")
        sampler.destroy_node()
        rclpy.shutdown()
        return 1

    print_statistics(sampler)
    sampler.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
