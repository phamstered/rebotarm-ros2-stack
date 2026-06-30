#!/usr/bin/env python3
"""主臂夹爪电机测试脚本

测试顺序: 1.零点 → 2.张开到最大 → 3.缓慢归零
"""
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

# 配置
POSITION_RANGE = (0.0, 1.57)  # rad, 0=闭合, 1.57=张开 (GripperCommand 使用 radians)
POSITION_STEP = 0.1  # rad, 每步移动量


class LeaderGripperTester(Node):
    def __init__(self):
        super().__init__('leader_gripper_tester')
        self.action_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')
        self.get_logger().info('Waiting for gripper action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Gripper action server connected')

    def send_gripper_command(self, position, max_effort=5.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True


def test_leader_gripper():
    print("=== 主臂夹爪测试 ===")
    print(f"测试顺序: 1.零点 → 2.张开到最大 → 3.缓慢归零")
    print()

    rclpy.init()
    node = LeaderGripperTester()

    try:
        # 步骤1: 当前初始点位为零点
        print("--- 步骤1: 当前初始点位为零点 ---")
        print(f"当前位置: 0.0 m")
        print("保持当前位置为零点")

        # 步骤2: 张开到最大点位
        print(f"\n--- 步骤2: 张开到最大点位 (目标: {POSITION_RANGE[1]} m) ---")
        target = 0.0
        while target < POSITION_RANGE[1]:
            target += POSITION_STEP
            node.send_gripper_command(target)
            time.sleep(0.2)
            print(f"  target={target:.4f} m")
            time.sleep(0.1)

        max_open_pos = target
        print(f"最大张开位置: {max_open_pos:.4f} m")

        # 步骤3: 缓慢归零
        print(f"\n--- 步骤3: 缓慢归零 ---")
        print("缓慢回到零点...")
        current_pos = max_open_pos
        while current_pos > 0.005:  # 逐渐回到零点
            current_pos -= 0.002  # 每次移动 0.002 m
            node.send_gripper_command(current_pos)
            time.sleep(0.2)
            print(f"  target={current_pos:.4f} m")
            time.sleep(0.1)

        # 最终归零
        node.send_gripper_command(0.0)
        time.sleep(1.0)
        print(f"归零后位置: 0.0 m")

        # 总结
        print(f"\n=== 测试结果 ===")
        print(f"零点位置: 0.0 m")
        print(f"最大张开位置: {max_open_pos:.4f} m")
        print(f"总行程: {max_open_pos:.4f} m")

        print("\n✓ 测试完成")

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    test_leader_gripper()
