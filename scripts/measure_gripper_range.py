#!/usr/bin/env python3
"""自动测量从臂夹爪电机运动范围

功能：
1. 缓慢移动夹爪到完全闭合位置
2. 缓慢移动夹爪到完全张开位置
3. 记录电机位置范围
4. 输出标定所需的参数

使用方法：
    python3 scripts/measure_gripper_range.py [--step SIZE] [--speed SPEED]

注意：
- 运行前确保从臂驱动未启动（本脚本直接控制电机）
- 夹爪会缓慢移动到两个极限位置
"""

import argparse
import sys
import os
import time
import threading
import numpy as np
from pathlib import Path

from reBotArm_control_py.actuator.gripper import Gripper

# 全局目标位置，控制循环中使用
_target_pos = [0.0]
_target_lock = threading.Lock()


def _gripper_controller(gc, dt):
    """控制循环回调"""
    with _target_lock:
        target = _target_pos[0]
    gc.mit(pos=target, kp=50.0, kd=5.0)


def measure_position(gripper, num_samples=10, delay=0.05):
    """多次采样取平均值，获取稳定的位置读数"""
    positions = []
    for _ in range(num_samples):
        pos = gripper.get_position(request=True)
        positions.append(pos)
        time.sleep(delay)
    return np.mean(positions), np.std(positions)


def find_limit(gripper, direction, step_size, threshold_stall, timeout):
    """
    移动到极限位置

    Args:
        gripper: Gripper 实例
        direction: 1.0 (闭合) 或 -1.0 (张开)
        step_size: 每步移动量 (rad)
        threshold_stall: 堵转检测阈值 (rad)
        timeout: 超时时间 (秒)

    Returns:
        (final_position, velocity_std)
    """
    global _target_pos
    print(f"\n  移动方向: {'闭合' if direction > 0 else '张开'}")

    # 初始位置
    pos_mean, _ = measure_position(gripper)
    start_pos = pos_mean
    print(f"  起始位置: {pos_mean:.4f} rad")

    # 设置初始目标
    with _target_lock:
        _target_pos[0] = pos_mean

    stall_count = 0
    stall_threshold = 5  # 连续N次位置变化小于阈值则认为堵转
    history = []
    start_time = time.time()

    while time.time() - start_time < timeout:
        # 计算目标位置
        target = pos_mean + direction * step_size

        # 通过控制循环发送位置命令
        with _target_lock:
            _target_pos[0] = target
        time.sleep(0.15)  # 等待电机响应

        # 测量新位置
        new_pos_mean, new_pos_std = measure_position(gripper)
        delta = abs(new_pos_mean - pos_mean)

        # 记录历史
        history.append(new_pos_mean)

        # 显示进度
        print(f"\r  位置: {new_pos_mean:+.4f} rad | "
              f"目标: {target:+.4f} rad | "
              f"变化: {delta:.5f} rad | "
              f"堵转计数: {stall_count}/{stall_threshold}",
              end='', flush=True)

        # 检测堵转
        if delta < threshold_stall:
            stall_count += 1
            if stall_count >= stall_threshold:
                print(f"\n  ✓ 检测到堵转，已到达极限位置")
                break
        else:
            stall_count = 0

        pos_mean = new_pos_mean

    # 计算最终位置和速度标准差
    final_pos, _ = measure_position(gripper, num_samples=20)

    # 计算最后几步的速度标准差（用于判断是否真的堵转）
    if len(history) > 5:
        velocities = np.diff(history[-5:])
        vel_std = np.std(velocities)
    else:
        vel_std = float('inf')

    print(f"  最终位置: {final_pos:.4f} rad")
    return final_pos, vel_std


def main():
    parser = argparse.ArgumentParser(description='测量从臂夹爪电机运动范围')
    parser.add_argument('--step', type=float, default=0.2,
                        help='每步移动量 (rad, 默认 0.2)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='移动速度系数 (0.1-1.0, 默认 1.0)')
    parser.add_argument('--threshold', type=float, default=0.001,
                        help='堵转检测阈值 (rad, 默认 0.001)')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='单方向超时时间 (秒, 默认 60)')
    parser.add_argument('--config', type=str, default=None,
                        help='Gripper 配置文件路径 (默认: src/rebotarm_hw_driver/config/gripper_can.yaml)')
    parser.add_argument('--auto', action='store_true',
                        help='自动模式，跳过确认提示')
    args = parser.parse_args()

    # 默认使用 leader_follower 项目的 CAN 配置
    if args.config is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        args.config = os.path.join(project_dir, 'src/rebotarm_hw_driver/config/gripper_can.yaml')

    # 调整步长
    step_size = args.step * args.speed

    print("\n" + "=" * 60)
    print("从臂夹爪电机运动范围测量")
    print("=" * 60)
    print(f"\n参数:")
    print(f"  步长: {step_size:.4f} rad")
    print(f"  堵转阈值: {args.threshold:.4f} rad")
    print(f"  超时时间: {args.timeout} 秒")
    print("\n注意事项:")
    print("  1. 确保从臂驱动未启动")
    print("  2. 夹爪将缓慢移动到两个极限位置")
    print("  3. 请勿触碰夹爪，等待测量完成")
    print("=" * 60)

    if not args.auto:
        input("\n按 Enter 开始测量...")
    else:
        print("\n自动模式，3秒后开始...")
        time.sleep(3)

    # 连接夹爪
    print("\n正在连接夹爪电机...")
    try:
        gripper = Gripper(args.config)
        gripper.enable()
        gripper.mode_mit(kp=20.0, kd=3.0)
        # 启动控制循环
        gripper.start_control_loop(_gripper_controller, rate=100)
        print("✓ 夹爪电机已连接并使能 (控制循环已启动)")
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        return

    try:
        # 获取初始位置
        initial_pos, _ = measure_position(gripper)
        print(f"\n初始位置: {initial_pos:.4f} rad")

        # 步骤1: 移动到完全闭合位置
        print("\n" + "-" * 60)
        print("步骤 1/2: 移动到完全闭合位置")
        print("-" * 60)
        closed_pos, _ = find_limit(
            gripper,
            direction=1.0,  # 正方向 = 闭合
            step_size=step_size,
            threshold_stall=args.threshold,
            timeout=args.timeout
        )

        # 短暂暂停
        time.sleep(1.0)

        # 步骤2: 移动到完全张开位置
        print("\n" + "-" * 60)
        print("步骤 2/2: 移动到完全张开位置")
        print("-" * 60)
        open_pos, _ = find_limit(
            gripper,
            direction=-1.0,  # 负方向 = 张开
            step_size=step_size,
            threshold_stall=args.threshold,
            timeout=args.timeout
        )

        # 计算结果
        motor_range = abs(open_pos - closed_pos)
        motor_min = min(closed_pos, open_pos)
        motor_max = max(closed_pos, open_pos)

        # 假设: closed = 0 (或最大), open = 最小 (负值)
        # 根据 follower_driver_node.py 的注释:
        # Zero = closed, negative = open
        # 所以 closed_pos 应该接近 0，open_pos 应该是负值

        # 输出结果
        print("\n\n" + "=" * 60)
        print("测量结果")
        print("=" * 60)
        print(f"\n电机位置 (rad):")
        print(f"  闭合位置: {closed_pos:.4f} rad")
        print(f"  张开位置: {open_pos:.4f} rad")
        print(f"  运动范围: {motor_range:.4f} rad")
        print(f"  最小值: {motor_min:.4f} rad")
        print(f"  最大值: {motor_max:.4f} rad")

        # 计算 prismatic 位移范围
        # 根据 follower_driver_node.py:
        # prismatic(m) = motor(rad) / -133.0
        # 或者: motor(rad) = prismatic(m) * -133.0
        prismatic_closed = closed_pos / -133.0
        prismatic_open = open_pos / -133.0
        prismatic_range = abs(prismatic_open - prismatic_closed)

        print(f"\nPrismatic 位移 (m):")
        print(f"  闭合位置: {prismatic_closed:.6f} m")
        print(f"  张开位置: {prismatic_open:.6f} m")
        print(f"  运动范围: {prismatic_range:.6f} m")

        # 计算主臂角度范围
        # 假设主臂夹爪: 0 rad (闭合) ~ 0.944 rad (张开)
        # 映射关系: follower_pos = sign * leader_pos + offset
        # 如果 offset = 0:
        #   sign = follower_range / leader_range
        #   sign = prismatic_range / 0.944
        leader_range = 0.944  # 主臂夹爪范围 (rad)
        estimated_sign = prismatic_range / leader_range

        print(f"\n主臂夹爪范围 (假设):")
        print(f"  闭合: 0.0 rad")
        print(f"  张开: {leader_range:.3f} rad")

        print(f"\n建议配置 (relay_real.yaml):")
        print(f"  gripper_finger_left_joint:")
        print(f"    sign: {estimated_sign:.6f}")
        print(f"    offset: 0.0")
        print(f"    leader_limit: [0, {leader_range:.3f}]")
        print(f"    follower_limit: [{min(prismatic_closed, prismatic_open):.6f}, {max(prismatic_closed, prismatic_open):.6f}]")

        # 验证
        print(f"\n验证:")
        print(f"  sign × leader_range = {estimated_sign:.6f} × {leader_range:.3f} = {estimated_sign * leader_range:.6f}")
        print(f"  实测 follower_range = {prismatic_range:.6f}")
        print(f"  误差 = {abs(estimated_sign * leader_range - prismatic_range):.6f}")

        print("\n" + "=" * 60)
        print("测量完成！")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n测量被中断")
    finally:
        # 清理
        print("\n正在关闭夹爪电机...")
        try:
            gripper.stop_control_loop()
            gripper.disable()
            gripper.disconnect()
            print("✓ 夹爪电机已关闭")
        except Exception as e:
            print(f"✗ 关闭失败: {e}")


if __name__ == '__main__':
    main()
