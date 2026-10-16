#!/usr/bin/env python3
"""夹爪电机测试脚本

测试夹爪的最大最小位置，遇到阻力立即停止。
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'Project' / 'reBotArm_control_py'))
sys.path.insert(0, str(Path.home() / 'Project' / 'reBotArm_control_py' / '.venv' / 'lib' / 'python3.10' / 'site-packages'))

from reBotArm_control_py.actuator.gripper import Gripper

# 配置
GRIPPER_CFG = str(Path.home() / 'Project' / 'rebotarm-ros2-stack' / 'src' / 'rebotarm_hw_driver' / 'config' / 'gripper_can.yaml')
TORQUE_THRESHOLD = 1.0  # Nm, 超过此值停止 (增大阈值)
POSITION_STEP = 0.2  # rad, 每步移动量 (增大步长)
POSITION_RANGE = (-3.0, 3.0)  # rad, 测试范围


def test_gripper():
    print("=== 夹爪电机测试 ===")
    print(f"配置: {GRIPPER_CFG}")
    print(f"测试顺序: 1.零点 → 2.张开到最大 → 3.归零")
    print()

    try:
        g = Gripper(cfg_path=GRIPPER_CFG)
        print("✓ Gripper 实例创建成功")

        print("使能电机...")
        if not g.enable():
            print("✗ 使能失败")
            return
        print("✓ 使能成功")

        g.mode_mit(kp=30.0, kd=3.0)
        print("✓ MIT 模式设置完成")

        # 步骤1: 当前初始点位为零点
        print("\n--- 步骤1: 当前初始点位为零点 ---")
        pos, vel, torq = g.get_state(request=True)
        print(f"当前位置: pos={pos:.4f} rad")
        print("保持当前位置为零点 (不调用 set_zero)")

        # 步骤2: 张开到最大点位
        print(f"\n--- 步骤2: 张开到最大点位 (目标: {POSITION_RANGE[0]} rad) ---")
        target = 0.0
        while target > POSITION_RANGE[0]:
            target -= POSITION_STEP
            g.mit(pos=target)
            time.sleep(0.1)  # 增加等待时间
            p, v, t = g.get_state(request=True)  # 主动请求反馈
            print(f"  target={target:.3f}, pos={p:.4f}, torq={t:.4f}", end="")
            if abs(t) > TORQUE_THRESHOLD:
                print(f" ← 阻力过大，停止")
                break
            else:
                print()
            time.sleep(0.1)

        max_open_pos = p
        print(f"最大张开位置: {max_open_pos:.4f} rad")

        # 步骤3: 缓慢归零
        print(f"\n--- 步骤3: 缓慢归零 ---")
        print("缓慢回到零点...")
        current_pos = max_open_pos
        while current_pos < -0.1:  # 逐渐回到零点
            current_pos += 0.05  # 每次移动 0.05 rad
            g.mit(pos=current_pos)
            time.sleep(0.1)
            p, v, t = g.get_state(request=True)
            print(f"  target={current_pos:.3f}, pos={p:.4f}")
            time.sleep(0.1)

        # 最终归零
        g.mit(pos=0.0)
        time.sleep(1.0)
        pos, vel, torq = g.get_state(request=True)
        print(f"归零后位置: pos={pos:.4f} rad")

        # 总结
        print(f"\n=== 测试结果 ===")
        print(f"零点位置: 0.0 rad")
        print(f"最大张开位置: {max_open_pos:.4f} rad")
        print(f"总行程: {abs(max_open_pos):.4f} rad")

        # 禁用
        print("\n禁用电机...")
        g.disable()
        g.disconnect()
        print("✓ 测试完成")

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_gripper()
