#!/usr/bin/env python3
"""主臂夹爪测试脚本 (直接使用 Python SDK)

测试顺序: 1.零点 → 2.张开到最大 → 3.缓慢归零
"""
import sys
import time
import struct
import serial

# 添加 SDK 路径
sys.path.insert(0, str('/home/jialiang/Project/Star-Arm-102/Python_SDK'))
import fashionstar_uart_sdk as uservo

# 配置
SERVO_BAUDRATE = 1000000
LEADER_PORT_NAME = "/dev/ttyCH341USB0"  # Jetson 上的串口
GRIPPER_SERVO_ID = 6
GRIPPER_ZERO = 500.0  # 夹爪零点角度 (robo_driver 中定义)
POSITION_RANGE = (GRIPPER_ZERO, GRIPPER_ZERO + 90.0)  # 度, 500=闭合, 590=张开
POSITION_STEP = 5.0  # 度, 每步移动量


def test_leader_gripper():
    print("=== 主臂夹爪测试 (Python SDK) ===")
    print(f"测试顺序: 1.零点 → 2.张开到最大 → 3.缓慢归零")
    print(f"串口: {LEADER_PORT_NAME}")
    print()

    try:
        # 初始化串口
        leader_uart = serial.Serial(
            port=LEADER_PORT_NAME,
            baudrate=SERVO_BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0
        )
        leader_control = uservo.UartServoManager(leader_uart)
        print("✓ 串口初始化成功")

        # 不调用 stop/reset，保持舵机当前状态

        # 读取所有舵机当前位置
        leader_control.send_sync_servo_monitor([0, 1, 2, 3, 4, 5, 6])
        current_angles = [leader_control.servos[i].angle_monitor for i in range(7)]
        gripper_start = current_angles[GRIPPER_SERVO_ID]
        print(f"\n当前位置:")
        for i in range(7):
            print(f"  servo {i}: {current_angles[i]:.2f} 度")

        # 步骤1: 当前位置为零点
        print(f"\n--- 步骤1: 当前位置为零点 ---")
        print(f"夹爪零点: {gripper_start:.2f} 度")

        # 步骤2: 张开到最大点位
        max_target = gripper_start + 90.0  # 张开 90 度
        print(f"\n--- 步骤2: 张开到最大点位 (目标: {max_target:.2f} 度) ---")
        target = gripper_start
        while target < max_target:
            target += POSITION_STEP
            # 只控制夹爪，其他舵机保持当前位置
            leader_control.send_sync_servo_monitor([0, 1, 2, 3, 4, 5, 6])
            current_angles = [leader_control.servos[i].angle_monitor for i in range(7)]
            command_data_list = []
            for i in range(7):
                if i == GRIPPER_SERVO_ID:
                    command_data_list.append(struct.pack("<BlLHHH", i, int(target * 10), 100, 50, 50, 0))
                else:
                    command_data_list.append(struct.pack("<BlLHHH", i, int(current_angles[i] * 10), 100, 50, 50, 0))
            leader_control.send_sync_multiturnanglebyinterval(14, 7, command_data_list)
            time.sleep(0.1)
            # 读取当前位置
            leader_control.send_sync_servo_monitor([GRIPPER_SERVO_ID])
            actual_angle = leader_control.servos[GRIPPER_SERVO_ID].angle_monitor
            print(f"  target={target:.2f}, actual={actual_angle:.2f}")
            time.sleep(0.1)

        max_open_pos = target
        print(f"最大张开位置: {max_open_pos:.2f} 度")

        # 步骤3: 缓慢归零
        print(f"\n--- 步骤3: 缓慢归零 ---")
        print("缓慢回到零点...")
        current_pos = max_open_pos
        while current_pos > gripper_start + 5.0:
            current_pos -= 2.0
            leader_control.send_sync_servo_monitor([0, 1, 2, 3, 4, 5, 6])
            current_angles = [leader_control.servos[i].angle_monitor for i in range(7)]
            command_data_list = []
            for i in range(7):
                if i == GRIPPER_SERVO_ID:
                    command_data_list.append(struct.pack("<BlLHHH", i, int(current_pos * 10), 100, 50, 50, 0))
                else:
                    command_data_list.append(struct.pack("<BlLHHH", i, int(current_angles[i] * 10), 100, 50, 50, 0))
            leader_control.send_sync_multiturnanglebyinterval(14, 7, command_data_list)
            time.sleep(0.1)
            leader_control.send_sync_servo_monitor([GRIPPER_SERVO_ID])
            actual_angle = leader_control.servos[GRIPPER_SERVO_ID].angle_monitor
            print(f"  target={current_pos:.2f}, actual={actual_angle:.2f}")
            time.sleep(0.1)

        # 最终归零
        leader_control.send_sync_servo_monitor([0, 1, 2, 3, 4, 5, 6])
        current_angles = [leader_control.servos[i].angle_monitor for i in range(7)]
        command_data_list = []
        for i in range(7):
            if i == GRIPPER_SERVO_ID:
                command_data_list.append(struct.pack("<BlLHHH", i, int(gripper_start * 10), 1500, 750, 750, 8000))
            else:
                command_data_list.append(struct.pack("<BlLHHH", i, int(current_angles[i] * 10), 1500, 750, 750, 8000))
        leader_control.send_sync_multiturnanglebyinterval(14, 7, command_data_list)
        time.sleep(2.0)
        leader_control.send_sync_servo_monitor([GRIPPER_SERVO_ID])
        final_angle = leader_control.servos[GRIPPER_SERVO_ID].angle_monitor
        print(f"归零后位置: {final_angle:.2f} 度")

        # 总结
        print(f"\n=== 测试结果 ===")
        print(f"零点位置: {gripper_start:.2f} 度")
        print(f"最大张开位置: {max_open_pos:.2f} 度")
        print(f"总行程: {max_open_pos - gripper_start:.2f} 度")

        # 关闭串口
        leader_uart.close()
        print("\n✓ 测试完成")

    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_leader_gripper()
