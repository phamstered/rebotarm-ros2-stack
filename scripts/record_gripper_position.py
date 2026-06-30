#!/usr/bin/env python3
"""记录主臂夹爪位置信息 (直接使用 Python SDK)

持续读取并打印 servo 6 (joint7_left) 的位置，方便用户手动移动夹爪时记录位置范围。
"""
import sys
import time
import signal

sys.path.insert(0, '/home/jialiang/Project/Star-Arm-102/Python_SDK/.venv/lib/python3.12/site-packages')
import fashionstar_uart_sdk as uservo
import serial

SERVO_PORT = '/dev/ttyCH341USB0'
SERVO_BAUDRATE = 1000000
GRIPPER_SERVO_ID = 6


def main():
    uart = serial.Serial(port=SERVO_PORT, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=0)
    control = uservo.UartServoManager(uart)

    print("=== 主臂夹爪位置记录 ===")
    print("请手动移动夹爪，Ctrl+C 停止")
    print("-" * 50)

    min_pos = float('inf')
    max_pos = float('-inf')
    count = 0

    running = True
    def on_sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_sigint)

    while running:
        control.send_sync_servo_monitor([GRIPPER_SERVO_ID])
        pos = control.servos[GRIPPER_SERVO_ID].angle_monitor
        pos_deg = pos
        min_pos = min(min_pos, pos)
        max_pos = max(max_pos, pos)
        count += 1
        if count % 10 == 0:
            print(f"servo6: {pos:.2f} deg  range: [{min_pos:.2f}, {max_pos:.2f}] deg")
        time.sleep(0.02)

    print("\n" + "=" * 50)
    print(f"位置范围: [{min_pos:.2f}, {max_pos:.2f}] deg")
    print(f"总行程: {max_pos - min_pos:.2f} deg")
    uart.close()


if __name__ == '__main__':
    main()
