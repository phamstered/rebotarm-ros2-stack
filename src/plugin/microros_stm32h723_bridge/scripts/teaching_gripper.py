#!/usr/bin/env python3
"""Read-only manual teaching readiness check; legacy electric actions fail closed."""
import argparse
import json
import math
import time
import uuid

from teaching_core import JOINT, controller_error, fresh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['move', 'cancel', 'check', 'check-manual'])
    parser.add_argument('--command-id', type=uuid.UUID)
    parser.add_argument('--position', type=float)
    args = parser.parse_args()
    if args.operation in ('move', 'cancel'):
        parser.error('手动示教不支持电动夹爪操作；请退出示教后使用 Trajectory 模式')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from microros_stm32h723_bridge.msg import HardwareStatus

    rclpy.init()
    node = rclpy.create_node('teaching_gripper_' + uuid.uuid4().hex[:12])
    latest = {}
    def store(key, msg):
        latest[key] = (msg, time.monotonic())
    for name, topic, typ in [('state', '/robot_feedback/joint_states', JointState),
                             ('status', '/stm32/status', HardwareStatus)]:
        node.create_subscription(typ, topic, lambda msg, key=name: store(key, msg), qos_profile_sensor_data)
    def await_future(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError('ROS 请求超时')
        return future.result()
    def call(path, typ, request):
        client = node.create_client(typ, path)
        try:
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError(f'服务未就绪：{path}')
            return await_future(client.call_async(request), 2)
        finally:
            node.destroy_client(client)
    def feedback():
        now = time.monotonic()
        ros_now = node.get_clock().now().nanoseconds
        for key in ('state', 'status'):
            if key not in latest:
                raise RuntimeError(f'等待新鲜反馈：{key}')
            msg, received = latest[key]
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if not fresh(received, stamp, now, ros_now):
                raise RuntimeError(f'反馈过期或时钟无效：{key}')
        status = latest['status'][0]
        if status.fault_code != 0 or status.mode != 1 or status.active_joint_mask != 127:
            raise RuntimeError('示教需要七轴使能、无故障 TRACK')
        state = latest['state'][0]
        index = state.name.index(JOINT)
        q, v = state.position[index], state.velocity[index]
        if not math.isfinite(q) or not math.isfinite(v) or not -0.001 <= q <= 0.051:
            raise RuntimeError('夹爪标定反馈无效')
        return q, v
    def controllers():
        response = call('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
        error = controller_error(response.controller)
        if error:
            raise RuntimeError(error)
    try:
        request = GetParameters.Request(names=['drag_teaching', 'manual_teaching_gripper'])
        values = call('/microros_stm32h723_hardware/get_parameters', GetParameters, request).values
        if len(values) != 2 or not all(value.type == 1 and value.bool_value for value in values):
            raise RuntimeError('硬件未启用经过增益校验的拖动示教模式，请升级并重新启动控制栈')
        controllers()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and len(latest) < 2:
            rclpy.spin_once(node, timeout_sec=0.02)
        feedback()
        print(json.dumps({'ok': True, 'message': '六轴重力补偿与手动夹爪已就绪'}, ensure_ascii=False))
    except Exception as error:
        print(json.dumps({'ok': False, 'message': str(error)}, ensure_ascii=False))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
