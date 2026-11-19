"""ROS-independent admission rules for manual drag teaching."""
JOINT = 'gripper_finger_left_joint'
def controller_error(controllers):
    active = {c.name: c for c in controllers if c.state == 'active'}
    if 'gravity_compensation_controller' not in active:
        return '重力补偿未激活'
    for name, controller in active.items():
        if name in {'arm_teleop_controller', 'arm_trajectory_controller', 'vla_position_controller',
                    'joint4_calibration_controller', 'gripper_teleop_controller', 'gripper_trajectory_controller'}:
            return f'示教模式存在冲突控制器：{name}'
        if any(interface.startswith(JOINT + '/') for interface in controller.claimed_interfaces):
            return '手动示教夹爪命令接口被占用'
        if any(interface == f'joint{i}/{kind}' for interface in controller.claimed_interfaces
               for i in range(1, 7) for kind in ('position', 'velocity')):
            return '六轴位置或速度接口被占用'
    return None


def fresh(received, stamp_ns, now, ros_now_ns):
    return (now - received <= 0.5 and stamp_ns > 0 and
            -50_000_000 <= ros_now_ns - stamp_ns < 500_000_000)
