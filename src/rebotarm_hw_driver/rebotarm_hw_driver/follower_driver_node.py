import signal
import sys
import os
import time
import threading
from pathlib import Path

_project_root = str(Path.home() / 'Project' / 'reBotArm_control_py')
_venv_site = str(Path.home() / 'Project' / 'reBotArm_control_py' / '.venv' / 'lib' / 'python3.10' / 'site-packages')
_venv_pinocchio = os.path.join(_venv_site, 'cmeel.prefix', 'lib', 'python3.10', 'site-packages')

for p in [_project_root, _venv_site, _venv_pinocchio]:
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from reBotArm_control_py.actuator.arm import RobotArm
from reBotArm_control_py.actuator.gripper import Gripper
from reBotArm_control_py.dynamics import compute_generalized_gravity

_JOINT4_GRAVITY_SCALE = 0.85
_ZERO = np.zeros(6)
_RETURN_DURATION = 2.0
_ZERO_THRESHOLD = 0.015

# joint1-3 (大电机)
_KP_RETURN_123 = 30.0
_KD_RETURN_123 = 3.0
_KP_HOLD_123 = 20.0
_KD_HOLD_123 = 2.0
_KP_TRACK_123 = 40.0
_KD_TRACK_123 = 6.0

# joint4-6 (小电机，降低增益减少振荡)
_KP_RETURN_456 = 15.0
_KD_RETURN_456 = 2.0
_KP_HOLD_456 = 10.0
_KD_HOLD_456 = 1.5
_KP_TRACK_456 = 20.0
_KD_TRACK_456 = 3.0

# 通用
_KP_IDLE = 0.0
_KD_IDLE = 1.0
_CMD_TIMEOUT = 0.1

# Gripper: prismatic displacement (m) → motor position (rad)
# Zero = closed, negative = open
# Main arm: 0 deg (closed) ~ 54.10 deg (0.944 rad, open)
# Motor: 0 rad (closed) ~ -2.66 rad (open)
# Prismatic: 0 m (closed) ~ 0.02 m (open)
# Scale: -2.66 / 0.02 = -133
_GRIPPER_POS_SCALE = -133.0
_GRIPPER_KP = 6.0   # 降低 kp 减少振荡
_GRIPPER_KD = 1.5   # 降低 kd
_GRIPPER_EMA_ALPHA = 0.5  # 滤波系数 (可被 ROS2 参数覆盖)
_GRIPPER_DEADBAND = 0.0005  # 死区 (m)


class _FollowController:
    _EMA_ALPHA = 0.5  # low-pass filter coefficient for target smoothing

    def __init__(self):
        self._tracking = False
        self._target_buf = [None, None]
        self._write_idx = 0
        self._lock = threading.Lock()
        self._last_cmd_time = time.monotonic()

        self._returning = False
        self._start_pos = None
        self._return_elapsed = 0.0
        self._return_done = False
        self._filtered_target = None

    def set_target(self, positions: np.ndarray):
        with self._lock:
            self._target_buf[self._write_idx] = positions.copy()
            self._write_idx = 1 - self._write_idx
            self._tracking = True
            self._last_cmd_time = time.monotonic()

    def clear_target(self):
        self._tracking = False

    def start_return(self, current_pos: np.ndarray):
        self._returning = True
        self._start_pos = current_pos.copy()
        self._return_elapsed = 0.0
        self._return_done = False

    @property
    def return_done(self) -> bool:
        return self._return_done

    def __call__(self, arm: RobotArm, dt: float) -> None:
        q = arm.get_positions()
        tau_g = compute_generalized_gravity(q=q)
        tau_g[3] *= _JOINT4_GRAVITY_SCALE

        # 按关节设置不同的增益 (joint1-3 大电机, joint4-6 小电机)
        kp = np.zeros(6)
        kd = np.zeros(6)

        if self._returning:
            self._return_elapsed += dt
            t = min(self._return_elapsed / _RETURN_DURATION, 1.0)
            alpha = 0.5 * (1.0 - np.cos(np.pi * t))
            target = self._start_pos * (1.0 - alpha)
            kp[:3] = _KP_RETURN_123
            kp[3:] = _KP_RETURN_456
            kd[:3] = _KD_RETURN_123
            kd[3:] = _KD_RETURN_456
            if t >= 1.0:
                self._return_done = True
                kp[:3] = _KP_HOLD_123
                kp[3:] = _KP_HOLD_456
                kd[:3] = _KD_HOLD_123
                kd[3:] = _KD_HOLD_456
        elif self._tracking:
            if time.monotonic() - self._last_cmd_time > _CMD_TIMEOUT:
                self._tracking = False
                target = q
                kp[:] = _KP_IDLE
                kd[:] = _KD_IDLE
            else:
                with self._lock:
                    target = self._target_buf[1 - self._write_idx]
                if target is None:
                    target = q
                # low-pass filter to smooth sudden position jumps
                if self._filtered_target is None:
                    self._filtered_target = target.copy()
                else:
                    self._filtered_target = (self._EMA_ALPHA * target
                                             + (1.0 - self._EMA_ALPHA) * self._filtered_target)
                target = self._filtered_target
                kp[:3] = _KP_TRACK_123
                kp[3:] = _KP_TRACK_456
                kd[:3] = _KD_TRACK_123
                kd[3:] = _KD_TRACK_456
        else:
            target = q
            kp[:] = _KP_IDLE
            kd[:] = _KD_IDLE
            self._filtered_target = None  # reset filter

        arm.mit(pos=target, vel=_ZERO, kp=kp, kd=kd, tau=tau_g,
                request_feedback=True)


class FollowerDriverNode(Node):
    def __init__(self):
        super().__init__('rebotarm_follower_driver')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('config_path', '')
        self.declare_parameter('enable_gripper', True)
        self.declare_parameter('gripper_ema_alpha', _GRIPPER_EMA_ALPHA)

        rate = self.get_parameter('rate').value
        config_path = self.get_parameter('config_path').value
        enable_gripper = self.get_parameter('enable_gripper').value
        self._gripper_ema_alpha = float(self.get_parameter('gripper_ema_alpha').value)

        kwargs = {}
        if config_path:
            kwargs['cfg_path'] = config_path
        self.arm = RobotArm(**kwargs)
        self.arm.connect()

        self.get_logger().info('Enabling motors...')
        try:
            self.arm.enable()
        except Exception as e:
            self.get_logger().error(f'Motor enable failed: {e}')
            self.get_logger().error('Check: ip -s link show can0')
            self.get_logger().error('Check: sudo ip link set can0 up type can bitrate 1000000')
            raise

        self.arm.mode_mit(kp=_ZERO, kd=np.full(self.arm.num_joints, 1.0))
        self.get_logger().info('MIT mode activated')
        self._ctrl = _FollowController()
        self.arm.start_control_loop(self._ctrl, rate=self.arm._rate)
        self._shutdown_done = False
        self._shutdown_complete = False

        self.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6']

        # Gripper
        self._gripper = None
        self._gripper_pos = 0.0
        self._gripper_filtered_pos = 0.0
        self._gripper_target = 0.0
        if enable_gripper:
            try:
                gripper_cfg = str(Path.home() / 'Project' / 'leader_follower' / 'src' / 'rebotarm_hw_driver' / 'config' / 'gripper_can.yaml')
                self._gripper = Gripper(cfg_path=gripper_cfg)
                self._gripper.enable()
                self._gripper.mode_mit(kp=_GRIPPER_KP, kd=_GRIPPER_KD)
                self.get_logger().info(f'Gripper enabled (kp={_GRIPPER_KP}, kd={_GRIPPER_KD}, ema_alpha={self._gripper_ema_alpha})')
                self.joint_names.append('gripper_finger_left_joint')
            except Exception as e:
                self.get_logger().warn(f'Gripper init failed: {e}')

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self._publish)

        self.create_subscription(
            JointState, '/joint_commands', self._command_cb, 10)

        self.get_logger().info(
            f'Follower driver started at {rate} Hz '
            f'(gravity comp @ {self.arm._rate} Hz, '
            f'gripper={"enabled" if self._gripper else "disabled"})')

    def _command_cb(self, msg: JointState):
        positions = np.zeros(6)
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                if name == 'gripper_finger_left_joint':
                    # Gripper: convert prismatic displacement (m) to motor position (rad)
                    if self._gripper is not None:
                        prismatic_pos = msg.position[i]

                        # 死区：变化太小则忽略
                        if abs(prismatic_pos - self._gripper_target) < _GRIPPER_DEADBAND:
                            continue

                        self._gripper_target = prismatic_pos

                        # EMA 滤波
                        self._gripper_filtered_pos = (
                            self._gripper_ema_alpha * prismatic_pos +
                            (1.0 - self._gripper_ema_alpha) * self._gripper_filtered_pos
                        )

                        motor_pos = self._gripper_filtered_pos * _GRIPPER_POS_SCALE
                        self._gripper_pos = motor_pos
                        self._gripper.mit(pos=motor_pos)
                else:
                    idx = self.joint_names.index(name)
                    positions[idx] = msg.position[i]
        self._ctrl.set_target(positions)

    def _publish(self):
        try:
            p, v, t = self.arm.get_positions(), self.arm.get_velocities(), self.arm.get_torques()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            if self._gripper is not None:
                # Convert motor position (rad) back to prismatic displacement (m)
                motor_pos = self._gripper.get_position(request=False)
                motor_vel = self._gripper.get_velocity(request=False)
                motor_torq = self._gripper.get_torque(request=False)
                gripper_pos = motor_pos / _GRIPPER_POS_SCALE
                gripper_vel = motor_vel / _GRIPPER_POS_SCALE

                msg.name = self.joint_names
                msg.position = p.tolist() + [gripper_pos]
                msg.velocity = v.tolist() + [gripper_vel]
                msg.effort = t.tolist() + [motor_torq]
            else:
                msg.name = self.joint_names
                msg.position = p.tolist()
                msg.velocity = v.tolist()
                msg.effort = t.tolist()

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Publish failed: {e}')

    def _check_zero(self):
        pos = self.arm.get_positions()
        return [(self.joint_names[i], pos[i])
                for i in range(self.arm.num_joints)
                if abs(pos[i]) > _ZERO_THRESHOLD]

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._ctrl.clear_target()
        self.get_logger().info('Returning to zero...')
        self._ctrl.start_return(self.arm.get_positions())

        deadline = time.monotonic() + _RETURN_DURATION + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.3)
            if self._ctrl.return_done:
                time.sleep(0.5)
                break

        not_zero = self._check_zero()
        if not not_zero:
            self.get_logger().info('All joints at zero, disabling motors...')
            self.arm.disable()
            self.arm.disconnect()
        else:
            self.get_logger().warn('=== 以下关节未归零，请手动归零 ===')
            for name, pos in not_zero:
                self.get_logger().warn(
                    f'  {name}: {pos:+.3f} rad ({np.degrees(pos):+.1f}°)')
            self.arm.disable()

        # Shutdown gripper
        if self._gripper is not None:
            self.get_logger().info('Disabling gripper...')
            try:
                self._gripper.disable()
                self._gripper.disconnect()
            except Exception as e:
                self.get_logger().warn(f'Gripper shutdown error: {e}')

        self._shutdown_complete = True


def main(args=None):
    rclpy.init(args=args)
    node = FollowerDriverNode()
    _shutdown_triggered = threading.Event()

    def _on_sigint(signum, frame):
        if _shutdown_triggered.is_set():
            node.get_logger().warn('Force exit!')
            node.arm.disconnect()
            sys.exit(1)
        _shutdown_triggered.set()
        node.get_logger().info('Ctrl+C: returning to zero... (再按 Ctrl+C 强制退出)')
        threading.Thread(target=node.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._shutdown_complete:
                break
    except Exception:
        pass


if __name__ == '__main__':
    main()
