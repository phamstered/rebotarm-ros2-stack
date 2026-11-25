"""Run the external Star Arm node with validated, complete monitor frames."""
import math
from numbers import Real

from .lifecycle import spin_node


def guarded_driver(vendor_class):
    """Keep vendor configuration/conversion, replacing only feedback assembly."""
    class LeaderDriver(vendor_class):
        def publish_current_angle(self):
            from sensor_msgs.msg import JointState

            servo = self.Servo
            manager = servo.uservo
            # The SDK leaves cached angles intact when a monitor reply is lost.
            # Invalidate before each request so silence cannot look fresh.
            for servo_id in servo.servo_ids:
                info = manager.servos.get(servo_id)
                if info is not None:
                    info.angle_monitor = None
            manager.send_sync_servo_monitor(servo.servo_ids)

            angles = []
            invalid = []
            for servo_id in servo.servo_ids:
                info = manager.servos.get(servo_id)
                angle = getattr(info, 'angle_monitor', None)
                if isinstance(angle, bool) or not isinstance(angle, Real) or not math.isfinite(angle):
                    invalid.append(servo_id)
                angles.append(angle)
            if invalid:
                self.get_logger().warning(
                    f'Leader feedback dropped: missing/invalid monitor angle for servo IDs {invalid}; '
                    'check leader power, serial connection and servo discovery.',
                    throttle_duration_sec=2.0,
                )
                return

            positions = [servo.servoangle2jointstate(servo_id=servo_id, servo_angle=angle)
                         for servo_id, angle in zip(servo.servo_ids, angles)]
            if not all(isinstance(value, Real) and not isinstance(value, bool)
                       and math.isfinite(value) for value in positions):
                raise ValueError('Leader servo conversion returned non-finite joint positions')
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [servo.JOINT_[index] for index in range(servo.SRV_NUM)]
            message.position = positions
            message.velocity = []
            message.effort = []
            self.current_angle = angles
            self.joint_states_publisher.publish(message)

    return LeaderDriver


def main(args=None):
    from robo_driver.robo_driver import Arm_contorl
    spin_node(guarded_driver(Arm_contorl), args=args,
              before_destroy=lambda node: node.node_close())


if __name__ == '__main__':
    main()
