#!/usr/bin/env python3
"""Bridge leader /joint_states from ROS_DOMAIN_ID=0 to ROS_DOMAIN_ID=1."""
import multiprocessing
import os


def subscriber_main(queue):
    os.environ['ROS_DOMAIN_ID'] = '0'
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('domain0_sub')
    node.create_subscription(JointState, '/joint_states', lambda m: queue.put(m), 10)
    node.get_logger().info('Bridge sub: /joint_states (domain 0)')
    rclpy.spin(node)


def publisher_main(queue):
    os.environ['ROS_DOMAIN_ID'] = '1'
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('domain1_pub')
    pub = node.create_publisher(JointState, '/leader/joint_states', 10)
    node.get_logger().info('Bridge pub: /leader/joint_states (domain 1)')

    while rclpy.ok():
        try:
            msg = queue.get(timeout=0.1)
            pub.publish(msg)
        except Exception:
            pass
        rclpy.spin_once(node, timeout_sec=0.005)


def main():
    multiprocessing.set_start_method('spawn')
    q = multiprocessing.Queue(maxsize=10)

    procs = [
        multiprocessing.Process(target=subscriber_main, args=(q,), daemon=True),
        multiprocessing.Process(target=publisher_main, args=(q,), daemon=True),
    ]

    for p in procs:
        p.start()

    try:
        for p in procs:
            p.join()
    except KeyboardInterrupt:
        for p in procs:
            p.terminate()


if __name__ == '__main__':
    main()
