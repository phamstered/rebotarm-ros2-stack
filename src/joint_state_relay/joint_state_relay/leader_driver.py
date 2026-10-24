"""Run the installed Star Arm driver with normal ROS signal shutdown handling.

The vendor SDK, node, serial configuration and joint mapping remain external.
"""
from .lifecycle import spin_node


def main(args=None):
    from robo_driver.robo_driver import Arm_contorl
    spin_node(Arm_contorl, args=args, before_destroy=lambda node: node.node_close())


if __name__ == '__main__':
    main()
