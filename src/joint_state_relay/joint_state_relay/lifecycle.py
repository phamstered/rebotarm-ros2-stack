"""ROS process lifecycle shared by the relay and the vendor leader adapter."""


def spin_node(factory, args=None, before_destroy=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy._rclpy_pybind11 import RCLError

    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM may shut the context down before spin returns.
        # Do not publish a rosout error through that already-closed context.
        pass
    except RCLError:
        # A signal can close the context just before spin builds its wait set.
        # Preserve native ROS failures while the context is still valid.
        if rclpy.ok():
            raise
    finally:
        try:
            if node is not None and before_destroy is not None:
                before_destroy(node)
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown()
