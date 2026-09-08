#!/usr/bin/env python3
"""Run the shared autonomy engine with ROS 2 transport, diagnostic wheels only."""
import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from duck2_core import AutonomyCore
from runtime import Runtime
import lifecycle


def main():
    # OpenCV's host-sized worker pool can starve ROS callbacks in a container.
    cv2.setNumThreads(1)
    lifecycle.init()
    node = Node("duck2_autonomy")
    runtime = Runtime(node, os.environ.get("VEHICLE_NAME", "duck2"))
    os.environ.setdefault("VEHICLE_NAME", runtime.vehicle)
    core = AutonomyCore("duck2_autonomy", runtime)
    # Perception is serialized here. Expiry and direct Stop run in the separate
    # supervisor process, so a blocked OpenCV callback cannot block those paths.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lifecycle.begin_cleanup()
        if rclpy.ok():
            core.on_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
