#!/usr/bin/env python3
"""Same HTTP desktop contract and acknowledgments, ROS 2 transport."""
import os
import threading
from http.server import ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from command_gateway import Gateway, handler_for
from runtime import Runtime
import lifecycle


def main():
    lifecycle.init()
    node = Node("duck2_command_gateway")
    runtime = Runtime(node, os.environ.get("VEHICLE_NAME", "duck2"))
    host = runtime.get_param("~listen_host", "127.0.0.1")
    port = runtime.get_param("~port", 8765)
    token = os.environ.get("DUCK2_CONTROL_TOKEN", "")
    if host not in ("127.0.0.1", "localhost") and not token:
        raise ValueError("Set DUCK2_CONTROL_TOKEN for a non-loopback gateway")
    gateway = Gateway(runtime.vehicle, runtime)
    server = ThreadingHTTPServer((host, port), handler_for(gateway, token))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lifecycle.begin_cleanup()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
