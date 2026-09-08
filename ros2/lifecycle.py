"""Keep ROS context alive until explicit zero publication and node cleanup."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions


def init():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def stop(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)


def begin_cleanup():
    # ros2 launch may forward a signal already delivered to the process group.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
