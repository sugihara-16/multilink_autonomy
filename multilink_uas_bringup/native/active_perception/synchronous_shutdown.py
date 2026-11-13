"""Finish a ROS callback before destroying its subscription or context."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
_stopping=False

def initialize():
    global _stopping
    _stopping=False
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def stop(signum,frame):
        global _stopping
        _stopping=True
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)

def spin(node):
    executor=rclpy.get_global_executor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not _stopping:executor.spin_once(timeout_sec=.1)
    finally:executor.remove_node(node)
