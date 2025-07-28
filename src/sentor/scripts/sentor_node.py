#!/usr/bin/env python3
import rclpy
import yaml
import time
import signal
import os
from sentor.TopicMonitor import TopicMonitor
from sentor.TopicMonitor import TopicMonitor
from sentor.MultiMonitor import MultiMonitor
from std_msgs.msg import String
from sentor_msgs.msg import SentorEvent
from std_srvs.srv import Empty

topic_monitors = []
event_pub = None
rich_event_pub = None

def __signal_handler(signum, frame):
    """ Gracefully stop all monitors on SIGINT. """
    for topic_monitor in topic_monitors:
        topic_monitor.kill_monitor()
    multi_monitor.stop_monitor()
    print("stopped.")
    os._exit(signal.SIGTERM)

def stop_monitoring(_):
    """ Stop all monitoring activities. """
    for topic_monitor in topic_monitors:
        topic_monitor.stop_monitor()
    multi_monitor.stop_monitor()
    return Empty.Response()

def start_monitoring(_):    
    """ Start monitoring activities. """
    for topic_monitor in topic_monitors:
        topic_monitor.start_monitor()
    multi_monitor.start_monitor()
    return Empty.Response()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rclpy.init()
    node = rclpy.create_node("sentor")

    # 🔹 Load Configuration
    config_file = node.declare_parameter("config_file", "").value
    topics = []

    try:
        items = [yaml.safe_load(open(item, 'r')) for item in config_file.split(',')]
        topics = [item for sublist in items for item in sublist]
    except Exception as e:
        node.get_logger().error(f"No configuration file provided: {e}")

    stop_srv = node.create_service(Empty, '/sentor/stop_monitor', stop_monitoring)
    start_srv = node.create_service(Empty, '/sentor/start_monitor', start_monitoring)

    event_pub = node.create_publisher(String, '/sentor/event', 10)
    rich_event_pub = node.create_publisher(SentorEvent, '/sentor/rich_event', 10)

    multi_monitor = MultiMonitor()  # 🔹 MultiMonitor no longer creates monitors!

    topic_monitors = []
    print("Monitoring topics:")
    for i, topic in enumerate(topics):
        if topic.get("include", True) is False:
            continue  # Skip topics marked as "include: False"

        try:
            topic_name = topic["name"]
        except KeyError:
            node.get_logger().error(f"Topic name not specified for entry: {topic}")
            continue

        # Extract parameters with defaults
        topic_monitor = TopicMonitor(
            topic_name=topic_name,
            msg_type=multi_monitor.get_message_type(topic["message_type"]),
            qos_profile=multi_monitor.get_qos_profile(topic.get("qos", {})),
            rate=topic.get("rate", 0),
            N=topic.get("N", 0),
            signal_when=topic.get("signal_when", {}),
            signal_lambdas=topic.get("signal_lambdas", []),
            processes=topic.get("execute", []),
            timeout=topic.get("timeout", 0),
            default_notifications=topic.get("default_notifications", True),
            event_callback=None,
            thread_num=i,
        )

        topic_monitors.append(topic_monitor)
        multi_monitor.register_monitors(topic_monitor)  # 🔹 We now pass monitors to MultiMonitor

    time.sleep(1)

    # Start monitoring
    for topic_monitor in topic_monitors:
        topic_monitor.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
