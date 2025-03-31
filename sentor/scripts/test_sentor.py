#!/usr/bin/env python3
import rclpy
import yaml
import time
import signal
import os
from sentor.TopicMonitor import TopicMonitor
from sentor.MultiMonitor import MultiMonitor
from std_msgs.msg import String
from sentor_msgs.msg import SentorEvent
from std_srvs.srv import Empty
import importlib
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor


topic_monitors = []
event_pub = None
rich_event_pub = None
multi_monitor = None

def __signal_handler(signum, frame):
    """ Gracefully stop all monitors on SIGINT. """
    for topic_monitor in topic_monitors:
        topic_monitor.kill_monitor()
    multi_monitor.stop_monitor()
    print("Stopped monitoring.")
    os._exit(signal.SIGTERM)

# def stop_monitoring(_):
#     """ Stop all monitoring activities. """
#     for topic_monitor in topic_monitors:
#         topic_monitor.stop_monitor()
#     multi_monitor.stop_monitor()
#     return Empty.Response()

# def start_monitoring(_):    
#     """ Start monitoring activities. """
#     for topic_monitor in topic_monitors:
#         topic_monitor.start_monitor()
#     multi_monitor.start_monitor()
#     return Empty.Response()

def start_monitoring(request, _):
    print("[Service] Received start_monitoring request")
    for topic_monitor in topic_monitors:
        topic_monitor.start_monitor()
    multi_monitor.start_monitor()
    return Empty.Response()

def stop_monitoring(request, _):
    print("[Service] Received stop_monitoring request")
    for topic_monitor in topic_monitors:
        topic_monitor.stop_monitor()
    multi_monitor.stop_monitor()
    return Empty.Response()

def resolve_qos(topic, multi_monitor):
    return get_qos_profile(topic["qos"]) if "qos" in topic else None

def resolve_msg_type(topic, multi_monitor):
    return get_message_type(topic["message_type"]) if "message_type" in topic else None

def get_message_type(type_str):
    """Import the actual message type class from a string like 'std_msgs/msg/Int32'."""
    if not type_str:
        return None
    try:
        package, msg_name = type_str.split('/msg/')
        module = importlib.import_module(f"{package}.msg")
        return getattr(module, msg_name)
    except Exception as e:
        print(f"[ERROR] Failed to import message type '{type_str}': {e}")
        return None
def get_qos_profile(qos_dict):
    """Convert YAML QoS dict into actual QoSProfile object."""
    reliability = ReliabilityPolicy.RELIABLE
    durability = DurabilityPolicy.VOLATILE
    history = HistoryPolicy.KEEP_LAST
    depth = 10

    if qos_dict.get("reliability", "").lower() == "best_effort":
        reliability = ReliabilityPolicy.BEST_EFFORT
    if qos_dict.get("durability", "").lower() == "transient_local":
        durability = DurabilityPolicy.TRANSIENT_LOCAL
    if qos_dict.get("history", "").lower() == "keep_all":
        history = HistoryPolicy.KEEP_ALL
    if "depth" in qos_dict:
        depth = qos_dict["depth"]

    return QoSProfile(
        reliability=reliability,
        durability=durability,
        history=history,
        depth=depth
    )

def event_callback(message, level="info", msg=None, nodes=None, topic_name=None):
    prefix = f"[{level.upper()}]"
    print(f"{prefix} {message}")

def main():
    global multi_monitor
    rclpy.init()
    node = rclpy.create_node("test_sentor")

    node.get_logger().info("Registering start/stop monitoring services...")
    node.create_service(Empty, "start_monitoring", start_monitoring)
    node.create_service(Empty, "stop_monitoring", stop_monitoring)

    config_file = node.declare_parameter("config_file", "config/test_monitor_config.yaml").value
    topics = []

    try:
        items = [yaml.safe_load(open(item, 'r')) for item in config_file.split(',')]
        for item in items:
            if isinstance(item, dict) and "monitors" in item:
                topics.extend(item["monitors"])
            elif isinstance(item, list):
                topics.extend(item)
            elif isinstance(item, dict):
                topics.append(item)
    except Exception as e:
        node.get_logger().error(f"Error loading config file: {e}")
        rclpy.shutdown()
        return


    multi_monitor = MultiMonitor()

    node.get_logger().info("Registering topic monitors:")
    node.get_logger().info(f"Loaded topics from config:\n{topics}")
    for i, topic in enumerate(topics):
        if not isinstance(topic, dict):
            continue
        if topic.get("include", True) is False:
            continue

        topic_name = topic.get("name")
        if not topic_name:
            continue

        qos_profile = resolve_qos(topic, multi_monitor)
        msg_type = resolve_msg_type(topic, multi_monitor)

        node.get_logger().info(f"[Monitor-{i}] Topic: {topic_name}")
        node.get_logger().info(f"[Monitor-{i}] Msg Type: {msg_type}")
        node.get_logger().info(f"[Monitor-{i}] QoS Profile: {qos_profile}")

        topic_monitor = TopicMonitor(
            topic_name=topic_name,
            msg_type=msg_type,
            qos_profile=qos_profile,
            rate=topic.get("rate", 0),
            N=topic.get("N", 0),
            signal_when_config=topic.get("signal_when", {}),
            signal_lambdas_config=topic.get("signal_lambdas", []),
            processes=topic.get("execute", []),
            timeout=topic.get("timeout", 0),
            default_notifications=topic.get("default_notifications", True),
            # event_callback=None,
            event_callback=event_callback,
            thread_num=i,
            enable_internal_logs=topic.get("enable_internal_logs", True)
        )

        topic_monitors.append(topic_monitor)
        multi_monitor.register_monitor(topic_monitor)

    # for topic_monitor in topic_monitors:
    #     topic_monitor.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)  # test_sentor

    # Also add each TopicMonitor's node
    for topic_monitor in topic_monitors:
        executor.add_node(topic_monitor.get_node())

    executor.add_node(multi_monitor)

    executor.spin()

if __name__ == "__main__":
    main()
