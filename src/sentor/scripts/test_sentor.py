#!/usr/bin/env python3
"""
Created on 20 May 2025

@author: Zhuoling Huang
"""
import os
import yaml
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty

from sentor.MultiMonitor   import MultiMonitor
from sentor.TopicMonitor   import TopicMonitor
from sentor.NodeMonitor    import NodeMonitor
from sentor.SafetyMonitor  import SafetyMonitor

# ─── Globals ────────────────────────────────────────────────────────────────
topic_monitors = []
node_monitors  = []
multi_monitor  = None

def shutdown_handler(signum, frame):
    for tm in topic_monitors:
        tm.stop_monitor()
    for nm in node_monitors:
        nm.stop_monitor()
    if multi_monitor:
        multi_monitor.stop_monitor()
    rclpy.shutdown()

signal.signal(signal.SIGINT, shutdown_handler)

# ─── Services ───────────────────────────────────────────────────────────────
def start_monitoring(request, _):
    for tm in topic_monitors: tm.start_monitor()
    for nm in node_monitors:  nm.start_monitor()
    multi_monitor.start_monitor()
    return Empty.Response()

def stop_monitoring(request, _):
    for tm in topic_monitors: tm.stop_monitor()
    for nm in node_monitors:  nm.stop_monitor()
    multi_monitor.stop_monitor()
    return Empty.Response()

def event_callback(msg, level="info", **_):
    print(f"[{level.upper()}] {msg}")

# ─── Helpers ────────────────────────────────────────────────────────────────
def get_message_type(type_str):
    pkg, msg = type_str.split('/msg/')
    module = __import__(f"{pkg}.msg", fromlist=[msg])
    return getattr(module, msg)

def get_qos_profile(qos):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    rel = ReliabilityPolicy.BEST_EFFORT if qos.get("reliability","").lower()=="best_effort" else ReliabilityPolicy.RELIABLE
    dur = DurabilityPolicy.TRANSIENT_LOCAL if qos.get("durability","").lower()=="transient_local" else DurabilityPolicy.VOLATILE
    hist= HistoryPolicy.KEEP_ALL if qos.get("history","").lower()=="keep_all" else HistoryPolicy.KEEP_LAST
    return QoSProfile(reliability=rel, durability=dur, history=hist, depth=qos.get("depth",10))

# ─── main() ─────────────────────────────────────────────────────────────────
def main():
    global topic_monitors, node_monitors, multi_monitor

    rclpy.init()
    driver = rclpy.create_node("test_sentor")

    # load config
    cfg_path = driver.declare_parameter("config_file","~/config/test_monitor_config.yaml")\
                     .get_parameter_value().string_value
    cfg_path = os.path.expanduser(cfg_path)
    with open(cfg_path,'r') as f:
        cfg = yaml.safe_load(f) or {}

    topics_cfg = cfg.get("monitors",   [])
    nodes_cfg  = cfg.get("node_monitors", [])

    # start/stop services
    driver.create_service(Empty, "start_monitoring", start_monitoring)
    driver.create_service(Empty, "stop_monitoring",  stop_monitoring)

    # multi_monitor
    multi_monitor = MultiMonitor()

    # 1) TopicMonitors
    for idx, t in enumerate(topics_cfg):
        # ensure every condition—and every lambda—has a tags list
        t.setdefault("signal_when", {}).setdefault("tags", [])
        for lam in t.get("signal_lambdas", []):
            lam.setdefault("tags", [])

        tm = TopicMonitor(
            topic_name            = t["name"],
            msg_type              = get_message_type(t["message_type"]),
            qos_profile           = get_qos_profile(t.get("qos",{})),
            rate                  = t["rate"],
            N                     = t["N"],
            signal_when_config    = t.get("signal_when", {}),
            signal_lambdas_config = t.get("signal_lambdas", []),
            processes             = t.get("execute", []),
            timeout               = t.get("timeout", 0.1),
            default_notifications = t.get("default_notifications", True),
            event_callback        = event_callback,
            thread_num            = idx,
            enable_internal_logs  = True,
        )
        topic_monitors.append(tm)
        multi_monitor.register_monitor(tm)

    # 2) NodeMonitors
    for n in nodes_cfg:
        nm = NodeMonitor(
            target_node_name  = n["name"],
            timeout           = n["timeout"],
            event_callback    = event_callback,
            safety_critical   = n.get("safety_critical", False),
            autonomy_critical = n.get("autonomy_critical", False),
            poll_rate         = n.get("poll_rate", 1.0),
        )
        node_monitors.append(nm)

    # 3) safety heartbeat (safety_critical)
    safety_hb = SafetyMonitor(
        topic     = 'safety/heartbeat',
        event_msg = 'Heartbeat',
        attr      = 'is_alive',
        srv_name  = 'heartbeat_override',
        event_cb  = event_callback,
        invert    = False,
    )
    # register both topic & node monitors
    for tm in topic_monitors:
        if tm.signal_when_cfg.get("safety_critical") or \
           any(l.get("safety_critical") for l in tm.signal_lambdas_config):
            safety_hb.register_monitor(tm)
    for nm in node_monitors:
        if nm.safety_critical:
            safety_hb.register_monitor(nm)

    # 4) warning heartbeat (autonomy_critical)
    warning_hb = SafetyMonitor(
        topic     = 'warning/heartbeat',
        event_msg = 'Warning-beat',
        attr      = 'is_autonomy_alive',
        srv_name  = 'warning_override',
        event_cb  = event_callback,
        invert    = False,
    )
    for tm in topic_monitors:
        if tm.signal_when_cfg.get("autonomy_critical") or \
           any(l.get("autonomy_critical") for l in tm.signal_lambdas_config):
            warning_hb.register_monitor(tm)
    for nm in node_monitors:
        if nm.autonomy_critical:
            warning_hb.register_monitor(nm)

    # 5) spin everything together
    executor = MultiThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(multi_monitor)
    executor.add_node(safety_hb)
    executor.add_node(warning_hb)
    for tm in topic_monitors:
        executor.add_node(tm.get_node())
    for nm in node_monitors:
        executor.add_node(nm.node)

    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



# def __signal_handler(signum, frame):
#     """ Gracefully stop all monitors on SIGINT. """
#     for topic_monitor in topic_monitors:
#         topic_monitor.kill_monitor()
#     multi_monitor.stop_monitor()
#     print("Stopped monitoring.")
#     os._exit(signal.SIGTERM)

# # def stop_monitoring(_):
# #     """ Stop all monitoring activities. """
# #     for topic_monitor in topic_monitors:
# #         topic_monitor.stop_monitor()
# #     multi_monitor.stop_monitor()
# #     return Empty.Response()

# # def start_monitoring(_):    
# #     """ Start monitoring activities. """
# #     for topic_monitor in topic_monitors:
# #         topic_monitor.start_monitor()
# #     multi_monitor.start_monitor()
# #     return Empty.Response()

# def start_monitoring(request, _):
#     print("[Service] Received start_monitoring request")
#     for topic_monitor in topic_monitors:
#         topic_monitor.start_monitor()
#     multi_monitor.start_monitor()
#     return Empty.Response()

# def stop_monitoring(request, _):
#     print("[Service] Received stop_monitoring request")
#     for topic_monitor in topic_monitors:
#         topic_monitor.stop_monitor()
#     multi_monitor.stop_monitor()
#     return Empty.Response()

# def resolve_qos(topic, multi_monitor):
#     return get_qos_profile(topic["qos"]) if "qos" in topic else None

# def resolve_msg_type(topic, multi_monitor):
#     return get_message_type(topic["message_type"]) if "message_type" in topic else None

# def get_message_type(type_str):
#     """Import the actual message type class from a string like 'std_msgs/msg/Int32'."""
#     if not type_str:
#         return None
#     try:
#         package, msg_name = type_str.split('/msg/')
#         module = importlib.import_module(f"{package}.msg")
#         return getattr(module, msg_name)
#     except Exception as e:
#         print(f"[ERROR] Failed to import message type '{type_str}': {e}")
#         return None
# def get_qos_profile(qos_dict):
#     """Convert YAML QoS dict into actual QoSProfile object."""
#     reliability = ReliabilityPolicy.RELIABLE
#     durability = DurabilityPolicy.VOLATILE
#     history = HistoryPolicy.KEEP_LAST
#     depth = 10

#     if qos_dict.get("reliability", "").lower() == "best_effort":
#         reliability = ReliabilityPolicy.BEST_EFFORT
#     if qos_dict.get("durability", "").lower() == "transient_local":
#         durability = DurabilityPolicy.TRANSIENT_LOCAL
#     if qos_dict.get("history", "").lower() == "keep_all":
#         history = HistoryPolicy.KEEP_ALL
#     if "depth" in qos_dict:
#         depth = qos_dict["depth"]

#     return QoSProfile(
#         reliability=reliability,
#         durability=durability,
#         history=history,
#         depth=depth
#     )

# def event_callback(message, level="info", msg=None, nodes=None, topic_name=None):
#     prefix = f"[{level.upper()}]"
#     print(f"{prefix} {message}")

# def main():
#     global multi_monitor
#     rclpy.init()
#     node = rclpy.create_node("test_sentor")

#     node.get_logger().info("Registering start/stop monitoring services...")
#     node.create_service(Empty, "start_monitoring", start_monitoring)
#     node.create_service(Empty, "stop_monitoring", stop_monitoring)

#     config_file = node.declare_parameter("config_file", "config/test_monitor_config.yaml").value
#     topics = []

#     try:
#         items = [yaml.safe_load(open(item, 'r')) for item in config_file.split(',')]
#         for item in items:
#             if isinstance(item, dict) and "monitors" in item:
#                 topics.extend(item["monitors"])
#             elif isinstance(item, list):
#                 topics.extend(item)
#             elif isinstance(item, dict):
#                 topics.append(item)
#     except Exception as e:
#         node.get_logger().error(f"Error loading config file: {e}")
#         rclpy.shutdown()
#         return

#     multi_monitor = MultiMonitor()

#     node.get_logger().info("Registering topic monitors:")
#     node.get_logger().info(f"Loaded topics from config:\n{topics}")
#     for i, topic in enumerate(topics):
#         if not isinstance(topic, dict):
#             continue
#         if topic.get("include", True) is False:
#             continue

#         topic_name = topic.get("name")
#         if not topic_name:
#             continue

#         qos_profile = resolve_qos(topic, multi_monitor)
#         msg_type = resolve_msg_type(topic, multi_monitor)

#         node.get_logger().info(f"[Monitor-{i}] Topic: {topic_name}")
#         node.get_logger().info(f"[Monitor-{i}] Msg Type: {msg_type}")
#         node.get_logger().info(f"[Monitor-{i}] QoS Profile: {qos_profile}")

#         topic_monitor = TopicMonitor(
#             topic_name=topic_name,
#             msg_type=msg_type,
#             qos_profile=qos_profile,
#             rate=topic.get("rate", 0),
#             N=topic.get("N", 0),
#             signal_when_config=topic.get("signal_when", {}),
#             signal_lambdas_config=topic.get("signal_lambdas", []),
#             processes=topic.get("execute", []),
#             timeout=topic.get("timeout", 0),
#             default_notifications=topic.get("default_notifications", True),
#             event_callback=event_callback,
#             thread_num=i,
#             enable_internal_logs=topic.get("enable_internal_logs", True)
#         )

#         topic_monitors.append(topic_monitor)
#         multi_monitor.register_monitor(topic_monitor)

#     # # Create SafetyMonitor
#     # safety_monitor = SafetyMonitor(
#     #     topic='safety/heartbeat',
#     #     event_msg='Heartbeat',
#     #     attr='is_alive',
#     #     srv_name='heartbeat_override',
#     #     event_cb=event_callback,
#     #     invert=False,
#     # )

#     # # Register TopicMonitors with SafetyMonitor
#     # for tm in topic_monitors:
#     #     # signal_when critical OR any lambda critical?
#     #     sw_crit = tm.signal_when_cfg.get('safety_critical', False)
#     #     lambdas_crit = any(l.get('safety_critical', False) for l in tm.signal_lambdas_config)
#     #     if sw_crit or lambdas_crit:
#     #         safety_monitor.register_monitor(tm)

    
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)  # test_sentor
    
#     # ──────────────────────────────────────────────────────────────────────────────
#     # (1) your existing safety monitor…
#     safety_monitor = SafetyMonitor(
#         topic='safety/heartbeat',
#         event_msg='Heartbeat',
#         attr='is_alive',
#         srv_name='heartbeat_override',
#         event_cb=event_callback,
#         invert=False,
#     )
#     for tm in topic_monitors:
#         sw_crit = tm.signal_when_cfg.get('safety_critical', False)
#         lam_crit = any(l.get('safety_critical', False) for l in tm.signal_lambdas_config)
#         if sw_crit or lam_crit:
#             safety_monitor.register_monitor(tm)
#     executor.add_node(safety_monitor)
#     # ──────────────────────────────────────────────────────────────────────────────

#     # (2) NEW: warning‐level heartbeat monitor
#     warning_monitor = SafetyMonitor(
#         topic='warning/heartbeat',
#         event_msg='Warning‐beat',
#         attr='is_autonomy_alive',
#         srv_name='warning_override',
#         event_cb=event_callback,
#         invert=False,
#     )
#     for tm in topic_monitors:
#         # pick up anything marked autonomy_critical in signal_when or lambdas
#         sw_warn = tm.signal_when_cfg.get('autonomy_critical', False)
#         lam_warn = any(l.get('autonomy_critical', False) for l in tm.signal_lambdas_config)
#         if sw_warn or lam_warn:
#             warning_monitor.register_monitor(tm)
#     executor.add_node(warning_monitor)

#     for topic_monitor in topic_monitors:
#         executor.add_node(topic_monitor.get_node())

#     executor.add_node(multi_monitor)
#     # executor.add_node(safety_monitor)  # ✅ Add SafetyMonitor to executor

#     executor.spin()

# if __name__ == "__main__":
#     main()