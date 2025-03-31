from threading import Event
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, HistoryPolicy
from sentor_msgs.msg import Monitor, MonitorArray  # Ensure this message is defined

class MultiMonitor(Node):
    def __init__(self, node_name='multi_monitor'):
        super().__init__(node_name)

        # Declare and get safety publication rate from parameters
        self.declare_parameter("safety_pub_rate", 10.0)
        rate = self.get_parameter("safety_pub_rate").value

        self.topic_monitors = []
        self._stop_event = Event()
        self.error_code = []

        self.monitors_pub = None  # Initialized dynamically when first monitor is registered
        self.timer = self.create_timer(1.0 / rate, self.callback)

    def get_default_qos(self):
        """Fallback QoS profile if not provided by monitor."""
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.get_logger().warn("[MultiMonitor] No QoS provided; using fallback default QoS.")
        return qos

    def register_monitor(self, topic_monitor):
        """Register a monitor and set QoS for MonitorArray publisher."""
        self.get_logger().info(f"[MultiMonitor] Registering monitor for topic: {topic_monitor.topic_name}")

        if not self.topic_monitors:
            qos_profile = topic_monitor.qos_profile or self.get_default_qos()
            self.monitors_pub = self.create_publisher(MonitorArray, "/sentor/monitors", qos_profile)

        self.topic_monitors.append(topic_monitor)

    def callback(self):
        """Periodic check and publish monitor states."""
        if not self._stop_event.is_set() and self.monitors_pub is not None:
            error_code_new = [
                monitor.conditions[expr]["satisfied"]
                for monitor in self.topic_monitors
                for expr in monitor.conditions
            ]

            if error_code_new != self.error_code:
                self.error_code = error_code_new

                msg = MonitorArray()
                msg.header.stamp = self.get_clock().now().to_msg()
                count = 0

                for monitor in self.topic_monitors:
                    for expr in monitor.conditions:
                        condition = Monitor()
                        condition.topic = monitor.topic_name
                        condition.condition = expr
                        condition.safety_critical = monitor.conditions[expr]["safety_critical"]
                        condition.autonomy_critical = monitor.conditions[expr]["autonomy_critical"]
                        condition.satisfied = self.error_code[count]
                        condition.tags = monitor.conditions[expr]["tags"]
                        msg.conditions.append(condition)
                        count += 1

                self.monitors_pub.publish(msg)

    def stop_monitor(self):
        self._stop_event.set()

    def start_monitor(self):
        self._stop_event.clear()
