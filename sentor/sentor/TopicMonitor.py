from threading import Thread, Event, Lock
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.timer import Timer
from sentor.ROSTopicFilter import ROSTopicFilter
from sentor.ROSTopicHz import ROSTopicHz
from sentor.ROSTopicPub import ROSTopicPub
from sentor.Executor import Executor
import time

class TopicMonitor(Thread):
    def __init__(
        self,
        topic_name,
        msg_type,
        qos_profile,
        rate,
        N,
        signal_when_config,
        signal_lambdas_config,
        processes,
        timeout,
        default_notifications,
        event_callback,
        thread_num,
        enable_internal_logs=True   # <-- new optional param
    ):
        """
        :param enable_internal_logs: If True, prints extra developer logs (like 'Lambda satisfied' traces).
        """
        super().__init__()

        # Basic fields
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.qos_profile = qos_profile
        self.rate = rate
        self.N = N
        self.signal_when_config = signal_when_config
        self.signal_lambdas_config = signal_lambdas_config
        self.processes = processes
        self.timeout = timeout if timeout > 0 else 0.1
        self.default_notifications = default_notifications
        self._event_callback = event_callback
        self.thread_num = thread_num

        # Developer log toggle
        self.enable_internal_logs = enable_internal_logs

        # Thread-control + concurrency
        self._stop_event = Event()
        self._lock = Lock()

        # Condition tracking
        self.conditions = {}
        self.sat_expr_timer = {}

        # Create the ROS2 Node
        self.node = Node(f"topic_monitor_{thread_num}")
        self.debug(f"[TopicMonitor] Created for topic: {self.topic_name}")

        # QoS fallback
        if self.qos_profile is None:
            self.node.get_logger().warn(f"[TopicMonitor] No QoS profile for {self.topic_name}, using fallback QoS")
            self.qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        # Optionally set up an Executor if needed
        if processes:
            self.executor = Executor(processes, event_callback)
        else:
            self.executor = None

        # Parse signal_when config
        self.process_signal_config()

        # Create ROSTopicHz for frequency monitoring
        self.hz_monitor = ROSTopicHz(self.node, self.topic_name, 1000, self.N)
        self.hz_monitor.start_monitoring(self.msg_type, self.qos_profile)

        # Periodically log frequency
        self.node.create_timer(5.0, self.log_hz_stats)

        # Set up ROSTopicPub if signal_when == "published"
        if self.signal_when_cfg.get("signal_when", "").lower() == "published":
            self.debug(f"[TopicMonitor] Setting up ROSTopicPub for {self.topic_name}")
            self.pub_monitor = ROSTopicPub(
                node=self.node,
                topic_name=self.topic_name,
                msg_type=self.msg_type,
                qos_profile=self.qos_profile,
                throttle_val=self.N
            )
            self.pub_monitor.register_published_cb(self.published_cb)

        # Build lambda filters
        for i, signal_lambda in enumerate(self.signal_lambdas_config):
            config = self.process_lambda_config(signal_lambda)
            expr = config["expr"]

            # Track condition
            self.conditions[expr] = {
                "satisfied": False,
                "safety_critical": config["safety_critical"],
                "autonomy_critical": config["autonomy_critical"],
                "tags": config.get("tags", [])
            }

            # Create ROSTopicFilter
            filter_monitor = ROSTopicFilter(
                self.node, self.topic_name, expr, config, throttle_val=self.N,
            )
            filter_monitor.register_satisfied_cb(self.lambda_satisfied_cb)
            filter_monitor.register_unsatisfied_cb(self.lambda_unsatisfied_cb)

    def debug(self, msg):
        """Helper method: only logs if enable_internal_logs=True."""
        if self.enable_internal_logs:
            self.node.get_logger().info(msg)

    def process_signal_config(self):
        """Parses the 'signal_when_config' to fill up self.signal_when_cfg dict and create conditions."""
        self.signal_when_cfg = {
            "signal_when": self.signal_when_config.get("condition", ""),
            "timeout": self.signal_when_config.get("timeout", self.timeout),
            "safety_critical": self.signal_when_config.get("safety_critical", False),
            "autonomy_critical": self.signal_when_config.get("autonomy_critical", False),
            "default_notifications": self.signal_when_config.get("default_notifications", self.default_notifications),
            "tags": self.signal_when_config.get("tags", [])
        }
        if self.signal_when_cfg["timeout"] <= 0:
            self.signal_when_cfg["timeout"] = self.timeout

        # If there's a condition, create an entry in self.conditions
        if self.signal_when_cfg["signal_when"]:
            self.conditions[self.signal_when_cfg["signal_when"]] = {
                "satisfied": False,
                "safety_critical": self.signal_when_cfg["safety_critical"],
                "autonomy_critical": self.signal_when_cfg["autonomy_critical"],
                "tags": self.signal_when_cfg["tags"]
            }

    def process_lambda_config(self, signal_lambda):
        """Parses each signal_lambda config to unify structure and fallback timeouts."""
        config = {
            "expr": signal_lambda.get("expression", ""),
            "timeout": signal_lambda.get("timeout", self.timeout),
            "safety_critical": signal_lambda.get("safety_critical", False),
            "autonomy_critical": signal_lambda.get("autonomy_critical", False),
            "default_notifications": signal_lambda.get("default_notifications", self.default_notifications),
            "repeat_exec": signal_lambda.get("repeat_exec", False),
            "tags": signal_lambda.get("tags", [])
        }
        if config["timeout"] <= 0:
            config["timeout"] = self.timeout
        return config

    def lambda_satisfied_cb(self, expr, msg, config):
        if expr in self.sat_expr_timer:
            return

        def on_timeout():
            self.debug(f"[TopicMonitor] Lambda satisfied: {expr}")
            self.conditions[expr]["satisfied"] = True

            if config.get("default_notifications"):
                message = f"Lambda '{expr}' satisfied on topic {self.topic_name}"
                level = "error" if config.get("safety_critical") else "warn"

                if self._event_callback:
                    self._event_callback(message, level, msg=msg, topic_name=self.topic_name)

                if level == "error":
                    self.node.get_logger().error(message)
                else:
                    self.node.get_logger().warn(message)

            # Here, mimic ROS1 by calling execute() when the condition is met.
            if not config.get("repeat_exec", False):
                # process_indices should be provided in your YAML for this lambda.
                self.execute(msg, config.get("process_indices"))
            # Optionally, if repeat_exec is enabled, you could set up a repeat timer.

        with self._lock:
            self.sat_expr_timer[expr] = self.node.create_timer(
                config["timeout"], lambda: self._on_lambda_timeout(expr, on_timeout)
            )

    def _on_lambda_timeout(self, expr, callback):
        """Helper to cancel the timer and then run the provided callback."""
        with self._lock:
            if expr in self.sat_expr_timer:
                self.sat_expr_timer[expr].cancel()
                del self.sat_expr_timer[expr]
        callback()

    def lambda_unsatisfied_cb(self, expr):
        """Called when the lambda condition is no longer satisfied."""
        with self._lock:
            if expr in self.sat_expr_timer:
                self.sat_expr_timer[expr].cancel()
                del self.sat_expr_timer[expr]
        if expr in self.conditions:
            self.conditions[expr]["satisfied"] = False

    def published_cb(self, msg):
        """Called when a 'published' message is received."""
        self.debug(f"[TopicMonitor] Topic {self.topic_name} is published")
        if "published" in self.conditions:
            self.conditions["published"]["satisfied"] = True

    def log_hz_stats(self):
        """Timer callback to log frequency stats from ROSTopicHz."""
        if not self.hz_monitor.enabled:
            return
        stats = self.hz_monitor.get_hz()
        if stats:
            rate, min_d, max_d, std_d, n = stats
            if self.enable_internal_logs:
                self.node.get_logger().info(
                    f"[HzMonitor] Rate={rate:.2f} Hz | Min={min_d:.3f}s | Max={max_d:.3f}s | Std={std_d:.4f}s | N={n}"
                )
        else:
            if self.enable_internal_logs:
                self.node.get_logger().info("[HzMonitor] No Hz stats yet. Waiting for more data.")

    def execute(self, msg=None, process_indices=None):
        """Triggers the executor to run configured processes if available."""
        if self.processes and self.executor:
            self.executor.execute(msg, process_indices)

    def run(self):
        """Background thread spin loop for the TopicMonitor's node."""
        while True:
            if not self._stop_event.is_set():
                rclpy.spin_once(self.node, timeout_sec=0.1)
            else:
                time.sleep(0.1)

    def stop_monitor(self):
        self._stop_event.set()
        self.hz_monitor.stop_monitoring()

    def start_monitor(self):
        self._stop_event.clear()
        self.node.get_logger().info(f"[TopicMonitor] Restarting monitor for {self.topic_name}")
        self.hz_monitor.start_monitoring(self.msg_type, self.qos_profile)

    def get_node(self):
        """Returns the node so it can be added to an executor externally."""
        return self.node

    def event_callback(message, level="info", msg=None, topic_name=None):
        print(f"[{level.upper()}] {message}")

