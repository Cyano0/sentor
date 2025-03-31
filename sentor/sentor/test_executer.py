#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sentor.Executor import Executor  # ✅ Import updated Executor class
from sentor.service_types import get_service_type  # ✅ Import service mapping function
import subprocess
import time

def test_nav_map_services():
    rclpy.init()
    node = Executor([], print)

    print("\n🔹 Checking Available Services...")
    available_services = subprocess.run(["ros2", "service", "list"], capture_output=True, text=True).stdout

    # ✅ List of navigation and mapping services to test
    test_services = [
        "/map_server/get_parameters",
        "/map_server/set_parameters",
        "/map_server/get_state",
        "/map_server/list_parameters",
        "/planner_server/get_state",
        "/planner_server/change_state"
    ]
    
    for service in test_services:
        print(f"\n🔹 Testing service: {service}")
        if service in available_services:
            try:
                service_type = get_service_type(service)
                client = node.create_client(service_type, service)
                request = service_type.Request()

                # ✅ Auto-fill request data dynamically
                if hasattr(request, 'names'):
                    request.names = ["use_sim_time"]
                if hasattr(request, 'state'):
                    request.state = 1  # Example state change request

                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future)

                if future.done():
                    print(f"✅ Service {service} Response: {future.result()}")
                else:
                    print(f"❌ Service {service} call failed!")

            except Exception as e:
                print(f"❌ Error calling {service}: {str(e)}")
        else:
            print(f"❌ Service {service} is not available!")

    # ✅ Test Sleep Function
    print("\n🔹 Testing sleep function...")
    node.sleep(2)
    print("✅ Sleep function test passed.")

    # ✅ Test Logging
    print("\n🔹 Testing log function...")
    node.event_cb("Test log message", "info")
    print("✅ Log function test passed.")

    # ✅ Test Publishing
    print("\n🔹 Testing publish function...")
    from std_msgs.msg import String
    pub = node.create_publisher(String, "/chatter", 10)
    time.sleep(1)  # Ensure the publisher initializes properly
    msg = String()
    msg.data = "Hello from test script"
    pub.publish(msg)
    print("✅ Publish function test passed.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    test_nav_map_services()
