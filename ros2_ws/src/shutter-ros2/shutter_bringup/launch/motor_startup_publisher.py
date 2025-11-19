#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class MotorStartupNode(Node):
    def __init__(self):
        super().__init__('motor_startup_publisher')
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/joint_group_controller/command',
                                         10)

        msg = Float64MultiArray()
        msg.data = [0.0, 1.0, -1.54, 0.0]  # ← your motor values

        # publish once after a small delay (to ensure controllers are running)
        time.sleep(1.0)
        self.pub.publish(msg)
        self.get_logger().info(f"Published initial motor command: {msg.data}")

        # shut down node
        rclpy.shutdown()

def main():
    rclpy.init()
    MotorStartupNode()

if __name__ == '__main__':
    main()
