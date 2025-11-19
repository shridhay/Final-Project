#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class MotorStartupPublisher(Node):
    def __init__(self):
        super().__init__('motor_startup_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_controller/command', 10)

        # Joint positions [motor1, motor2, motor3, motor4]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

        self.angle = 0.0  # global phase angle
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        amplitude = math.pi / 2  # max ±pi/2 for each motor
        self.angle += 0.05  # controls speed of oscillation

        # Each motor oscillates with different phase offsets using cosine
        self.joint_positions[0] = amplitude * math.cos(self.angle)          # Motor1
        self.joint_positions[1] = amplitude * math.cos(self.angle + math.pi/2)  # Motor2
        self.joint_positions[2] = amplitude * math.cos(self.angle + math.pi)    # Motor3
        self.joint_positions[3] = amplitude * math.cos(self.angle + 3*math.pi/2)  # Motor4

        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorStartupPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
