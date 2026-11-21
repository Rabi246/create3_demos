#!/usr/bin/env python3
"""Bump and go (package version): drive forward, turn in place on a bump or cliff.

Speeds and turn time are ROS 2 parameters, so you can change them from the
launch file, a YAML file, or the command line without editing this code.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector


class BumpAndGo(Node):
    def __init__(self):
        super().__init__('bump_and_go')

        # Parameters with defaults (same values as the old script)
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('turn_speed', 0.8)      # rad/s
        self.declare_parameter('turn_time', 2.0)       # seconds
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_time = self.get_parameter('turn_time').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.on_hazard,
            qos_profile_sensor_data,
        )
        self.turn_until = 0.0
        self.create_timer(0.1, self.tick)  # 10 Hz control loop
        self.get_logger().info(
            f'bump_and_go started (forward={self.forward_speed} m/s, '
            f'turn={self.turn_speed} rad/s for {self.turn_time} s)')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_hazard(self, msg):
        for d in msg.detections:
            if d.type in (HazardDetection.BUMP, HazardDetection.CLIFF):
                if self.now() >= self.turn_until:  # ignore if already turning
                    self.turn_until = self.now() + self.turn_time
                    self.get_logger().info(f'hazard at {d.header.frame_id}, turning')
                return

    def tick(self):
        cmd = Twist()
        if self.now() < self.turn_until:
            cmd.angular.z = self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
        self.cmd_pub.publish(cmd)

    def stop(self):
        # Send a few zero-velocity commands so the stop gets through
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main():
    # Keep rclpy from shutting the context down on Ctrl+C before we can send
    # the stop command: Python raises KeyboardInterrupt instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BumpAndGo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
