#!/usr/bin/env python3
"""Drive a square using /odom feedback.

Each side: drive forward until the robot has moved `side_length` metres from
where the side started. Each corner: turn left in place until the heading has
changed by 90 degrees. After `sides` sides the node stops and exits.

The robot stops early if the bumper or a cliff sensor triggers.
Wheel odometry drifts a little, so the square will not close perfectly.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

FORWARD = 'forward'
TURN = 'turn'


def yaw_from_quaternion(q):
    """Heading angle (radians) from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """Smallest signed difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')

        self.declare_parameter('side_length', 0.5)     # metres
        self.declare_parameter('forward_speed', 0.1)   # m/s
        self.declare_parameter('turn_speed', 0.5)      # rad/s
        self.declare_parameter('sides', 4)
        self.side_length = self.get_parameter('side_length').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.sides = self.get_parameter('sides').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector, 'hazard_detection',
            self.on_hazard, qos_profile_sensor_data)

        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.state = FORWARD
        self.sides_done = 0
        self.leg_start_x = self.leg_start_y = 0.0
        self.turn_start_yaw = 0.0
        self.finished = False

        self.create_timer(0.05, self.tick)  # 20 Hz control loop
        self.get_logger().info(
            f'drive_square started (side={self.side_length} m, '
            f'speed={self.forward_speed} m/s, turn={self.turn_speed} rad/s, '
            f'sides={self.sides}); waiting for /odom')

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.have_odom:
            self.have_odom = True
            self.start_forward()

    def on_hazard(self, msg):
        for d in msg.detections:
            if d.type in (HazardDetection.BUMP, HazardDetection.CLIFF):
                self.get_logger().warn(
                    f'hazard at {d.header.frame_id}, stopping the square')
                self.finished = True
                return

    def start_forward(self):
        self.state = FORWARD
        self.leg_start_x, self.leg_start_y = self.x, self.y
        self.get_logger().info(f'side {self.sides_done + 1}/{self.sides}: forward')

    def start_turn(self):
        self.state = TURN
        self.turn_start_yaw = self.yaw
        self.get_logger().info('corner: turning left 90 degrees')

    def tick(self):
        if self.finished or not self.have_odom:
            return
        cmd = Twist()
        if self.state == FORWARD:
            travelled = math.hypot(self.x - self.leg_start_x,
                                   self.y - self.leg_start_y)
            if travelled >= self.side_length:
                self.sides_done += 1
                if self.sides_done >= self.sides:
                    self.get_logger().info('square finished')
                    self.finished = True
                    return
                self.start_turn()
            else:
                cmd.linear.x = self.forward_speed
        else:  # TURN
            turned = angle_diff(self.yaw, self.turn_start_yaw)
            if turned >= math.pi / 2:
                self.start_forward()
            else:
                cmd.angular.z = self.turn_speed
        self.cmd_pub.publish(cmd)

    def stop(self):
        # Send a few zero-velocity commands so the stop gets through
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main():
    # Python raises KeyboardInterrupt on Ctrl+C, so we can still send a stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DriveSquare()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
