#!/usr/bin/env python3
"""Drive a square using /odom feedback.

Each side: drive forward until the robot has moved `side_length` metres from
where the side started. Each corner: turn left in place until it has rotated
90 degrees. After `sides` sides the node stops and exits.

Made robust for a slow or lossy Wi-Fi link:
- The rotation is ACCUMULATED from small heading changes, so it cannot be fooled
  by the +/-180 degree wrap-around when odometry arrives late.
- The robot slows down as it nears each target, so late odometry overshoots less.
- If odometry stops arriving, the robot is told to stand still until it returns.
- The robot pauses briefly after each side and corner so odometry can catch up.
- The robot stops early if the bumper or a cliff sensor triggers.
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
PAUSE = 'pause'


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

        self.declare_parameter('side_length', 0.2)         # metres
        self.declare_parameter('forward_speed', 0.08)      # m/s (maximum)
        self.declare_parameter('turn_speed', 0.4)          # rad/s (maximum)
        self.declare_parameter('sides', 4)
        self.declare_parameter('min_forward_speed', 0.03)  # m/s near the target
        self.declare_parameter('min_turn_speed', 0.15)     # rad/s near the target
        self.declare_parameter('odom_timeout', 0.5)        # s without odom -> stand still
        self.declare_parameter('pause_time', 0.5)          # s pause between phases
        self.side_length = self.get_parameter('side_length').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.sides = self.get_parameter('sides').value
        self.min_forward_speed = self.get_parameter('min_forward_speed').value
        self.min_turn_speed = self.get_parameter('min_turn_speed').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.pause_time = self.get_parameter('pause_time').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector, 'hazard_detection',
            self.on_hazard, qos_profile_sensor_data)

        self.x = self.y = self.yaw = 0.0
        self.turned = 0.0              # rotation accumulated since the turn began
        self.have_odom = False
        self.last_odom_time = 0.0
        self.last_warn_time = -1e9
        self.state = PAUSE
        self.pause_until = 0.0
        self.next_action = None
        self.sides_done = 0
        self.leg_start_x = self.leg_start_y = 0.0
        self.finished = False

        self.create_timer(0.05, self.tick)  # 20 Hz control loop
        self.get_logger().info(
            f'drive_square started (side={self.side_length} m, '
            f'speed={self.forward_speed} m/s, turn={self.turn_speed} rad/s, '
            f'sides={self.sides}); waiting for /odom')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.have_odom:
            # Add up small heading changes instead of comparing with the start
            # heading, so the +/-pi wrap-around can never fool us.
            self.turned += angle_diff(yaw, self.yaw)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw
        self.last_odom_time = self.now()
        if not self.have_odom:
            self.have_odom = True
            self.begin_pause(self.start_forward)

    def on_hazard(self, msg):
        for d in msg.detections:
            if d.type in (HazardDetection.BUMP, HazardDetection.CLIFF):
                self.get_logger().warning(
                    f'hazard at {d.header.frame_id}, stopping the square')
                self.finished = True
                return

    def begin_pause(self, next_action):
        self.state = PAUSE
        self.next_action = next_action
        self.pause_until = self.now() + self.pause_time

    def start_forward(self):
        self.state = FORWARD
        self.leg_start_x, self.leg_start_y = self.x, self.y
        self.get_logger().info(f'side {self.sides_done + 1}/{self.sides}: forward')

    def start_turn(self):
        self.state = TURN
        self.turned = 0.0
        self.get_logger().info('corner: turning left 90 degrees')

    def tick(self):
        if self.finished or not self.have_odom:
            return
        now = self.now()

        # Odometry stopped arriving: stand still until it comes back.
        if now - self.last_odom_time > self.odom_timeout:
            self.cmd_pub.publish(Twist())
            if now - self.last_warn_time > 2.0:
                self.last_warn_time = now
                self.get_logger().warning(
                    'no odometry for a while, standing still until it returns')
            return

        cmd = Twist()
        if self.state == PAUSE:
            if now >= self.pause_until:
                self.next_action()
        elif self.state == FORWARD:
            travelled = math.hypot(self.x - self.leg_start_x,
                                   self.y - self.leg_start_y)
            remaining = self.side_length - travelled
            if remaining <= 0.0:
                self.sides_done += 1
                if self.sides_done >= self.sides:
                    self.get_logger().info('square finished')
                    self.finished = True
                    return
                self.begin_pause(self.start_turn)
            else:
                # Slow down as the target gets close
                cmd.linear.x = min(self.forward_speed,
                                   max(self.min_forward_speed, remaining))
        elif self.state == TURN:
            remaining = math.pi / 2 - self.turned
            if remaining <= 0.0:
                self.begin_pause(self.start_forward)
            else:
                cmd.angular.z = min(self.turn_speed,
                                    max(self.min_turn_speed, remaining))
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
