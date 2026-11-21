#!/usr/bin/env python3
"""Drive the Create 3 from the keyboard with W A S D.

Keys (click on this terminal first, then press them):

    w  forward            q  forward and left       r  faster
    s  backward           e  forward and right      f  slower
    a  turn left          space  stop               x  quit
    d  turn right

Hold a key to keep moving. After you let go the robot stops by itself (within
about `hold_time` seconds), so if this program dies it does not run away.
A single tap moves it for `first_hold` seconds. Ctrl+C also stops it.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

MAX_LINEAR = 0.3     # m/s, close to the Create 3's own limit
MAX_ANGULAR = 1.5    # rad/s
LINEAR_STEP = 0.03   # how much 'r' / 'f' change the speed
ANGULAR_STEP = 0.2

# key -> (forward direction, turn direction, name); positive turn = left
KEYS = {
    'w': (1.0, 0.0, 'forward'),
    's': (-1.0, 0.0, 'backward'),
    'a': (0.0, 1.0, 'turn left'),
    'd': (0.0, -1.0, 'turn right'),
    'q': (1.0, 1.0, 'forward and left'),
    'e': (1.0, -1.0, 'forward and right'),
}


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')

        number = ParameterDescriptor(dynamic_typing=True)   # accept 1 and 1.0
        self.declare_parameter('linear_speed', 0.15, number)    # m/s
        self.declare_parameter('angular_speed', 0.8, number)    # rad/s
        self.declare_parameter('hold_time', 0.3, number)        # seconds to keep moving after a repeat
        self.declare_parameter('first_hold', 0.8, number)       # seconds to keep moving after a new press
        self.declare_parameter('publish_rate', 20.0, number)    # commands per second
        self.linear_speed = min(float(self.get_parameter('linear_speed').value), MAX_LINEAR)
        self.angular_speed = min(float(self.get_parameter('angular_speed').value), MAX_ANGULAR)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.first_hold = float(self.get_parameter('first_hold').value)
        self.period = 1.0 / max(float(self.get_parameter('publish_rate').value), 1.0)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.direction = (0.0, 0.0)   # (forward, turn), each -1, 0 or 1
        self.expire = 0.0             # time when the current movement runs out
        self.last_key = ('', 0.0)     # (key, time) of the latest movement key
        self.stop_repeats = 0         # extra zero commands still to send after a stop
        self.label = ''
        self.running = True

    def set_direction(self, key, forward, turn, name):
        now = time.monotonic()
        # A key that arrives right after the same key is the terminal's auto-repeat.
        # The first press of a held key is followed by a pause before the repeats
        # start, so a new press has to keep the robot going for longer.
        is_repeat = key == self.last_key[0] and now - self.last_key[1] < 0.15
        self.last_key = (key, now)
        hold = self.hold_time if is_repeat else self.first_hold
        self.direction = (forward, turn)
        self.expire = max(self.expire, now + hold) if is_repeat else now + hold
        if name != self.label:
            self.label = name
            self.get_logger().info(name)

    def stop_now(self):
        if self.direction != (0.0, 0.0):
            self.direction = (0.0, 0.0)
            self.stop_repeats = 3
            self.label = ''
            self.get_logger().info('stop')

    def handle_keys(self, data):
        if '\x1b' in data:      # arrow keys etc. send escape sequences: ignore them
            return
        for key in data.lower():
            if key in KEYS:
                self.set_direction(key, *KEYS[key])
            elif key == ' ':
                self.stop_now()
            elif key == 'r':
                self.change_speed(LINEAR_STEP, ANGULAR_STEP)
            elif key == 'f':
                self.change_speed(-LINEAR_STEP, -ANGULAR_STEP)
            elif key == 'x':
                self.running = False

    def change_speed(self, linear_delta, angular_delta):
        self.linear_speed = min(max(self.linear_speed + linear_delta, 0.03), MAX_LINEAR)
        self.angular_speed = min(max(self.angular_speed + angular_delta, 0.2), MAX_ANGULAR)
        self.get_logger().info(
            f'speed: {self.linear_speed:.2f} m/s, {self.angular_speed:.1f} rad/s')

    def tick(self):
        """Called every cycle: stop on timeout, publish the current command."""
        if self.direction != (0.0, 0.0) and time.monotonic() > self.expire:
            self.stop_now()
        cmd = Twist()
        if self.direction != (0.0, 0.0):
            cmd.linear.x = self.direction[0] * self.linear_speed
            cmd.angular.z = self.direction[1] * self.angular_speed
            self.cmd_pub.publish(cmd)
        elif self.stop_repeats > 0:
            self.stop_repeats -= 1
            self.cmd_pub.publish(cmd)   # all zeros

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)   # read single keys; Ctrl+C still works
        try:
            while rclpy.ok() and self.running:
                ready, _, _ = select.select([fd], [], [], self.period)
                if ready:
                    self.handle_keys(os.read(fd, 64).decode(errors='ignore'))
                self.tick()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def stop(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main():
    if not sys.stdin.isatty():
        print('wasd_teleop needs a real terminal. Run it with: '
              'ros2 run create3_demos wasd_teleop')
        return
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WasdTeleop()
    print(__doc__)
    try:
        node.run()
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
