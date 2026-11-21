#!/usr/bin/env python3
"""Bump and go: drive forward, and when something is hit, turn AWAY from it.

Built on the first version (drive forward, turn in place on a bump or cliff).
New: the robot turns away from the side that was hit, shows a color on its light
ring for each kind of hit, and plays a short beep.

Light ring colors:  left = blue, right = green, front = magenta, cliff = yellow.
Everything is a ROS 2 parameter, so you can change it from the YAML file, the
launch file, or the command line without editing this code.
"""
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import (
    AudioNote, AudioNoteVector, HazardDetection, HazardDetectionVector,
    LedColor, LightringLeds)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

# (red, green, blue) for the light ring, 0-255
COLORS = {
    'left': (0, 0, 255),
    'right': (0, 255, 0),
    'front': (255, 0, 255),
    'cliff': (255, 200, 0),
}
# Beep pitch (Hz) for each kind of hit
BEEPS = {
    'left': [440],
    'right': [660],
    'front': [550],
    'cliff': [660, 440],  # two falling notes
}
BEEP_SECONDS = 0.15


def classify(hazard):
    """Return ('left' | 'right' | 'front' | 'cliff')."""
    name = hazard.header.frame_id.lower()
    if hazard.type == HazardDetection.CLIFF:
        return 'cliff'
    if 'left' in name:
        return 'left'
    if 'right' in name:
        return 'right'
    return 'front'


class BumpAndGo(Node):
    def __init__(self):
        super().__init__('bump_and_go')

        # Parameters with defaults
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('turn_speed', 0.8)      # rad/s
        self.declare_parameter('turn_time', 2.0)       # seconds
        self.declare_parameter('lights', True)         # color the light ring on a hit
        self.declare_parameter('sound', True)          # beep on a hit
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_time = self.get_parameter('turn_time').value
        self.use_lights = self.get_parameter('lights').value
        self.use_sound = self.get_parameter('sound').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.light_pub = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.on_hazard,
            qos_profile_sensor_data,
        )
        self.turn_until = 0.0
        self.turn_dir = 1.0      # +1 = turn left, -1 = turn right
        self.lights_on = False
        self.front_flip = 1.0    # alternate left/right when hit dead ahead
        self.create_timer(0.1, self.tick)  # 10 Hz control loop
        self.get_logger().info(
            f'bump_and_go started (forward={self.forward_speed} m/s, '
            f'turn={self.turn_speed} rad/s for {self.turn_time} s, '
            f'lights={self.use_lights}, sound={self.use_sound})')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_hazard(self, msg):
        for d in msg.detections:
            if d.type in (HazardDetection.BUMP, HazardDetection.CLIFF):
                if self.now() >= self.turn_until:  # ignore if already turning
                    self.react(d)
                return

    def react(self, hazard):
        side = classify(hazard)
        # Turn away from the side that was hit. Positive angular z = turn left.
        if side == 'left':
            self.turn_dir = -1.0
        elif side == 'right':
            self.turn_dir = 1.0
        else:  # dead ahead or a cliff: alternate directions
            self.turn_dir = self.front_flip
            self.front_flip = -self.front_flip
        self.turn_until = self.now() + self.turn_time
        way = 'left' if self.turn_dir > 0 else 'right'
        self.get_logger().info(
            f'hazard at {hazard.header.frame_id} ({side}), turning {way}')
        if self.use_lights:
            self.show_color(*COLORS[side])
        if self.use_sound:
            self.beep(BEEPS[side])

    def stamp(self, msg):
        """Fill in the message time if the message has a header."""
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()

    def show_color(self, red, green, blue):
        msg = LightringLeds()
        self.stamp(msg)
        msg.override_system = True
        msg.leds = [LedColor(red=red, green=green, blue=blue) for _ in range(6)]
        self.light_pub.publish(msg)
        self.lights_on = True

    def release_lights(self):
        """Give the light ring back to the robot's own light patterns."""
        msg = LightringLeds()
        self.stamp(msg)
        msg.override_system = False
        msg.leds = [LedColor(red=0, green=0, blue=0) for _ in range(6)]
        self.light_pub.publish(msg)
        self.lights_on = False

    def beep(self, frequencies):
        msg = AudioNoteVector()
        self.stamp(msg)
        msg.append = False
        msg.notes = [
            AudioNote(frequency=f, max_runtime=Duration(
                sec=0, nanosec=int(BEEP_SECONDS * 1e9)))
            for f in frequencies
        ]
        self.audio_pub.publish(msg)

    def tick(self):
        cmd = Twist()
        if self.now() < self.turn_until:
            cmd.angular.z = self.turn_dir * self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
            if self.lights_on:      # turn finished: lights back to normal
                self.release_lights()
        self.cmd_pub.publish(cmd)

    def stop(self):
        # Send a few zero-velocity commands so the stop gets through
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)
        if self.lights_on:
            self.release_lights()


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
