#!/usr/bin/env python3
"""A mission built from the Create 3's own action servers.

Instead of publishing speeds to /cmd_vel and watching /odom, this node sends the
robot GOALS and waits for the robot to report back:

    /undock          (optional, first)
    /drive_distance  -> drive `side_length` metres forward
    /rotate_angle    -> turn left 90 degrees
    ...repeated `sides` times (a square)
    /dock            (optional, last)

The robot measures distance and angle itself, so a slow or lossy Wi-Fi link
does not make the moves sloppy. While a move runs, the robot streams FEEDBACK
("0.12 m to go"), which this node logs about once a second. Ctrl+C cancels the
goal that is running.
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import Dock, DriveDistance, RotateAngle, Undock
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class ActionMission(Node):
    def __init__(self):
        super().__init__('action_mission')

        self.declare_parameter('side_length', 0.5)       # metres per side
        self.declare_parameter('sides', 4)
        self.declare_parameter('drive_speed', 0.1)       # m/s (maximum)
        self.declare_parameter('turn_speed', 0.5)        # rad/s (maximum)
        self.declare_parameter('undock_first', False)    # start by undocking
        self.declare_parameter('dock_at_end', False)     # finish by docking
        self.declare_parameter('pause_time', 0.5)        # seconds between steps
        self.declare_parameter('server_timeout', 10.0)   # seconds to wait for servers
        self.declare_parameter('feedback_period', 1.0)   # seconds between progress logs (0 = off)
        self.side_length = self.get_parameter('side_length').value
        self.sides = self.get_parameter('sides').value
        self.drive_speed = self.get_parameter('drive_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.undock_first = self.get_parameter('undock_first').value
        self.dock_at_end = self.get_parameter('dock_at_end').value
        self.pause_time = self.get_parameter('pause_time').value
        self.server_timeout = self.get_parameter('server_timeout').value
        self.feedback_period = self.get_parameter('feedback_period').value

        self.undock_client = ActionClient(self, Undock, 'undock')
        self.dock_client = ActionClient(self, Dock, 'dock')
        self.drive_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.rotate_client = ActionClient(self, RotateAngle, 'rotate_angle')
        self.current_goal = None   # handle of the goal that is running

    def wait_for_servers(self):
        clients = [('drive_distance', self.drive_client),
                   ('rotate_angle', self.rotate_client)]
        if self.undock_first:
            clients.append(('undock', self.undock_client))
        for name, client in clients:
            if not client.wait_for_server(timeout_sec=self.server_timeout):
                self.get_logger().error(
                    f'action server /{name} not available. Is the robot connected?')
                return False
        # Docking is optional: older robot firmware names this action DockServo,
        # which does not match the Dock action type, so skip it instead of failing.
        if self.dock_at_end and not self.dock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warning(
                'the /dock action does not match the robot (older firmware uses '
                'DockServo). Docking will be skipped; put the robot on its dock by hand.')
            self.dock_at_end = False
        return True

    def make_feedback_callback(self, describe):
        """Build a callback that logs the robot's progress, at most once per period."""
        last_log = [0.0]   # a list so the callback can change it

        def callback(feedback_msg):
            now = time.monotonic()
            if now - last_log[0] < self.feedback_period:
                return
            last_log[0] = now
            # Progress messages are nice to have: a problem here must never stop the mission.
            try:
                text = describe(feedback_msg.feedback)
            except (AttributeError, TypeError, ValueError) as error:
                self.get_logger().warning(f'cannot read feedback ({error}); progress messages off')
                self.feedback_period = 0.0
                return
            self.get_logger().info(f'  {text}')
        return callback

    def send(self, client, goal, label, describe=None):
        """Send one goal and wait for it to finish. Returns True if it succeeded.

        `describe` turns the robot's feedback message into a short text, e.g.
        "0.12 m to go". Leave it out for goals with nothing useful to report.
        """
        self.get_logger().info(label)
        feedback_cb = None
        if describe is not None and self.feedback_period > 0.0:
            feedback_cb = self.make_feedback_callback(describe)
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{label}: the robot rejected the goal')
            return False

        self.current_goal = handle
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.current_goal = None

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'{label}: did not succeed (status {status}), stopping the mission')
            return False
        return True

    def drive(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = float(distance)
        goal.max_translation_speed = float(self.drive_speed)
        return self.send(
            self.drive_client, goal, f'drive {distance} m',
            describe=lambda fb: f'{fb.remaining_travel_distance:.2f} m to go')

    def rotate(self, angle):
        goal = RotateAngle.Goal()
        goal.angle = float(angle)
        goal.max_rotation_speed = float(self.turn_speed)
        return self.send(
            self.rotate_client, goal, f'rotate {math.degrees(angle):.0f} degrees',
            describe=lambda fb: (
                f'{math.degrees(abs(fb.remaining_angle_travel)):.0f} degrees to go'))

    def run(self):
        if not self.wait_for_servers():
            return False
        if self.undock_first:
            if not self.send(self.undock_client, Undock.Goal(), 'undocking'):
                return False
            time.sleep(self.pause_time)
        for i in range(self.sides):
            self.get_logger().info(f'side {i + 1}/{self.sides}')
            if not self.drive(self.side_length):
                return False
            time.sleep(self.pause_time)
            if not self.rotate(math.pi / 2):   # positive angle = turn left
                return False
            time.sleep(self.pause_time)
        if self.dock_at_end:
            if not self.send(self.dock_client, Dock.Goal(), 'docking'):
                return False
        self.get_logger().info('mission finished')
        return True

    def cancel_current(self):
        """Ask the robot to cancel whatever goal is running (used on Ctrl+C)."""
        if self.current_goal is not None:
            self.get_logger().info('cancelling the current goal')
            cancel_future = self.current_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            self.current_goal = None


def main():
    # Python raises KeyboardInterrupt on Ctrl+C, so we can still cancel the goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ActionMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cancel_current()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
