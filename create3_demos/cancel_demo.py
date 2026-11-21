#!/usr/bin/env python3
"""Cancel an action goal while it is running.

This node asks the robot to drive a long way forward (/drive_distance), then
CANCELS the goal part of the way through:

  * automatically after `cancel_after` seconds, or
  * when you press Enter in the terminal (whichever happens first).

When a goal is cancelled the robot stops, and the action server reports the
status CANCELED instead of SUCCEEDED. Topics and services have no equivalent:
this is something only actions can do.
"""
import select
import sys
import termios
import time

import rclpy
from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import DriveDistance, Undock
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class CancelDemo(Node):
    def __init__(self):
        super().__init__('cancel_demo')

        # dynamic_typing lets `-p cancel_after:=0` (an integer) work as well as `0.0`
        number = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('distance', 1.0, number)        # metres to ask for
        self.declare_parameter('speed', 0.1, number)           # m/s (maximum)
        self.declare_parameter('cancel_after', 4.0, number)    # seconds; 0 = only cancel with Enter
        self.declare_parameter('undock_first', False)  # start by undocking
        self.declare_parameter('server_timeout', 10.0)
        self.distance = float(self.get_parameter('distance').value)
        self.speed = float(self.get_parameter('speed').value)
        self.cancel_after = float(self.get_parameter('cancel_after').value)
        self.undock_first = self.get_parameter('undock_first').value
        self.server_timeout = self.get_parameter('server_timeout').value

        self.drive_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.undock_client = ActionClient(self, Undock, 'undock')
        self.goal_handle = None            # the goal that is running
        self.remaining = None         # last remaining distance reported by the robot

    def on_feedback(self, feedback_msg):
        self.remaining = feedback_msg.feedback.remaining_travel_distance
        self.get_logger().info(f'  {self.remaining:.2f} m to go')

    def enter_pressed(self):
        """True if the user pressed Enter (never blocks)."""
        if not sys.stdin.isatty():
            return False
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if ready:
            sys.stdin.readline()
            return True
        return False

    def undock(self):
        if not self.undock_client.wait_for_server(timeout_sec=self.server_timeout):
            self.get_logger().error('action server /undock not available')
            return False
        self.get_logger().info('undocking')
        future = self.undock_client.send_goal_async(Undock.Goal())
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('the robot rejected the undock goal')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(0.5)
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def cancel(self):
        """Ask the robot to cancel the running goal and wait for its answer."""
        self.get_logger().info('>>> cancelling the goal')
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def run(self):
        if not self.drive_client.wait_for_server(timeout_sec=self.server_timeout):
            self.get_logger().error('action server /drive_distance not available. '
                                    'Is the robot connected?')
            return False
        if self.undock_first and not self.undock():
            return False

        goal = DriveDistance.Goal()
        goal.distance = float(self.distance)
        goal.max_translation_speed = float(self.speed)

        self.get_logger().info(f'asking the robot to drive {self.distance} m')
        send_future = self.drive_client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().error('the robot rejected the goal')
            return False

        if self.cancel_after > 0.0:
            self.get_logger().info(
                f'will cancel after {self.cancel_after:.1f} s, or press Enter to cancel now')
        else:
            self.get_logger().info('press Enter to cancel')

        # Ignore Enter presses from before the goal was accepted (that can take a few seconds).
        if sys.stdin.isatty():
            termios.tcflush(sys.stdin, termios.TCIFLUSH)

        result_future = self.goal_handle.get_result_async()
        start = time.monotonic()
        cancelled = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            timed_out = self.cancel_after > 0.0 and time.monotonic() - start >= self.cancel_after
            if not cancelled and (timed_out or self.enter_pressed()):
                self.cancel()
                cancelled = True

        if not result_future.done():
            return False
        status = result_future.result().status
        name = STATUS_NAMES.get(status, f'status {status}')
        self.get_logger().info(f'the robot says: {name}')
        if self.remaining is not None and status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                f'at its last report it had {self.remaining:.2f} m of the {self.distance} m left '
                '(the robot only reports every few seconds, so the real value is a bit lower)')
        return True

    def cancel_current(self):
        """Used on Ctrl+C: make sure the robot is not left driving."""
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CancelDemo()
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
