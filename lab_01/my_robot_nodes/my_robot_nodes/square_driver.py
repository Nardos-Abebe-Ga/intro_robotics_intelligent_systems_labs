#!/usr/bin/env python3
"""Action client that drives the robot in a square."""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate
from nav_msgs.msg import Odometry

class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")

        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(
            self, DriveDistance, "drive_distance"
        )

        self.rotate_client = ActionClient(
            self, Rotate, "rotate"
        )

        self.odom_received = False

        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

    def send_drive_goal(self, distance):
        self.get_logger().info(
            f"Driving forward {distance:.2f} meters"
        )

        goal = DriveDistance.Goal()
        goal.distance = distance

        self.drive_client.wait_for_server()

        future = self.drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Drive goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if not result.reached:
            self.get_logger().error(
                f"Failed to drive distance. Travelled {result.travelled:.2f} m"
            )
            return False

        self.get_logger().info("Drive completed.")
        return True

    def send_rotate_goal(self, angle):
        self.get_logger().info(
            f"Rotating {math.degrees(angle):.1f} degrees"
        )

        goal = Rotate.Goal()
        goal.angle = angle

        self.rotate_client.wait_for_server()

        future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Rotate goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        result = result_future.result().result

        if status != 4:
            self.get_logger().error(
                f"Rotation failed. Turned {result.turned:.2f} rad"
            )
            return False

        self.get_logger().info("Rotation completed.")
        return True

    def drive_square(self):
        side_length = self.get_parameter("side_length").value
        turn_angle = math.pi / 2.0

        self.get_logger().info(
            f"Starting square with side length {side_length:.2f} m"
        )

        for side in range(4):

            self.get_logger().info(f"Starting side {side + 1}")

            if not self.send_drive_goal(side_length):
                self.get_logger().error(
                    f"Square stopped at side {side + 1}: drive failed."
                )
                return

            if not self.send_rotate_goal(turn_angle):
                self.get_logger().error(
                    f"Square stopped at side {side + 1}: rotation failed."
                )
                return

        self.get_logger().info("Square completed successfully!")

    def odom_callback(self, msg):
        self.odom_received = True
def main():
    rclpy.init()

    node = SquareDriver()

    try:
        node.get_logger().info("Waiting for robot odometry...")

        while rclpy.ok() and not node.odom_received:
            rclpy.spin_once(node, timeout_sec=0.5)

        node.get_logger().info(
            "Odometry received! Waiting briefly for drive server..."
        )

        time.sleep(2.0)

        node.drive_square()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
