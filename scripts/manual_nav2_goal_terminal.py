#!/usr/bin/env python3
"""Terminal helper that publishes manual ENU start/goal poses for Nav2 flight."""

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quat(yaw):
    half = yaw * 0.5
    pose = Pose()
    pose.orientation.z = math.sin(half)
    pose.orientation.w = math.cos(half)
    return pose.orientation


class ManualNav2GoalTerminal(Node):
    def __init__(self):
        super().__init__("manual_nav2_goal_terminal")
        self.topic = self.declare_parameter("goal_topic", "/manual_nav2_start_goal").value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(PoseArray, self.topic, qos)

    def parse_pose(self, text):
        parts = [float(x) for x in text.replace(",", " ").split()]
        if len(parts) != 3:
            raise ValueError("expected: east north yaw_deg")
        pose = Pose()
        pose.position.x = parts[0]
        pose.position.y = parts[1]
        pose.position.z = 0.0
        pose.orientation = yaw_to_quat(math.radians(parts[2]))
        return pose

    def run_once(self):
        print("輸入起點 ENU 座標：east north yaw_deg，例如 0 0 0")
        start = self.parse_pose(input("start> "))
        print("輸入終點 ENU 座標：east north yaw_deg，例如 5 0 0")
        goal = self.parse_pose(input("goal> "))

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.poses = [start, goal]
        self.publisher.publish(msg)
        self.get_logger().info(f"published start/goal to {self.topic}")


def main():
    rclpy.init()
    node = ManualNav2GoalTerminal()
    try:
        node.run_once()
        rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
