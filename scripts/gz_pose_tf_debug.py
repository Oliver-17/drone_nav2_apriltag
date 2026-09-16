#!/usr/bin/env python3
"""Publish a separate ground-truth TF from Gazebo /world/*/pose/info.

This node is for debugging only. It listens to the TFMessage produced by
ros_gz_bridge from gz.msgs.Pose_V, picks the requested Gazebo pose, then republishes
it as world -> gt_base_link so it does not interfere with SLAM frames.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class GzPoseTfDebug(Node):
    def __init__(self):
        super().__init__('gz_pose_tf_debug')
        self.declare_parameter('input_topic', '/gz_pose_info')
        self.declare_parameter('target_name', 'x500_depth_nav2_0')
        self.declare_parameter('target_index', 0)
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'gt_base_link')
        self.declare_parameter('queue_size', 10)

        input_topic = self.get_parameter('input_topic').value
        self.target_name = self.get_parameter('target_name').value
        self.target_index = int(self.get_parameter('target_index').value)
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        queue_size = int(self.get_parameter('queue_size').value)
        self.last_candidates = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, input_topic, self._on_pose_info, qos)
        self.create_timer(2.0, self._log_candidates_if_needed)
        self.matched = False

        self.get_logger().info(
            f'Gazebo pose debug: {input_topic}, target contains {self.target_name!r}, '
            f'target_index={self.target_index}, publishing {self.parent_frame} -> {self.child_frame}'
        )

    def _on_pose_info(self, msg):
        self.last_candidates = [tf.child_frame_id for tf in msg.transforms[:30]]
        selected = None
        selected_index = None
        for index, tf in enumerate(msg.transforms):
            if self.target_name and self.target_name in tf.child_frame_id:
                selected = tf
                selected_index = index
                if 'base_link' in tf.child_frame_id:
                    break

        if selected is None:
            if 0 <= self.target_index < len(msg.transforms):
                selected = msg.transforms[self.target_index]
                selected_index = self.target_index
            else:
                self.matched = False
                return

        source_name = selected.child_frame_id or f'index {selected_index}'
        out = selected
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        self.broadcaster.sendTransform(out)
        t = out.transform.translation
        if not self.matched:
            self.get_logger().info(f'Matched Gazebo pose source: {source_name}')
        self.get_logger().info(
            f'GT {self.parent_frame}->{self.child_frame}: '
            f'x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}',
            throttle_duration_sec=1.0,
        )
        self.matched = True

    def _log_candidates_if_needed(self):
        if self.matched:
            return
        if not self.last_candidates:
            self.get_logger().warn('No Gazebo pose TFMessage received yet')
            return
        self.get_logger().warn(
            'No pose matched target_name. First candidates: ' + ', '.join(self.last_candidates[:10])
        )


def main():
    rclpy.init()
    node = GzPoseTfDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received Ctrl+C')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
