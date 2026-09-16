#!/usr/bin/env python3
"""Check the live inputs used by Cartographer depth SLAM."""

import math
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
import tf2_ros


class SlamInputCheck(Node):
    def __init__(self):
        super().__init__('check_slam_inputs')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('points_topic', '/cartographer/depth_points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_front_link')
        self.declare_parameter('timeout_sec', 5.0)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.points_topic = self.get_parameter('points_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.imu_msg = None
        self.cloud_msg = None
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)
        self.create_subscription(PointCloud2, self.points_topic, self._on_cloud, qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_imu(self, msg):
        self.imu_msg = msg

    def _on_cloud(self, msg):
        self.cloud_msg = msg

    def run(self):
        deadline = self.get_clock().now().nanoseconds / 1e9 + self.timeout_sec
        while rclpy.ok() and self.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.imu_msg is not None and self.cloud_msg is not None:
                break

        ok = True
        if self.imu_msg is None:
            self.get_logger().error(f'No IMU received on {self.imu_topic}')
            ok = False
        else:
            a = self.imu_msg.linear_acceleration
            self.get_logger().info(
                f'IMU {self.imu_topic}: frame={self.imu_msg.header.frame_id}, '
                f'accel=({a.x:.3f}, {a.y:.3f}, {a.z:.3f}) m/s^2'
            )
            if self.imu_msg.header.frame_id != self.base_frame:
                self.get_logger().error(f'IMU frame should be {self.base_frame}')
                ok = False
            if a.z < 0.0:
                self.get_logger().error('IMU z is negative. PX4 FRD was not converted to ROS FLU.')
                ok = False

        if self.cloud_msg is None:
            self.get_logger().error(f'No PointCloud2 received on {self.points_topic}')
            ok = False
        else:
            self.get_logger().info(
                f'Cloud {self.points_topic}: frame={self.cloud_msg.header.frame_id}, '
                f'width={self.cloud_msg.width}, height={self.cloud_msg.height}, '
                f'point_step={self.cloud_msg.point_step}'
            )
            if self.cloud_msg.header.frame_id != self.camera_frame:
                self.get_logger().error(f'Cloud frame should be {self.camera_frame}')
                ok = False
            bounds = self._cloud_bounds(self.cloud_msg)
            if bounds is not None:
                self.get_logger().info(
                    'Cloud xyz bounds in its frame: '
                    f'x=[{bounds[0][0]:.2f}, {bounds[0][1]:.2f}], '
                    f'y=[{bounds[1][0]:.2f}, {bounds[1][1]:.2f}], '
                    f'z=[{bounds[2][0]:.2f}, {bounds[2][1]:.2f}]'
                )

        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            self.get_logger().info(
                f'TF {self.base_frame} -> {self.camera_frame}: '
                f'trans=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), '
                f'quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
            )
        except Exception as exc:
            self.get_logger().error(f'Cannot lookup TF {self.base_frame} -> {self.camera_frame}: {exc}')
            ok = False

        if ok:
            self.get_logger().info('SLAM input check passed.')
            return 0
        return 1

    @staticmethod
    def _cloud_bounds(msg):
        offsets = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z') and f.datatype == PointField.FLOAT32}
        if set(offsets) != {'x', 'y', 'z'} or msg.point_step <= 0:
            return None
        endian = '>' if msg.is_bigendian else '<'
        total = msg.width * msg.height
        stride = max(1, total // 20000)
        mins = [math.inf, math.inf, math.inf]
        maxs = [-math.inf, -math.inf, -math.inf]
        found = False
        data = msg.data
        for idx in range(0, total, stride):
            base = (idx // msg.width) * msg.row_step + (idx % msg.width) * msg.point_step
            vals = []
            for axis in ('x', 'y', 'z'):
                vals.append(struct.unpack_from(endian + 'f', data, base + offsets[axis])[0])
            if not all(math.isfinite(v) for v in vals):
                continue
            found = True
            for i, v in enumerate(vals):
                mins[i] = min(mins[i], v)
                maxs[i] = max(maxs[i], v)
        if not found:
            return None
        return tuple(zip(mins, maxs))


def main():
    rclpy.init()
    node = SlamInputCheck()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
