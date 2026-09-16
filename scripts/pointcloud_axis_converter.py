#!/usr/bin/env python3
"""Rewrite or convert PointCloud2 axes before feeding SLAM.

The default mode is identity because Gazebo native depth/points may already
publish point coordinates in camera link axes. Use mode=optical_to_link only
when the numeric points are truly ROS optical axes:
  optical: x right, y down, z forward
  link:    x forward, y left, z up
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudAxisConverter(Node):
    def __init__(self):
        super().__init__('pointcloud_axis_converter')

        self.declare_parameter('input_topic', '/cartographer/depth_points_raw')
        self.declare_parameter('output_topic', '/cartographer/depth_points')
        self.declare_parameter('output_frame_id', 'camera_front_link')
        self.declare_parameter('mode', 'identity')
        self.declare_parameter('queue_size', 1)

        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.mode = self.get_parameter('mode').value
        queue_size = int(self.get_parameter('queue_size').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        valid_modes = ('identity', 'optical_to_link')
        if self.mode not in valid_modes:
            raise ValueError(f"mode must be one of {valid_modes}, got {self.mode!r}")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.convert_cloud,
            qos,
        )

        detail = ' formula: x=x_in, y=y_in, z=z_in'
        if self.mode == 'optical_to_link':
            detail = ' formula: x=z_optical, y=-x_optical, z=-y_optical'
        self.get_logger().info(
            f'PointCloud2 mode={self.mode}: {input_topic} -> {output_topic}, '
            f'output_frame_id={self.output_frame_id}{detail}'
        )

    def convert_cloud(self, msg):
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense

        if self.mode == 'identity':
            out.data = msg.data
            self.publisher.publish(out)
            return

        dtype = self._cloud_dtype(msg)
        if dtype is None:
            self.get_logger().error('Input cloud must contain float32 x, y, z fields')
            return

        data = bytearray(msg.data)
        cloud = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=data,
            strides=(msg.row_step, msg.point_step),
        )

        x_optical = cloud['x'].copy()
        y_optical = cloud['y'].copy()
        z_optical = cloud['z'].copy()

        cloud['x'] = z_optical
        cloud['y'] = -x_optical
        cloud['z'] = -y_optical

        out.data = bytes(data)
        self.publisher.publish(out)

    @staticmethod
    def _cloud_dtype(msg):
        endian = '>' if msg.is_bigendian else '<'
        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                if field.datatype != PointField.FLOAT32 or field.count != 1:
                    return None
                offsets[field.name] = field.offset

        if not all(name in offsets for name in ('x', 'y', 'z')):
            return None

        return np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': msg.point_step,
        })


def main():
    rclpy.init()
    node = PointCloudAxisConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
