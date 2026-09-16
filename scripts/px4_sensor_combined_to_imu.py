#!/usr/bin/env python3
"""Convert PX4 SensorCombined raw IMU to ROS sensor_msgs/Imu.

PX4 body frame is FRD (x forward, y right, z down). ROS base_link is FLU
(x forward, y left, z up), so y and z are negated.
"""

import math
import sys

import rclpy
from px4_msgs.msg import SensorCombined
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu


class Px4SensorCombinedToImu(Node):
    def __init__(self):
        super().__init__('px4_sensor_combined_to_imu')

        self.declare_parameter('input_topic', '/MAV1/fmu/out/sensor_combined')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('stamp_mode', 'px4_timestamp')
        self.declare_parameter('queue_size', 10)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.stamp_mode = self.get_parameter('stamp_mode').value
        queue_size = int(self.get_parameter('queue_size').value)

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )
        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=queue_size)

        self.publisher = self.create_publisher(Imu, output_topic, pub_qos)
        self.subscription = self.create_subscription(SensorCombined, input_topic, self.convert, sub_qos)

        self.get_logger().info(
            f'PX4 SensorCombined -> Imu: {input_topic} -> {output_topic}, '
            f'frame_id={self.frame_id}, stamp_mode={self.stamp_mode}'
        )

    def convert(self, msg):
        if not self._valid3(msg.gyro_rad) or not self._valid3(msg.accelerometer_m_s2):
            return

        out = Imu()
        if self.stamp_mode != 'px4_timestamp':
            self.get_logger().warn(
                f"Unsupported stamp_mode='{self.stamp_mode}', using px4_timestamp",
                throttle_duration_sec=2.0,
            )
        # PX4 SensorCombined timestamp is in microseconds. When PX4 uXRCE-DDS
        # timestamp sync is disabled, this remains on the PX4/sim time axis.
        stamp_us = int(msg.timestamp)
        out.header.stamp.sec = stamp_us // 1000000
        out.header.stamp.nanosec = (stamp_us % 1000000) * 1000
        out.header.frame_id = self.frame_id

        # PX4 FRD -> ROS FLU.
        out.angular_velocity.x = float(msg.gyro_rad[0])
        out.angular_velocity.y = -float(msg.gyro_rad[1])
        out.angular_velocity.z = -float(msg.gyro_rad[2])
        out.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        out.linear_acceleration.y = -float(msg.accelerometer_m_s2[1])
        out.linear_acceleration.z = -float(msg.accelerometer_m_s2[2])

        # SensorCombined has no absolute orientation.
        out.orientation_covariance[0] = -1.0
        self.publisher.publish(out)

    @staticmethod
    def _valid3(values):
        return all(math.isfinite(float(v)) for v in values)


def main():
    rclpy.init()
    node = Px4SensorCombinedToImu()
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
