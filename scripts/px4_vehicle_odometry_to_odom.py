#!/usr/bin/env python3
"""Convert PX4 VehicleOdometry to ROS nav_msgs/Odometry.

PX4 local position is NED and body frame is FRD. ROS odom is ENU and
base_link is FLU.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster


class Px4VehicleOdometryToOdom(Node):
    def __init__(self):
        super().__init__('px4_vehicle_odometry_to_odom')

        self.declare_parameter('input_topic', '/MAV1/fmu/out/vehicle_odometry')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('queue_size', 30)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('stamp_with_ros_time', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        queue_size = int(self.get_parameter('queue_size').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.stamp_with_ros_time = bool(self.get_parameter('stamp_with_ros_time').value)

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )
        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=queue_size)

        self.publisher = self.create_publisher(Odometry, output_topic, pub_qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.subscription = self.create_subscription(VehicleOdometry, input_topic, self.convert, sub_qos)

        self.get_logger().info(
            f'PX4 VehicleOdometry -> Odometry: {input_topic} -> {output_topic}, '
            f'{self.odom_frame_id} -> {self.child_frame_id}'
        )

    def convert(self, msg):
        if not self._valid3(msg.position) or not self._valid4(msg.q):
            return

        out = Odometry()
        if self.stamp_with_ros_time:
            out.header.stamp = self.get_clock().now().to_msg()
        else:
            stamp_us = int(msg.timestamp_sample or msg.timestamp)
            out.header.stamp.sec = stamp_us // 1000000
            out.header.stamp.nanosec = (stamp_us % 1000000) * 1000
        out.header.frame_id = self.odom_frame_id
        out.child_frame_id = self.child_frame_id

        # PX4 local NED -> ROS ENU.
        out.pose.pose.position.x = float(msg.position[1])
        out.pose.pose.position.y = float(msg.position[0])
        out.pose.pose.position.z = -float(msg.position[2])

        # PX4 q is body FRD -> local NED. ROS wants base_link FLU -> odom ENU.
        q_px4 = self._normalize((float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])))
        q_enu_from_ned = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)  # w, x, y, z
        q_frd_from_flu = (0.0, 1.0, 0.0, 0.0)  # 180 deg about X
        q_ros = self._quat_multiply(self._quat_multiply(q_enu_from_ned, q_px4), q_frd_from_flu)
        q_ros = self._normalize(q_ros)
        out.pose.pose.orientation.w = q_ros[0]
        out.pose.pose.orientation.x = q_ros[1]
        out.pose.pose.orientation.y = q_ros[2]
        out.pose.pose.orientation.z = q_ros[3]

        # PX4 velocity is in NED when velocity_frame is LOCAL_FRAME_NED.
        out.twist.twist.linear.x = float(msg.velocity[1])
        out.twist.twist.linear.y = float(msg.velocity[0])
        out.twist.twist.linear.z = -float(msg.velocity[2])

        # PX4 body angular velocity FRD -> ROS FLU.
        out.twist.twist.angular.x = float(msg.angular_velocity[0])
        out.twist.twist.angular.y = -float(msg.angular_velocity[1])
        out.twist.twist.angular.z = -float(msg.angular_velocity[2])

        self._fill_covariance(out, msg)
        self.publisher.publish(out)
        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = out.child_frame_id
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _fill_covariance(out, msg):
        for i in range(36):
            out.pose.covariance[i] = 0.0
            out.twist.covariance[i] = 0.0
        # Diagonal variances after NED -> ENU axis swap.
        out.pose.covariance[0] = float(msg.position_variance[1])
        out.pose.covariance[7] = float(msg.position_variance[0])
        out.pose.covariance[14] = float(msg.position_variance[2])
        out.pose.covariance[21] = float(msg.orientation_variance[0])
        out.pose.covariance[28] = float(msg.orientation_variance[1])
        out.pose.covariance[35] = float(msg.orientation_variance[2])
        out.twist.covariance[0] = float(msg.velocity_variance[1])
        out.twist.covariance[7] = float(msg.velocity_variance[0])
        out.twist.covariance[14] = float(msg.velocity_variance[2])

    @staticmethod
    def _quat_multiply(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    @staticmethod
    def _normalize(q):
        norm = math.sqrt(sum(v * v for v in q))
        if norm <= 1e-9:
            return (1.0, 0.0, 0.0, 0.0)
        return tuple(v / norm for v in q)

    @staticmethod
    def _valid3(values):
        return all(math.isfinite(float(v)) for v in values)

    @staticmethod
    def _valid4(values):
        return all(math.isfinite(float(v)) for v in values)


def main():
    rclpy.init()
    node = Px4VehicleOdometryToOdom()
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
