#!/usr/bin/env python3
"""Monitor SLAM yaw-test topics.

This tool does not control the vehicle. It only prints PX4 odometry, converted
ROS odometry, Cartographer TF, IMU, and depth cloud timing so a pure yaw test can
show which stream diverges first.
"""

import csv
import math
import os
import sys
import time
from datetime import datetime
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
import tf2_ros


class TopicStats:
    def __init__(self, window_sec=3.0):
        self.window_sec = window_sec
        self.times = deque()
        self.last_msg = None
        self.last_stamp = None

    def update(self, now_sec, msg, stamp_sec=None):
        self.last_msg = msg
        self.last_stamp = stamp_sec
        self.times.append(now_sec)
        while self.times and now_sec - self.times[0] > self.window_sec:
            self.times.popleft()

    def hz(self):
        if len(self.times) < 2:
            return 0.0
        dt = self.times[-1] - self.times[0]
        if dt <= 0.0:
            return 0.0
        return (len(self.times) - 1) / dt


class YawTopicCheck(Node):
    def __init__(self):
        super().__init__('check_yaw_topics')

        self.declare_parameter('px4_odom_topic', '/MAV1/fmu/out/vehicle_odometry')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('points_topic', '/MAV1/camera_front/depth/points')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('print_period_sec', 1.0)
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('stale_age_sec', 0.15)
        self.declare_parameter('output_file', 'auto')

        self.px4_odom_topic = self.get_parameter('px4_odom_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.points_topic = self.get_parameter('points_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        print_period = float(self.get_parameter('print_period_sec').value)
        self.clock_topic = self.get_parameter('clock_topic').value
        self.stale_age_sec = float(self.get_parameter('stale_age_sec').value)
        self.output_file = self._resolve_output_file(self.get_parameter('output_file').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.px4_stats = TopicStats()
        self.odom_stats = TopicStats()
        self.imu_stats = TopicStats()
        self.points_stats = TopicStats()
        self.clock_stats = TopicStats()

        self.last_point_stamp_sec = None
        self.last_point_arrival_wall = None
        self.point_header_dt = None
        self.point_arrival_dt = None
        self.point_age = None
        self.point_stamp_sec = None

        self.csv_file = None
        self.csv_writer = None
        if self.output_file:
            os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
            self.csv_file = open(self.output_file, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self._csv_fields())
            self.csv_writer.writeheader()
            self.csv_file.flush()

        self.create_subscription(VehicleOdometry, self.px4_odom_topic, self._on_px4_odom, qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)
        self.create_subscription(PointCloud2, self.points_topic, self._on_points, qos)
        self.create_subscription(Clock, self.clock_topic, self._on_clock, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(print_period, self._print_status)

        self.get_logger().info(
            'Monitoring yaw/timing topics: '
            f'px4_odom={self.px4_odom_topic}, odom={self.odom_topic}, '
            f'imu={self.imu_topic}, points={self.points_topic}, clock={self.clock_topic}, '
            f'tf={self.map_frame}->{self.base_frame}, output_file={self.output_file}'
        )

    def _resolve_output_file(self, value):
        if value and value != 'auto':
            return value
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(
            '/home/zhg/ncrl_mqtt/catkin_ws/src/drone_nav2_apriltag/docs',
            f'check_yaw_topics_{stamp}.csv',
        )

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_px4_odom(self, msg):
        stamp = self._px4_stamp_sec(msg.timestamp_sample or msg.timestamp)
        self.px4_stats.update(self._now_sec(), msg, stamp)

    def _on_odom(self, msg):
        self.odom_stats.update(self._now_sec(), msg, self._ros_stamp_sec(msg.header.stamp))

    def _on_imu(self, msg):
        self.imu_stats.update(self._now_sec(), msg, self._ros_stamp_sec(msg.header.stamp))

    def _on_points(self, msg):
        stamp_sec = self._ros_stamp_sec(msg.header.stamp)
        self.point_stamp_sec = stamp_sec

        if self.last_point_stamp_sec is not None:
            self.point_header_dt = stamp_sec - self.last_point_stamp_sec
        self.last_point_stamp_sec = stamp_sec

        arrival_wall = time.monotonic()
        if self.last_point_arrival_wall is not None:
            self.point_arrival_dt = arrival_wall - self.last_point_arrival_wall
        self.last_point_arrival_wall = arrival_wall

        if self.clock_stats.last_stamp is not None:
            self.point_age = self.clock_stats.last_stamp - stamp_sec

        self.points_stats.update(self._now_sec(), msg, stamp_sec)

    def _on_clock(self, msg):
        stamp_sec = self._ros_stamp_sec(msg.clock)
        self.clock_stats.update(time.monotonic(), msg, stamp_sec)
        if self.point_stamp_sec is not None:
            self.point_age = stamp_sec - self.point_stamp_sec

    def _print_status(self):
        px4_yaw = self._px4_yaw_deg()
        odom_yaw = self._odom_yaw_deg()
        tf_yaw = self._tf_yaw_deg()
        imu_accel = self._imu_accel()
        frames = self._frames()
        timing = self._timing_summary()
        point_info = self._point_timing_summary()
        row = self._make_csv_row(px4_yaw, odom_yaw, tf_yaw)
        self._write_csv(row)

        self.get_logger().info(
            f'hz px4={self.px4_stats.hz():5.1f} odom={self.odom_stats.hz():5.1f} '
            f'imu={self.imu_stats.hz():5.1f} points={self.points_stats.hz():5.1f} | '
            f'yaw_deg px4={self._fmt(px4_yaw)} odom={self._fmt(odom_yaw)} '
            f'map_tf={self._fmt(tf_yaw)} | {timing} | {point_info} | {frames} | {imu_accel}'
        )

    def _px4_yaw_deg(self):
        msg = self.px4_stats.last_msg
        if msg is None or not self._valid4(msg.q):
            return None
        q_px4 = self._normalize((float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])))
        q_enu_from_ned = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
        q_frd_from_flu = (0.0, 1.0, 0.0, 0.0)
        q_ros = self._quat_multiply(self._quat_multiply(q_enu_from_ned, q_px4), q_frd_from_flu)
        return self._yaw_from_wxyz_deg(q_ros)

    def _odom_yaw_deg(self):
        msg = self.odom_stats.last_msg
        if msg is None:
            return None
        q = msg.pose.pose.orientation
        return self._yaw_from_wxyz_deg((q.w, q.x, q.y, q.z))

    def _tf_yaw_deg(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        return self._yaw_from_wxyz_deg((q.w, q.x, q.y, q.z))

    def _imu_accel(self):
        msg = self.imu_stats.last_msg
        if msg is None:
            return 'imu_accel=none'
        a = msg.linear_acceleration
        return f'imu_accel=({a.x:+.2f},{a.y:+.2f},{a.z:+.2f})'

    def _frames(self):
        parts = []
        if self.odom_stats.last_msg is not None:
            parts.append(f'odom_frame={self.odom_stats.last_msg.header.frame_id}->{self.odom_stats.last_msg.child_frame_id}')
        if self.imu_stats.last_msg is not None:
            parts.append(f'imu_frame={self.imu_stats.last_msg.header.frame_id}')
        if self.points_stats.last_msg is not None:
            parts.append(f'points_frame={self.points_stats.last_msg.header.frame_id}')
        return ' '.join(parts) if parts else 'frames=none'

    def _point_timing_summary(self):
        parts = []
        if self.point_header_dt is not None:
            parts.append(f'point_dt_header={self.point_header_dt:+.3f}s')
        if self.point_arrival_dt is not None:
            parts.append(f'point_dt_arrival={self.point_arrival_dt:+.3f}s')
        if self.point_age is not None:
            stale = ' STALE' if self.point_age > self.stale_age_sec else ''
            parts.append(f'point_age={self.point_age:+.3f}s{stale}')
        if self.point_stamp_sec is not None:
            parts.append(f'point_stamp={self.point_stamp_sec:.3f}')
        return ' '.join(parts) if parts else 'point_timing=waiting'

    def _make_csv_row(self, px4_yaw, odom_yaw, tf_yaw):
        return {
            'node_time_sec': self._now_sec(),
            'clock_sec': self.clock_stats.last_stamp,
            'px4_hz': self.px4_stats.hz(),
            'odom_hz': self.odom_stats.hz(),
            'imu_hz': self.imu_stats.hz(),
            'points_hz': self.points_stats.hz(),
            'px4_yaw_deg': px4_yaw,
            'odom_yaw_deg': odom_yaw,
            'map_tf_yaw_deg': tf_yaw,
            'points_minus_imu_sec': self._stamp_delta(self.points_stats, self.imu_stats),
            'points_minus_odom_sec': self._stamp_delta(self.points_stats, self.odom_stats),
            'odom_minus_imu_sec': self._stamp_delta(self.odom_stats, self.imu_stats),
            'point_dt_header_sec': self.point_header_dt,
            'point_dt_arrival_sec': self.point_arrival_dt,
            'point_age_sec': self.point_age,
            'point_stale': bool(self.point_age is not None and self.point_age > self.stale_age_sec),
            'point_stamp_sec': self.point_stamp_sec,
            'odom_frame': self._odom_frame_text(),
            'imu_frame': self.imu_stats.last_msg.header.frame_id if self.imu_stats.last_msg is not None else '',
            'points_frame': self.points_stats.last_msg.header.frame_id if self.points_stats.last_msg is not None else '',
        }

    def _write_csv(self, row):
        if self.csv_writer is None:
            return
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _odom_frame_text(self):
        msg = self.odom_stats.last_msg
        if msg is None:
            return ''
        return f'{msg.header.frame_id}->{msg.child_frame_id}'

    @staticmethod
    def _stamp_delta(a, b):
        if a.last_stamp is None or b.last_stamp is None:
            return None
        return a.last_stamp - b.last_stamp

    @staticmethod
    def _csv_fields():
        return [
            'node_time_sec', 'clock_sec',
            'px4_hz', 'odom_hz', 'imu_hz', 'points_hz',
            'px4_yaw_deg', 'odom_yaw_deg', 'map_tf_yaw_deg',
            'points_minus_imu_sec', 'points_minus_odom_sec', 'odom_minus_imu_sec',
            'point_dt_header_sec', 'point_dt_arrival_sec', 'point_age_sec',
            'point_stale', 'point_stamp_sec',
            'odom_frame', 'imu_frame', 'points_frame',
        ]

    def _timing_summary(self):
        imu = self.imu_stats.last_stamp
        odom = self.odom_stats.last_stamp
        points = self.points_stats.last_stamp
        parts = []
        if imu is not None and points is not None:
            parts.append(f'points-imu={points - imu:+.3f}s')
        if odom is not None and points is not None:
            parts.append(f'points-odom={points - odom:+.3f}s')
        if imu is not None and odom is not None:
            parts.append(f'odom-imu={odom - imu:+.3f}s')
        return ' '.join(parts) if parts else 'timing=waiting'

    def destroy_node(self):
        if self.csv_file is not None:
            self.get_logger().info(f'Wrote topic diagnostics CSV: {self.output_file}')
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()

    @staticmethod
    def _fmt(value):
        return ' none' if value is None else f'{value:+6.1f}'

    @staticmethod
    def _ros_stamp_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _px4_stamp_sec(stamp_us):
        return float(stamp_us) * 1e-6

    @staticmethod
    def _yaw_from_wxyz_deg(q):
        w, x, y, z = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

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
    def _valid4(values):
        return all(math.isfinite(float(v)) for v in values)


def main():
    rclpy.init()
    node = YawTopicCheck()
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
