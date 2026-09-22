#!/usr/bin/env python3
"""Check MAV1 PX4/OptiTrack topics before running the Nav2 flight flow."""

import math
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition, VehicleOdometry, VehicleStatus
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class TopicState:
    stamp_sec: Optional[float] = None
    count: int = 0
    msg: object = None

    def update(self, node: Node, msg: object) -> None:
        self.stamp_sec = node.get_clock().now().nanoseconds / 1e9
        self.count += 1
        self.msg = msg

    def age(self, node: Node) -> Optional[float]:
        if self.stamp_sec is None:
            return None
        now_sec = node.get_clock().now().nanoseconds / 1e9
        return max(0.0, now_sec - self.stamp_sec)

    def fresh(self, node: Node, timeout: float) -> bool:
        age = self.age(node)
        return age is not None and age <= timeout


def px4_ned_position_to_ros_enu(msg: VehicleOdometry):
    return (
        float(msg.position[1]),
        float(msg.position[0]),
        -float(msg.position[2]),
    )


def px4_ned_velocity_to_ros_enu(msg: VehicleOdometry):
    return (
        float(msg.velocity[1]),
        float(msg.velocity[0]),
        -float(msg.velocity[2]),
    )


class Mav1Nav2TopicCheck(Node):
    def __init__(self):
        super().__init__("mav1_nav2_topic_check")

        p = self.declare_parameter
        self.px4_namespace = p("px4_namespace", "/MAV1").value.rstrip("/")
        self.mocap_pose_topic = p("mocap_pose_topic", "/vrpn_mocap/MAV1/pose_reliable").value
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.map_frame = p("map_frame", "map").value
        self.topic_timeout = float(p("topic_timeout", 1.0).value)
        self.tf_timeout = float(p("tf_timeout", 0.2).value)
        self.report_period = float(p("report_period", 1.0).value)
        self.exit_when_ready = bool(p("exit_when_ready", False).value)
        self.require_mocap = bool(p("require_mocap", False).value)
        self.require_yaw_good = bool(p("require_yaw_good", False).value)

        self.vehicle_odom = TopicState()
        self.local_pos = TopicState()
        self.estimator = TopicState()
        self.status = TopicState()
        self.mocap = TopicState()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        ns = self.px4_namespace
        self.create_subscription(
            VehicleOdometry,
            f"{ns}/fmu/out/vehicle_odometry",
            lambda msg: self.vehicle_odom.update(self, msg),
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"{ns}/fmu/out/vehicle_local_position_v1",
            lambda msg: self.local_pos.update(self, msg),
            qos,
        )
        self.create_subscription(
            EstimatorStatusFlags,
            f"{ns}/fmu/out/estimator_status_flags",
            lambda msg: self.estimator.update(self, msg),
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            f"{ns}/fmu/out/vehicle_status_v1",
            lambda msg: self.status.update(self, msg),
            qos,
        )
        self.create_subscription(
            PoseStamped,
            self.mocap_pose_topic,
            lambda msg: self.mocap.update(self, msg),
            qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(self.report_period, self.report)

        self.get_logger().info("MAV1 Nav2 topic check started")
        self.get_logger().info(f"PX4 namespace: {self.px4_namespace}")
        self.get_logger().info(f"Mocap topic  : {self.mocap_pose_topic}")
        self.get_logger().info("This node only checks topics/TF. It does not send PX4 commands.")

    def report(self):
        topic_ok = self._topics_ok()
        estimator_ok = self._estimator_ok()
        tf_ok = self._tf_ok()
        ready = topic_ok and estimator_ok and tf_ok

        lines = []
        lines.append("=" * 64)
        lines.append(f"Nav2 readiness: {'READY' if ready else 'NOT READY'}")
        lines.append(self._topic_line("vehicle_odometry", self.vehicle_odom))
        lines.append(self._topic_line("vehicle_local_position_v1", self.local_pos))
        lines.append(self._topic_line("estimator_status_flags", self.estimator))
        lines.append(self._topic_line("vehicle_status_v1", self.status))
        lines.append(self._topic_line("mocap_pose", self.mocap))
        lines.extend(self._vehicle_odom_lines())
        lines.extend(self._local_position_lines())
        lines.extend(self._estimator_lines())
        lines.extend(self._status_lines())
        lines.extend(self._tf_lines())
        self.get_logger().info("\n".join(lines))

        if ready and self.exit_when_ready:
            self.get_logger().info("All required checks passed; exiting because exit_when_ready=true")
            rclpy.shutdown()

    def _topics_ok(self) -> bool:
        required = [
            self.vehicle_odom.fresh(self, self.topic_timeout),
            self.local_pos.fresh(self, self.topic_timeout),
            self.estimator.fresh(self, self.topic_timeout),
            self.status.fresh(self, self.topic_timeout),
        ]
        if self.require_mocap:
            required.append(self.mocap.fresh(self, self.topic_timeout))
        return all(required)

    def _estimator_ok(self) -> bool:
        lp = self.local_pos.msg
        est = self.estimator.msg
        if lp is None or est is None:
            return False
        required = [
            bool(lp.xy_valid),
            bool(lp.z_valid),
            bool(est.cs_ev_pos),
            bool(est.cs_ev_hgt),
        ]
        if self.require_yaw_good:
            required.extend([
                bool(est.cs_yaw_align),
                bool(lp.heading_good_for_control),
            ])
        return all(required)

    def _tf_ok(self) -> bool:
        odom_ok = self._can_transform(self.odom_frame, self.base_frame)
        map_ok = self._can_transform(self.map_frame, self.base_frame)
        return odom_ok and map_ok

    def _can_transform(self, target: str, source: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
            return True
        except TransformException:
            return False

    def _topic_line(self, name: str, state: TopicState) -> str:
        age = state.age(self)
        if age is None:
            return f"[topic] {name}: missing"
        flag = "ok" if age <= self.topic_timeout else "stale"
        return f"[topic] {name}: {flag}, count={state.count}, age={age:.2f}s"

    def _vehicle_odom_lines(self):
        msg = self.vehicle_odom.msg
        if msg is None:
            return ["[vehicle_odometry] no data"]
        enu_pos = px4_ned_position_to_ros_enu(msg)
        enu_vel = px4_ned_velocity_to_ros_enu(msg)
        raw_pos = tuple(float(v) for v in msg.position)
        return [
            f"[vehicle_odometry] pose_frame={msg.pose_frame}, velocity_frame={msg.velocity_frame}, quality={msg.quality}",
            f"[vehicle_odometry] raw PX4 NED pos=({raw_pos[0]:.3f}, {raw_pos[1]:.3f}, {raw_pos[2]:.3f})",
            f"[vehicle_odometry] expected ROS ENU /odom pos=({enu_pos[0]:.3f}, {enu_pos[1]:.3f}, {enu_pos[2]:.3f})",
            f"[vehicle_odometry] expected ROS ENU velocity=({enu_vel[0]:.3f}, {enu_vel[1]:.3f}, {enu_vel[2]:.3f})",
        ]

    def _local_position_lines(self):
        msg = self.local_pos.msg
        if msg is None:
            return ["[local_position] no data"]
        return [
            "[local_position] "
            f"xy_valid={msg.xy_valid}, z_valid={msg.z_valid}, "
            f"v_xy_valid={msg.v_xy_valid}, v_z_valid={msg.v_z_valid}, "
            f"heading_good={msg.heading_good_for_control}",
            "[local_position] "
            f"x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, "
            f"heading={msg.heading:.3f}, dead_reckoning={msg.dead_reckoning}",
        ]

    def _estimator_lines(self):
        msg = self.estimator.msg
        if msg is None:
            return ["[estimator] no data"]
        return [
            "[estimator] "
            f"tilt={msg.cs_tilt_align}, yaw={msg.cs_yaw_align}, "
            f"ev_pos={msg.cs_ev_pos}, ev_yaw={msg.cs_ev_yaw}, ev_hgt={msg.cs_ev_hgt}, "
            f"mag={msg.cs_mag}, gnss={msg.cs_gnss_pos}, fake_pos={msg.cs_valid_fake_pos}",
        ]

    def _status_lines(self):
        msg = self.status.msg
        if msg is None:
            return ["[vehicle_status] no data"]
        return [
            "[vehicle_status] "
            f"arming_state={msg.arming_state}, nav_state={msg.nav_state}, "
            f"preflight={msg.pre_flight_checks_pass}, failsafe={msg.failsafe}, "
            f"gcs_lost={msg.gcs_connection_lost}",
        ]

    def _tf_lines(self):
        return [
            f"[tf] {self.odom_frame} -> {self.base_frame}: {'ok' if self._can_transform(self.odom_frame, self.base_frame) else 'missing'}",
            f"[tf] {self.map_frame} -> {self.base_frame}: {'ok' if self._can_transform(self.map_frame, self.base_frame) else 'missing'}",
        ]


def main():
    rclpy.init()
    node = Mav1Nav2TopicCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received Ctrl+C")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
