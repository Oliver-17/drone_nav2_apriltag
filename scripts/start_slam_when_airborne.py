#!/usr/bin/env python3
"""Start a SLAM launch file after PX4 reports the vehicle is airborne."""

import os
import signal
import subprocess
import sys

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition


class StartSlamWhenAirborne(Node):
    def __init__(self):
        super().__init__("start_slam_when_airborne")

        self.px4_ns = self._param("px4_namespace", "/MAV1").rstrip("/")
        self.start_altitude = float(self._param("start_altitude", 2.5))
        self.package = self._param("slam_package", "drone_nav2_apriltag")
        self.launch_file = self._param("slam_launch_file", "cartographer_3d_depth.launch.py")
        self.use_sim_time = self._param("use_sim_time", True)
        self.slam_launch_args = {
            "raw_points_topic": self._param(
                "raw_points_topic",
                f"{self.px4_ns}/camera_front/depth/points",
            ),
            "imu_topic": self._param("imu_topic", "/imu"),
            "gz_imu_topic": self._param(
                "gz_imu_topic",
                "/world/nav2_arena/model/x500_depth_nav2_0/link/base_link/sensor/imu_sensor/imu",
            ),
            "px4_imu_topic": self._param("px4_imu_topic", f"{self.px4_ns}/fmu/out/sensor_combined"),
            "px4_odom_topic": self._param("px4_odom_topic", f"{self.px4_ns}/fmu/out/vehicle_odometry"),
            "odom_topic": self._param("odom_topic", "/odom"),
            "imu_frame": self._param("imu_frame", "base_link"),
        }

        self.initial_z = None
        self.current_altitude = None
        self.started = False
        self.process = None

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.px4_ns + "/fmu/out/vehicle_local_position_v1",
            self._on_local_position,
            sub_qos,
        )
        self.timer = self.create_timer(1.0, self._watch_child)
        self.status_timer = self.create_timer(1.0, self._log_wait_status)

        self.get_logger().info(
            f"Waiting until altitude >= {self.start_altitude:.2f} m, then start "
            f"{self.package} {self.launch_file}"
        )

    def _param(self, name, default):
        try:
            return self.declare_parameter(name, default).value
        except ParameterAlreadyDeclaredException:
            return self.get_parameter(name).value

    def _on_local_position(self, msg):
        if not msg.z_valid:
            return
        if self.initial_z is None:
            self.initial_z = msg.z
            self.get_logger().info(f"Recorded initial NED z={self.initial_z:.2f}")
            return
        if self.started:
            return

        altitude = self.initial_z - msg.z
        self.current_altitude = altitude
        if altitude >= self.start_altitude:
            self._start_slam(altitude)

    def _log_wait_status(self):
        if self.started:
            return
        if self.initial_z is None:
            self.get_logger().info("Waiting for PX4 local position before arming SLAM gate")
            return
        altitude = self.current_altitude if self.current_altitude is not None else 0.0
        self.get_logger().info(
            f"SLAM gate waiting: altitude {altitude:.2f} / {self.start_altitude:.2f} m"
        )

    def _start_slam(self, altitude):
        self.started = True
        cmd = [
            "ros2",
            "launch",
            self.package,
            self.launch_file,
            f"use_sim_time:={str(self.use_sim_time).lower()}",
        ]
        for name, value in self.slam_launch_args.items():
            if value not in (None, ""):
                cmd.append(f"{name}:={value}")

        self.get_logger().info(
            f"Altitude {altitude:.2f} m reached, starting SLAM: {' '.join(cmd)}"
        )
        self.process = subprocess.Popen(cmd, preexec_fn=os.setsid)

    def _watch_child(self):
        if self.process is None:
            return
        code = self.process.poll()
        if code is not None:
            self.get_logger().warn(f"SLAM launch exited, return code={code}")
            self.process = None

    def destroy_node(self):
        if self.process is not None and self.process.poll() is None:
            self.get_logger().info("Stopping SLAM launch")
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        super().destroy_node()


def main():
    rclpy.init()
    node = StartSlamWhenAirborne()
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
