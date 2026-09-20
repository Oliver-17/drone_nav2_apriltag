#!/usr/bin/env python3

import math
from pathlib import Path
import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
from frame_conversions import VectorENU, enu_to_ned, yaw_enu_to_ned

import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class CmdVelToPx4(Node):
    """Convert Nav2 planar velocity commands into fixed-altitude PX4 setpoints."""

    def __init__(self):
        super().__init__("cmd_vel_to_px4")

        self.px4_ns = self.declare_parameter("px4_namespace", "/MAV1").value.rstrip("/")
        self.target_system = int(self.declare_parameter("target_system", 1).value)
        self.fixed_altitude = float(self.declare_parameter("fixed_altitude", 2.0).value)
        self.max_xy_speed = float(self.declare_parameter("max_xy_speed", 0.7).value)
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f"{self.px4_ns}/fmu/in/offboard_control_mode",
            qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f"{self.px4_ns}/fmu/in/trajectory_setpoint",
            qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            f"{self.px4_ns}/fmu/in/vehicle_command",
            qos,
        )

        self.cmd = Twist()
        self.yaw_enu = 0.0
        self.armed = False
        self.offboard_requested = False
        self.setpoint_count = 0

        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        self.create_timer(0.05, self._timer_cb)

        self.get_logger().info(
            f"Bridging {cmd_vel_topic} to {self.px4_ns} PX4 setpoints at z={self.fixed_altitude:.2f} m"
        )

    def _cmd_vel_cb(self, msg):
        self.cmd = msg

    def _timer_cb(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        offboard = OffboardControlMode()
        offboard.timestamp = now_us
        offboard.position = True
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        vx = self._clamp(self.cmd.linear.x, -self.max_xy_speed, self.max_xy_speed)
        vy = self._clamp(self.cmd.linear.y, -self.max_xy_speed, self.max_xy_speed)
        wz = self.cmd.angular.z
        self.yaw_enu = self._wrap_angle(self.yaw_enu + wz * 0.05)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now_us
        alt_ned = enu_to_ned(VectorENU(east=math.nan, north=math.nan, up=self.fixed_altitude))
        setpoint.position = alt_ned.as_list()
        vel_ned = enu_to_ned(VectorENU(east=vx, north=vy, up=0.0))
        setpoint.velocity = vel_ned.as_list()
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.yaw = yaw_enu_to_ned(self.yaw_enu)
        setpoint.yawspeed = -wz
        self.setpoint_pub.publish(setpoint)

        self.setpoint_count += 1
        if self.setpoint_count == 20:
            self._request_offboard_and_arm(now_us)

    def _request_offboard_and_arm(self, now_us):
        self._publish_vehicle_command(
            now_us,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self._publish_vehicle_command(
            now_us,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("Requested PX4 offboard mode and arm")

    def _publish_vehicle_command(self, now_us, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _wrap_angle(value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


def main():
    rclpy.init()
    node = CmdVelToPx4()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
