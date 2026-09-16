#!/usr/bin/env python3
"""PX4 offboard takeoff, then release control to PX4 loiter."""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

S_WAIT_FCU = "WAIT_FCU"
S_WARMUP = "WARMUP"
S_ARMING = "ARMING"
S_TAKEOFF = "TAKEOFF"
S_RELEASE = "RELEASE"
S_DONE = "DONE"
S_ABORT = "ABORT"

LOOP_HZ = 10.0
PX4_MAIN_MODE_AUTO = 4.0
PX4_AUTO_SUBMODE_LOITER = 3.0


class TakeoffHold(Node):
    def __init__(self):
        super().__init__("takeoff_hold")

        p = self.declare_parameter
        self.px4_ns = p("px4_namespace", "/MAV1").value.rstrip("/")
        self.target_system = int(p("target_system", 1).value)
        self.takeoff_altitude = float(p("takeoff_altitude", 3.0).value)
        self.altitude_tolerance = float(p("altitude_tolerance", 0.3).value)
        self.arm_timeout = float(p("arm_timeout", 30.0).value)
        self.takeoff_timeout = float(p("takeoff_timeout", 30.0).value)
        self.release_after_takeoff = bool(p("release_after_takeoff", True).value)
        self.land_on_shutdown = bool(p("land_on_shutdown", False).value)

        self.pos = None
        self.status = None
        self.origin_x = None
        self.origin_y = None
        self.state = S_WAIT_FCU
        self.ticks = 0
        self.finished = False
        self.exit_code = 0
        self.was_offboard = False

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        ns = self.px4_ns
        self.pub_ocm = self.create_publisher(
            OffboardControlMode, ns + "/fmu/in/offboard_control_mode", pub_qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, ns + "/fmu/in/trajectory_setpoint", pub_qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, ns + "/fmu/in/vehicle_command", pub_qos)
        self.create_subscription(
            VehicleLocalPosition, ns + "/fmu/out/vehicle_local_position_v1",
            self._on_local_position, sub_qos)
        self.create_subscription(
            VehicleStatus, ns + "/fmu/out/vehicle_status_v1",
            self._on_status, sub_qos)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  PX4 namespace / target_system : {self.px4_ns} / {self.target_system}")
        self.get_logger().info(f"  takeoff altitude              : {self.takeoff_altitude:.2f} m")
        self.get_logger().info(f"  release_after_takeoff         : {self.release_after_takeoff}")
        self.get_logger().info("=" * 60)

        self.timer = self.create_timer(1.0 / LOOP_HZ, self._loop)

    def _on_local_position(self, msg):
        self.pos = msg

    def _on_status(self, msg):
        self.status = msg

    def _goto(self, state, why=""):
        if state == self.state:
            return
        self.get_logger().info(f"[狀態] {self.state} -> {state}" + (f" ({why})" if why else ""))
        self.state = state
        self.ticks = 0

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def is_armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.pub_ocm.publish(msg)

    def publish_setpoint(self):
        if self.pos is None:
            return
        msg = TrajectorySetpoint()
        msg.position = [float(self.origin_x), float(self.origin_y), -self.takeoff_altitude]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        msg.timestamp = self.now_us()
        self.pub_sp.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.pub_cmd.publish(msg)

    def set_auto_loiter(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            PX4_MAIN_MODE_AUTO,
            PX4_AUTO_SUBMODE_LOITER,
        )

    def _loop(self):
        self.ticks += 1

        if self.state == S_WAIT_FCU:
            if self.pos is not None and self.pos.xy_valid and self.pos.z_valid:
                self.origin_x = self.pos.x
                self.origin_y = self.pos.y
                self._goto(S_WARMUP, "PX4 local position ready")
            elif self.ticks % int(LOOP_HZ * 2) == 1:
                self.get_logger().info(
                    f"等 {self.px4_ns}/fmu/out/vehicle_local_position_v1 ... 檢查 MicroXRCEAgent 與 namespace")
            return

        if self.state in (S_WARMUP, S_ARMING, S_TAKEOFF):
            self.publish_offboard_mode()
            self.publish_setpoint()

        if self.was_offboard and not self.is_offboard() and self.state == S_TAKEOFF:
            self.get_logger().error(
                f"PX4 left offboard before takeoff was complete, nav_state={self.status.nav_state if self.status else 'unknown'}")
            self._goto(S_ABORT, "lost offboard")
            return
        if self.is_offboard():
            self.was_offboard = True

        if self.state == S_WARMUP:
            if self.ticks >= LOOP_HZ:
                self._goto(S_ARMING, "setpoint stream ready")

        elif self.state == S_ARMING:
            if self.ticks % int(LOOP_HZ) == 1:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.is_offboard() and self.is_armed():
                self._goto(S_TAKEOFF, "armed and offboard")
            elif self.ticks > LOOP_HZ * self.arm_timeout:
                self.get_logger().error("arm/offboard timeout")
                self._goto(S_ABORT, "arm timeout")

        elif self.state == S_TAKEOFF:
            err = abs((-self.takeoff_altitude) - self.pos.z)
            if self.ticks % int(LOOP_HZ * 2) == 0:
                altitude = -self.pos.z
                self.get_logger().info(
                    f"takeoff: altitude {altitude:.2f} / {self.takeoff_altitude:.2f} m")
            if err <= self.altitude_tolerance:
                if self.release_after_takeoff:
                    self._goto(S_RELEASE, "takeoff altitude reached")
                else:
                    self.get_logger().info("takeoff altitude reached; keeping offboard hold")
            elif self.ticks > LOOP_HZ * self.takeoff_timeout:
                self.get_logger().error(f"takeoff timeout, remaining error {err:.2f} m")
                self._goto(S_ABORT, "takeoff timeout")

        elif self.state == S_RELEASE:
            if self.ticks == 1:
                self.get_logger().info("Switching PX4 to AUTO.LOITER and releasing offboard setpoints")
            if self.ticks <= int(LOOP_HZ):
                self.set_auto_loiter()
            else:
                self._goto(S_DONE, "PX4 should now hold position by itself")

        elif self.state == S_DONE:
            self.get_logger().info("takeoff stage complete; start SLAM, then start fly_nodes")
            self.finished = True
            self.exit_code = 0
            self.timer.cancel()

        elif self.state == S_ABORT:
            self.finished = True
            self.exit_code = 1
            self.timer.cancel()

    def destroy_node(self):
        if self.land_on_shutdown and self.is_armed():
            self.get_logger().info("land_on_shutdown=true, sending NAV_LAND")
            for _ in range(10):
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        super().destroy_node()


def main():
    rclpy.init()
    node = TakeoffHold()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Received Ctrl+C")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == "__main__":
    sys.exit(main())
