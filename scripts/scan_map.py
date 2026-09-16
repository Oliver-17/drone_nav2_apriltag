#!/usr/bin/env python3
"""scan_map.py -- PX4 offboard lawnmower scan controller for RTAB-Map."""

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
S_SCAN = "SCAN"
S_LANDING = "LANDING"
S_HOLD = "HOLD"
S_DONE = "DONE"
S_ABORT = "ABORT"

LOOP_HZ = 10.0


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class ScanMap(Node):
    """Take off and fly a relative lawnmower path for map collection."""

    def __init__(self):
        super().__init__("scan_map")

        p = self.declare_parameter
        self.px4_ns = p("px4_namespace", "/MAV1").value
        self.target_system = int(p("target_system", 1).value)
        self.flight_altitude = float(p("flight_altitude", 3.0).value)
        self.scan_length = float(p("scan_length", 22.0).value)
        self.scan_width = float(p("scan_width", 10.0).value)
        self.lane_spacing = float(p("lane_spacing", 2.5).value)
        self.arrival_radius = float(p("arrival_radius", 0.8).value)
        self.hold_time = float(p("hold_time", 1.0).value)
        self.leg_timeout = float(p("leg_timeout", 60.0).value)
        self.face_travel = as_bool(p("face_travel_direction", True).value)
        self.land_at_end = as_bool(p("land_at_end", False).value)
        self.start_east = float(p("start_east", 0.0).value)
        self.start_north = float(p("start_north", 0.0).value)

        if self.flight_altitude <= 0.5:
            self.get_logger().warn("flight_altitude 太低，已強制改成 0.5 m")
            self.flight_altitude = 0.5
        if self.lane_spacing <= 0.2:
            self.get_logger().warn("lane_spacing 太小，已強制改成 0.2 m")
            self.lane_spacing = 0.2

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        ns = self.px4_ns.rstrip("/")
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

        self.pos = None
        self.status = None
        self.origin_ned = None
        self.path = []
        self.leg = 0
        self.ticks = 0
        self.hold_ticks = 0
        self.state = S_WAIT_FCU
        self.was_offboard = False
        self.finished = False
        self.exit_code = 0

        self._print_banner()
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._loop)

    def _print_banner(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("  RTAB-Map 掃圖控制：起飛 + lawnmower path")
        self.get_logger().info(f"  PX4 namespace / target_system : {self.px4_ns} / {self.target_system}")
        self.get_logger().info(f"  高度/長度/寬度/間距 : {self.flight_altitude} / {self.scan_length} / {self.scan_width} / {self.lane_spacing}")
        self.get_logger().info(f"  結束後降落 : {self.land_at_end}")
        self.get_logger().info("=" * 60)

    def _on_local_position(self, msg):
        self.pos = msg

    def _on_status(self, msg):
        self.status = msg

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def is_armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.pub_cmd.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.pub_ocm.publish(msg)

    def publish_setpoint(self, ned_x, ned_y, ned_z, yaw=float("nan")):
        msg = TrajectorySetpoint()
        msg.position = [float(ned_x), float(ned_y), float(ned_z)]
        msg.velocity = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.yaw = float(yaw)
        msg.yawspeed = float("nan")
        msg.timestamp = self.now_us()
        self.pub_sp.publish(msg)

    def _goto(self, state, why=""):
        if state == self.state:
            return
        self.get_logger().info(f"[狀態] {self.state} -> {state}" + (f" ({why})" if why else ""))
        self.state = state
        self.ticks = 0

    def _build_path(self):
        """Build ENU-offset scan path, then convert it to local NED waypoints."""
        ox, oy, _ = self.origin_ned
        lanes = max(2, int(math.floor(self.scan_width / self.lane_spacing)) + 1)
        north0 = self.start_north - self.scan_width / 2.0
        enu_points = [(self.start_east, self.start_north)]

        for i in range(lanes):
            north = north0 + i * self.lane_spacing
            if i == lanes - 1:
                north = self.start_north + self.scan_width / 2.0
            east_a = self.start_east
            east_b = self.start_east + self.scan_length
            if i % 2 == 0:
                enu_points.append((east_a, north))
                enu_points.append((east_b, north))
            else:
                enu_points.append((east_b, north))
                enu_points.append((east_a, north))

        self.path = [
            (ox + north, oy + east, -self.flight_altitude)
            for east, north in enu_points
        ]
        self.get_logger().info(f"掃描 waypoint 數量：{len(self.path)}")

    def _target(self):
        if self.state in (S_WARMUP,):
            return self.pos.x, self.pos.y, self.pos.z, float("nan")
        if self.state in (S_ARMING, S_TAKEOFF):
            return self.origin_ned[0], self.origin_ned[1], -self.flight_altitude, float("nan")
        if self.state in (S_SCAN, S_HOLD):
            x, y, z = self.path[min(self.leg, len(self.path) - 1)]
            return x, y, z, self._yaw_to(x, y)
        return self.pos.x, self.pos.y, self.pos.z, float("nan")

    def _yaw_to(self, ned_x, ned_y):
        if not self.face_travel or self.pos is None:
            return float("nan")
        dx = ned_x - self.pos.x
        dy = ned_y - self.pos.y
        if math.hypot(dx, dy) < max(self.arrival_radius, 0.5):
            return float("nan")
        return math.atan2(dy, dx)

    def _publish_current_setpoint(self):
        self.publish_offboard_mode()
        self.publish_setpoint(*self._target())

    def _loop(self):
        self.ticks += 1

        if self.state == S_WAIT_FCU:
            if self.pos is not None and self.pos.xy_valid and self.pos.z_valid:
                self.origin_ned = (self.pos.x, self.pos.y, self.pos.z)
                self._build_path()
                self._goto(S_WARMUP, "PX4 位置估計已收斂")
            else:
                self.get_logger().info(
                    "等 PX4 VehicleLocalPosition... 檢查 MicroXRCEAgent、namespace 與 QoS",
                    throttle_duration_sec=5.0)
            return

        if self.state in (S_WARMUP, S_ARMING, S_TAKEOFF, S_SCAN, S_HOLD):
            self._publish_current_setpoint()

        if self.was_offboard and not self.is_offboard() and self.state in (S_TAKEOFF, S_SCAN, S_HOLD):
            nav_state = self.status.nav_state if self.status is not None else "unknown"
            self.get_logger().error(f"被踢出 offboard，nav_state={nav_state}")
            self._goto(S_ABORT, "失去 offboard")
            return
        if self.is_offboard():
            self.was_offboard = True

        if self.state == S_WARMUP:
            if self.ticks >= LOOP_HZ:
                self._goto(S_ARMING, "setpoint 串流已建立")
        elif self.state == S_ARMING:
            self._do_arming()
        elif self.state == S_TAKEOFF:
            self._do_takeoff()
        elif self.state == S_SCAN:
            self._do_scan()
        elif self.state == S_HOLD:
            self._do_hold()
        elif self.state == S_LANDING:
            self._do_landing()
        elif self.state == S_DONE:
            self._finish(0, "任務結束")
        elif self.state == S_ABORT:
            self._finish(1, "任務中止")

    def _do_arming(self):
        if self.ticks % 10 == 1:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if self.is_offboard() and self.is_armed():
            self._goto(S_TAKEOFF, "已進入 offboard 並解鎖")
        elif self.ticks > LOOP_HZ * 30:
            self.get_logger().error("30 秒內沒能進入 offboard/armed")
            self._goto(S_ABORT, "解鎖逾時")

    def _do_takeoff(self):
        err = abs(-self.flight_altitude - self.pos.z)
        if err < 0.5:
            self.leg = 0
            self.hold_ticks = 0
            self._goto(S_SCAN, "爬升完成，開始掃圖")
        elif self.ticks > LOOP_HZ * self.leg_timeout:
            self.get_logger().error(f"爬升逾時，還差 {err:.2f} m")
            self._goto(S_LANDING, "爬升逾時")

    def _do_scan(self):
        tx, ty, tz = self.path[self.leg]
        d_xy = math.hypot(tx - self.pos.x, ty - self.pos.y)
        d_z = abs(tz - self.pos.z)

        if self.ticks % 20 == 0:
            self.get_logger().info(
                f"掃描點 {self.leg + 1}/{len(self.path)}，水平剩 {d_xy:.2f} m，高度差 {d_z:.2f} m")

        if d_xy < self.arrival_radius and d_z < 0.7:
            self.hold_ticks += 1
            if self.hold_ticks == 1:
                self.get_logger().info(f"到達掃描點 {self.leg + 1}/{len(self.path)}")
            if self.hold_ticks >= LOOP_HZ * self.hold_time:
                self.hold_ticks = 0
                self.leg += 1
                self.ticks = 0
                if self.leg >= len(self.path):
                    self.get_logger().info("掃描路徑完成")
                    self._goto(S_LANDING if self.land_at_end else S_HOLD,
                               "路徑完成")
        else:
            self.hold_ticks = 0

        if self.ticks > LOOP_HZ * self.leg_timeout:
            self.get_logger().error(f"掃描點 {self.leg + 1} 逾時，水平剩 {d_xy:.2f} m")
            self._goto(S_LANDING, "單點逾時")

    def _do_hold(self):
        if self.ticks == 1:
            self.get_logger().info("保持最後掃描點，RTAB-Map 可繼續存圖；Ctrl+C 可停止")

    def _do_landing(self):
        if self.ticks == 1 or self.ticks % 20 == 1:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if not self.is_armed() and self.ticks > LOOP_HZ * 2:
            self._goto(S_DONE, "已著陸並上鎖")
        elif self.ticks > LOOP_HZ * 60:
            self.get_logger().warn("降落逾時，不再等待")
            self._goto(S_DONE, "降落逾時")

    def _finish(self, code, message):
        self.get_logger().info(message)
        self.timer.cancel()
        self.exit_code = code
        self.finished = True


def main():
    rclpy.init()
    node = ScanMap()
    code = 0
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.exit_code
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C")
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
