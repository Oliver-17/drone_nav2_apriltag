#!/usr/bin/env python3
"""Take off, accept manual map start/goal coordinates, then drive PX4 from Nav2."""

import math
import sys
from pathlib import Path

import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
from frame_conversions import VectorENU, enu_to_ned, yaw_enu_to_ned

import rclpy
from geometry_msgs.msg import PoseArray, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

S_WAIT_FCU = "WAIT_FCU"
S_WARMUP = "WARMUP"
S_ARMING = "ARMING"
S_TAKEOFF = "TAKEOFF"
S_WAIT_INPUT = "WAIT_INPUT"
S_NAVIGATING = "NAVIGATING"
S_LANDING = "LANDING"
S_DONE = "DONE"
S_ABORT = "ABORT"

LOOP_HZ = 10.0


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


class TakeoffManualNav2(Node):
    def __init__(self):
        super().__init__("takeoff_manual_nav2")

        p = self.declare_parameter
        self.px4_ns = p("px4_namespace", "/MAV1").value.rstrip("/")
        self.target_system = int(p("target_system", 1).value)
        self.takeoff_altitude = float(p("takeoff_altitude", 0.5).value)
        self.altitude_tolerance = float(p("altitude_tolerance", 0.1).value)
        self.arm_timeout = float(p("arm_timeout", 30.0).value)
        self.takeoff_timeout = float(p("takeoff_timeout", 30.0).value)
        self.land_after_goal = bool(p("land_after_goal", True).value)
        self.land_on_abort = bool(p("land_on_abort", True).value)
        self.land_command_duration = float(p("land_command_duration", 3.0).value)
        self.max_xy_speed = float(p("max_xy_speed", 0.7).value)
        self.cmd_vel_topic = p("cmd_vel_topic", "/cmd_vel").value
        self.odom_topic = p("odom_topic", "/odom").value
        self.map_frame = p("map_frame", "map").value
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.goal_action = p("goal_action", "/navigate_to_pose").value
        self.manual_goal_topic = p("manual_goal_topic", "/manual_nav2_start_goal").value
        self.require_valid_local_position = bool(p("require_valid_local_position", True).value)

        self.pos = None
        self.odom = None
        self.status = None
        self.cmd = Twist()
        self.origin_x = None
        self.origin_y = None
        self.yaw_enu = 0.0
        self.state = S_WAIT_FCU
        self.ticks = 0
        self.finished = False
        self.exit_code = 0
        self.manual_start = None
        self.manual_goal = None
        self.map_to_odom = None
        self.goal_handle = None
        self.was_offboard = False

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        ns = self.px4_ns
        self.pub_ocm = self.create_publisher(OffboardControlMode, ns + "/fmu/in/offboard_control_mode", pub_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, ns + "/fmu/in/trajectory_setpoint", pub_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, ns + "/fmu/in/vehicle_command", pub_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.nav_client = ActionClient(self, NavigateToPose, self.goal_action)

        self.create_subscription(VehicleLocalPosition, ns + "/fmu/out/vehicle_local_position_v1", self._on_local_position, sub_qos)
        self.create_subscription(VehicleStatus, ns + "/fmu/out/vehicle_status_v1", self._on_status, sub_qos)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        goal_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseArray, self.manual_goal_topic, self._on_manual_goal, goal_qos)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"PX4 namespace / target_system : {self.px4_ns} / {self.target_system}")
        self.get_logger().info(f"takeoff altitude              : {self.takeoff_altitude:.2f} m")
        self.get_logger().info(f"manual input topic            : {self.manual_goal_topic}")
        self.get_logger().info(f"land after goal               : {self.land_after_goal}")
        self.get_logger().info(f"require valid local position  : {self.require_valid_local_position}")
        self.get_logger().info("=" * 60)

        self.timer = self.create_timer(1.0 / LOOP_HZ, self._loop)

    def _on_local_position(self, msg):
        self.pos = msg

    def _on_status(self, msg):
        self.status = msg

    def _on_cmd_vel(self, msg):
        self.cmd = msg

    def _on_odom(self, msg):
        self.odom = msg

    def _on_manual_goal(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().error("manual goal message needs two poses: start, goal")
            return
        self.manual_start = self._pose_to_tuple(msg.poses[0])
        self.manual_goal = self._pose_to_tuple(msg.poses[1])
        self.get_logger().info(
            f"received manual ENU start=({self.manual_start[0]:.2f}, {self.manual_start[1]:.2f}, "
            f"{math.degrees(self.manual_start[2]):.1f} deg), "
            f"goal=({self.manual_goal[0]:.2f}, {self.manual_goal[1]:.2f}, "
            f"{math.degrees(self.manual_goal[2]):.1f} deg)")

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

    def publish_offboard_mode(self, velocity=False):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = bool(velocity)
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.pub_ocm.publish(msg)

    def publish_hold_setpoint(self):
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

    def publish_nav2_velocity_setpoint(self):
        vx = max(-self.max_xy_speed, min(self.max_xy_speed, float(self.cmd.linear.x)))
        vy = max(-self.max_xy_speed, min(self.max_xy_speed, float(self.cmd.linear.y)))
        wz = float(self.cmd.angular.z)
        self.yaw_enu = wrap_angle(self.yaw_enu + wz / LOOP_HZ)

        msg = TrajectorySetpoint()
        alt_ned = enu_to_ned(VectorENU(east=math.nan, north=math.nan, up=self.takeoff_altitude))
        msg.position = alt_ned.as_list()
        vel_ned = enu_to_ned(VectorENU(east=vx, north=vy, up=0.0))
        msg.velocity = vel_ned.as_list()
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = yaw_enu_to_ned(self.yaw_enu)
        msg.yawspeed = -wz
        msg.timestamp = self.now_us()
        self.pub_sp.publish(msg)

    def publish_zero_velocity_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = 0.0
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

    @staticmethod
    def _pose_to_tuple(pose):
        yaw = quat_to_yaw(pose.orientation)
        return (float(pose.position.x), float(pose.position.y), yaw)

    def _compute_map_to_odom(self):
        if self.odom is None or self.manual_start is None:
            return None
        sx, sy, syaw = self.manual_start
        ox = self.odom.pose.pose.position.x
        oy = self.odom.pose.pose.position.y
        oyaw = quat_to_yaw(self.odom.pose.pose.orientation)
        yaw = wrap_angle(syaw - oyaw)
        c = math.cos(yaw)
        s = math.sin(yaw)
        tx = sx - (c * ox - s * oy)
        ty = sy - (s * ox + c * oy)
        return (tx, ty, yaw)

    def _publish_map_to_odom(self):
        if self.map_to_odom is None:
            return
        tx, ty, yaw = self.map_to_odom
        qx, qy, qz, qw = yaw_to_quat(yaw)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _send_nav2_goal(self):
        sx, sy, syaw = self.manual_start
        gx_in, gy_in, gyaw_in = self.manual_goal
        if self.odom is None:
            self.get_logger().info("waiting for /odom before sending goal")
            return
        ox = self.odom.pose.pose.position.x
        oy = self.odom.pose.pose.position.y
        oyaw = quat_to_yaw(self.odom.pose.pose.orientation)
        dx = gx_in - sx
        dy = gy_in - sy
        yaw_offset = wrap_angle(oyaw - syaw)
        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        gx = ox + c * dx - s * dy
        gy = oy + s * dx + c * dy
        gyaw = wrap_angle(oyaw + (gyaw_in - syaw))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(gyaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Nav2 action server not ready: {self.goal_action}")
            self._goto(S_ABORT, "Nav2 action unavailable")
            return
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f"sent Nav2 goal: input_start=({sx:.2f}, {sy:.2f}), "
            f"input_goal=({gx_in:.2f}, {gy_in:.2f}), mapped_goal=({gx:.2f}, {gy:.2f}), "
            f"yaw={math.degrees(gyaw):.1f} deg")

    def _on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            self._goto(S_ABORT, "goal rejected")
            return
        self.get_logger().info("Nav2 goal accepted")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        self.get_logger().info(f"Nav2 finished with status={result.status}")
        if self.land_after_goal:
            self._goto(S_LANDING, "Nav2 finished; landing")
        else:
            self._goto(S_DONE, "Nav2 finished")

    def _loop(self):
        self.ticks += 1
        if self.state == S_WAIT_FCU:
            if self.pos is not None and self.pos.xy_valid and self.pos.z_valid:
                self.origin_x = self.pos.x
                self.origin_y = self.pos.y
                self._goto(S_WARMUP, "PX4 local position ready")
            elif self.ticks % int(LOOP_HZ * 2) == 1:
                if self.pos is None:
                    self.get_logger().info(
                        f"等 {self.px4_ns}/fmu/out/vehicle_local_position_v1 ... 檢查 MicroXRCEAgent 與 namespace")
                else:
                    self.get_logger().info(
                        f"收到 local position 但尚未 valid: xy_valid={self.pos.xy_valid}, z_valid={self.pos.z_valid}, "
                        f"x={self.pos.x:.3f}, y={self.pos.y:.3f}, z={self.pos.z:.3f}")
            return

        if self.state in (S_WARMUP, S_ARMING, S_TAKEOFF, S_WAIT_INPUT):
            self.publish_offboard_mode(velocity=False)
            self.publish_hold_setpoint()
        elif self.state == S_NAVIGATING:
            self.publish_offboard_mode(velocity=True)
            self.publish_nav2_velocity_setpoint()
        elif self.state == S_LANDING:
            self.publish_offboard_mode(velocity=True)
            self.publish_zero_velocity_setpoint()

        if self.was_offboard and not self.is_offboard() and self.state in (S_TAKEOFF, S_WAIT_INPUT, S_NAVIGATING):
            self.get_logger().error(
                f"PX4 left offboard, nav_state={self.status.nav_state if self.status else 'unknown'}")
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
                self.get_logger().error(
                    f"arm/offboard timeout, armed={self.is_armed()}, offboard={self.is_offboard()}, "
                    f"arming_state={self.status.arming_state if self.status else 'unknown'}, "
                    f"nav_state={self.status.nav_state if self.status else 'unknown'}")
                self._goto(S_ABORT, "arm timeout")

        elif self.state == S_TAKEOFF:
            err = abs((-self.takeoff_altitude) - self.pos.z)
            if self.ticks % int(LOOP_HZ * 2) == 0:
                self.get_logger().info(f"takeoff: altitude {-self.pos.z:.2f} / {self.takeoff_altitude:.2f} m")
            if err <= self.altitude_tolerance:
                self._goto(S_WAIT_INPUT, "takeoff altitude reached")
            elif self.ticks > LOOP_HZ * self.takeoff_timeout:
                self.get_logger().error(f"takeoff timeout, remaining error {err:.2f} m")
                self._goto(S_ABORT, "takeoff timeout")

        elif self.state == S_WAIT_INPUT:
            if self.ticks % int(LOOP_HZ * 3) == 1:
                self.get_logger().info(
                    f"hovering; publish start/goal from another terminal on {self.manual_goal_topic}")
            if self.manual_start is not None and self.manual_goal is not None:
                self._send_nav2_goal()
                self._goto(S_NAVIGATING, "manual goal sent")

        elif self.state == S_LANDING:
            if self.ticks == 1:
                self.get_logger().info("Sending PX4 LAND command")
            if self.ticks <= max(1, int(LOOP_HZ * self.land_command_duration)):
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            else:
                self._goto(S_DONE, "land command sent")

        elif self.state == S_DONE:
            self.finished = True
            self.exit_code = 0
            self.timer.cancel()

        elif self.state == S_ABORT:
            if self.land_on_abort and self.is_armed():
                self.get_logger().error("abort while armed; sending LAND")
                for _ in range(10):
                    self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.finished = True
            self.exit_code = 1
            self.timer.cancel()


def main():
    rclpy.init()
    node = TakeoffManualNav2()
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
