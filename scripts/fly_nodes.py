#!/usr/bin/env python3
# =============================================================================
#  fly_nodes.py — 照拓樸圖的節點順序飛（T3b）
#
#  這是「地圖驗證工具」，不是正式的飛控程式。它的設計目標是：
#  在階段 1（TF）、2（cmd_vel 橋接）、4（Nav2）都還沒好的情況下，
#  仍然能證明「這張拓樸圖飛得完」。
#
#  資料流：
#      graphs/nav2_arena.geojson  ──> 節點座標（誰在哪裡）
#      route_server /compute_route ──> 節點順序（該怎麼走）
#                                       ↓
#                            TrajectorySetpoint.position
#                                       ↓
#                                      PX4
#
#  為什麼順序要問 route_server 而不是自己寫死：
#      寫死的話，就算把 geojson 裡的 cost / speed_limit / penalty 全部刪掉，
#      飛出來也一模一樣 —— 那等於只把拓樸圖當成一張座標清單在用，
#      證明不了「拓樸圖有在做事」。問 route_server 之後，改 geojson 的 cost
#      就能讓無人機改走另一條路，一行程式碼都不用動。
#
#  為什麼不用 /compute_and_track_route（會觸發地圖裡的 operations）：
#      那個要靠 TF 判斷「到節點了沒」（route_tracker.hpp:88 的註解寫著
#      base_frame pose in route_frame），而 TF 是階段 1 的東西，還沒有。
#      所以這裡只用 /compute_route（純規劃），到達判定自己用
#      VehicleLocalPosition 算。地圖裡的 scan_apriltag 不會被觸發。
#
#  它不做什麼：
#      不避障（兩點之間直線飛，安全靠節點放對位置）、不編隊（一台）、
#      不觸發 route operations。這些分別是階段 4、6 和階段 1 的事。
#
#  用法：
#      ros2 launch drone_nav2_apriltag fly_nodes.launch.py
#  或單獨跑（需自己先開 route_server）：
#      ros2 run drone_nav2_apriltag fly_nodes.py --ros-args -p flight_altitude:=3.0
# =============================================================================

import json
import math
import os
import subprocess
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav2_msgs.action import ComputeRoute
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)


# 狀態機。用字串而不是列舉，是為了印 log 的時候直接看得懂。
S_WAIT_ROUTE = "WAIT_ROUTE"    # 等 route_server 回答走哪些節點
S_WAIT_FCU = "WAIT_FCU"        # 等 PX4 的位置估計收斂
S_WARMUP = "WARMUP"            # 先把 setpoint 串流跑起來
S_ARMING = "ARMING"            # 切 offboard + 解鎖
S_TAKEOFF = "TAKEOFF"          # 爬升到巡航高度
S_FLY = "FLY"                  # 依序飛節點
S_YAW_SCAN = "YAW_SCAN"        # 到指定節點後原地旋轉掃描
S_TURN_SETTLE = "TURN_SETTLE"  # 轉彎節點先停住對準下一段再走
S_LANDING = "LANDING"          # 交給 PX4 自動降落
S_DONE = "DONE"
S_ABORT = "ABORT"

LOOP_HZ = 10.0                 # PX4 要求 offboard setpoint 至少 2 Hz，10 Hz 留足餘裕


class FlyNodes(Node):

    def __init__(self):
        super().__init__("fly_nodes")

        # ---------- 參數 ----------
        p = self.declare_parameter
        default_graph = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "graphs", "nav2_arena.geojson")
        self.graph_file = p("graph_file", default_graph).value
        self.px4_ns = p("px4_namespace", "").value          # 多機時是 /MAV1 這種
        self.target_system = p("target_system", 1).value    # 多機時是 instance+1
        self.start_node = p("start_node", -1).value         # -1 = 用最小的 id
        self.goal_node = p("goal_node", -1).value           # -1 = 用最大的 id
        self.use_route_server = p("use_route_server", True).value
        self.node_sequence = list(p("node_sequence", [0]).value)  # 上面設 False 時才用
        self.flight_altitude = p("flight_altitude", 3.0).value
        self.arrival_radius = p("arrival_radius", 1.0).value
        self.cruise_speed = float(p("cruise_speed", 0.3).value)
        self.turn_slow_speed = float(p("turn_slow_speed", 0.12).value)
        self.turn_slow_radius = float(p("turn_slow_radius", 2.0).value)
        self.turn_angle_threshold = math.radians(float(p("turn_angle_threshold_deg", 20.0).value))
        self.turn_settle_time = max(0.0, float(p("turn_settle_time", 0.0).value))
        self.hold_time = p("hold_time", 1.0).value
        self.leg_timeout = p("leg_timeout", 60.0).value     # 單段逾時就中止
        self.face_travel = p("face_travel_direction", True).value
        self.land_at_goal = p("land_at_goal", True).value
        self.scan_yaw_nodes = {
            int(n) for n in p("scan_yaw_nodes", [-1]).value if int(n) >= 0
        }
        self.scan_yaw_speed = math.radians(float(p("scan_yaw_speed_deg_s", 18.0).value))
        self.scan_yaw_turns = max(0.0, float(p("scan_yaw_turns", 1.0).value))
        self.scan_yaw_hold_time = max(0.0, float(p("scan_yaw_hold_time", 0.5).value))
        self.save_map_nodes = {
            int(n) for n in p("save_map_nodes", [-1]).value if int(n) >= 0
        }
        self.save_map_file = str(
            p("save_map_file", "/home/zhg/ncrl_mqtt/maps/arena").value
        )
        self.saved_map_nodes = set()
        # 起飛點在世界座標裡的位置。PX4 的 local NED 原點是 EKF 初始化時
        # 飛機所在的位置，也就是 spawn 點。預設用起點節點的座標，
        # 因為 start_arena_sitl.sh 就是把飛機 spawn 在那裡。
        self.origin_x = p("origin_x", float("nan")).value
        self.origin_y = p("origin_y", float("nan")).value

        # ---------- 讀圖 ----------
        self.nodes = self._load_graph(self.graph_file)
        if not self.nodes:
            self.get_logger().error(f"讀不到任何節點： {self.graph_file}")
            raise SystemExit(1)
        if self.start_node < 0:
            self.start_node = min(self.nodes)
        if self.goal_node < 0:
            self.goal_node = max(self.nodes)
        if math.isnan(self.origin_x):
            self.origin_x, self.origin_y = self.nodes[self.start_node]

        # ---------- QoS ----------
        # 訂閱 /fmu/out/* 一定要 BEST_EFFORT。PX4 的 uXRCE-DDS client 是用
        # BEST_EFFORT 發布的，而 rclpy 預設是 RELIABLE —— 兩者不相容，
        # 訂閱會「安靜地」收不到任何資料，不會有任何錯誤訊息。
        # （`ros2 topic echo` 看得到是因為它會自動匹配 QoS，所以更容易誤判。）
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)
        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        ns = self.px4_ns
        # 版本後綴規則（PX4 v1.17，src/modules/uxrce_dds_client/utilities.hpp:34-41）：
        # 只有 .msg 裡 MESSAGE_VERSION != 0 的訊息才有 _vN。實測：
        #   VehicleLocalPosition = 1、VehicleStatus = 1  -> 有 _v1
        #   TrajectorySetpoint / OffboardControlMode / VehicleCommand = 0 -> 沒有
        # 無腦全加或全不加都會錯。
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

        # ---------- 內部狀態 ----------
        self.pos = None                # 最新的 VehicleLocalPosition
        self.status = None             # 最新的 VehicleStatus
        self.route = []                # route_server 回答的節點順序
        self.leg = 0                   # 現在飛第幾段
        self.ticks = 0                 # 進入目前狀態之後過了幾個 tick
        self.hold_ticks = 0
        self.command_x = None
        self.command_y = None
        self.scan_node = None
        self.scan_x = None
        self.scan_y = None
        self.scan_total_ticks = 0
        self.scan_hold_total_ticks = 0
        self.turn_settle_node = None
        self.turn_settle_x = None
        self.turn_settle_y = None
        self.turn_settle_yaw = float("nan")
        self.turn_settle_total_ticks = 0
        self.scanned_nodes = set()
        self.state = S_WAIT_ROUTE if self.use_route_server else S_WAIT_FCU
        self.route_requested = False
        self.was_offboard = False      # 用來偵測「被失效保護踢出 offboard」
        # 結束旗標。不在 timer callback 裡直接呼叫 rclpy.shutdown()：
        # 那樣會讓 spin() 卡住不返回，程序印完「任務結束」還是不會退出（實測過）。
        # 改成設旗標、由 main() 的迴圈自己跳出，行為明確可預期。
        self.finished = False
        self.exit_code = 0

        if not self.use_route_server:
            self.route = [n for n in self.node_sequence if n in self.nodes]
            self.get_logger().warn(
                f"use_route_server=False —— 順序是手動指定的： {self.route}。"
                "這樣不會用到拓樸圖的 cost，只是把它當座標清單用。")

        self.route_client = ActionClient(self, ComputeRoute, "compute_route")

        self._print_banner()
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._loop)

    # =====================================================================
    #  讀檔與座標轉換
    # =====================================================================

    def _load_graph(self, path):
        """只取節點 id 與座標。邊在這裡用不到 —— 順序是 route_server 給的。"""
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        out = {}
        for feat in data.get("features", []):
            if feat.get("geometry", {}).get("type") != "Point":
                continue
            nid = feat.get("properties", {}).get("id")
            xy = feat["geometry"]["coordinates"]
            if nid is not None:
                out[int(nid)] = (float(xy[0]), float(xy[1]))
        return out

    def enu_to_ned(self, ex, en):
        """世界 ENU 座標 -> PX4 的 local NED 座標。

        兩件事同時發生，分開想比較不會錯：
          1. 軸的定義不同：ENU 是 (東, 北, 上)，NED 是 (北, 東, 下)。
             前兩軸互換，第三軸變號。
          2. 原點不同：地圖的原點是世界原點，PX4 的原點是 EKF 啟動時
             飛機所在的位置（也就是 spawn 點），所以要先減掉偏移。
        """
        return (en - self.origin_y,        # NED x = 北
                ex - self.origin_x)        # NED y = 東

    def _print_banner(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  圖檔       : {self.graph_file}")
        self.get_logger().info(f"  節點數     : {len(self.nodes)}")
        self.get_logger().info(f"  起點 -> 終點 : {self.start_node} -> {self.goal_node}")
        self.get_logger().info(f"  順序來源   : "
                               f"{'route_server' if self.use_route_server else '手動指定'}")
        self.get_logger().info(f"  local NED 原點 (世界 ENU) : "
                               f"({self.origin_x}, {self.origin_y})")
        self.get_logger().info(f"  巡航高度   : {self.flight_altitude} m")
        self.get_logger().info(f"  到達半徑   : {self.arrival_radius} m")
        self.get_logger().info(f"  移動速度   : {self.cruise_speed} m/s")
        self.get_logger().info(f"  轉彎降速   : {self.turn_slow_speed} m/s within {self.turn_slow_radius} m")
        if self.turn_settle_time > 0.0:
            self.get_logger().info(f"  轉彎等待   : {self.turn_settle_time:.1f} s")
        if self.scan_yaw_nodes:
            self.get_logger().info(
                "  原地掃描節點 : "
                + ", ".join(str(n) for n in sorted(self.scan_yaw_nodes))
                + f" / {self.scan_yaw_turns:.1f} turn @ "
                + f"{math.degrees(self.scan_yaw_speed):.1f} deg/s")
        if self.save_map_nodes:
            self.get_logger().info(
                "  存圖節點   : " + ", ".join(str(n) for n in sorted(self.save_map_nodes)))
        self.get_logger().info(f"  PX4 namespace / target_system : "
                               f"'{self.px4_ns}' / {self.target_system}")
        self.get_logger().info("=" * 60)

    def _goto(self, new_state, why=""):
        if new_state == self.state:
            return
        self.get_logger().info(f"[狀態] {self.state} -> {new_state}"
                               + (f"   （{why}）" if why else ""))
        self.state = new_state
        self.ticks = 0

    # =====================================================================
    #  訂閱回呼
    # =====================================================================

    def _on_local_position(self, msg):
        self.pos = msg

    def _on_status(self, msg):
        self.status = msg

    def is_armed(self):
        return self.status is not None and \
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_offboard(self):
        return self.status is not None and \
            self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    # =====================================================================
    #  發布
    # =====================================================================

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self):
        """告訴 PX4「我接下來要用哪一種 setpoint」。

        這個訊息必須跟 setpoint 一起持續發，斷了 PX4 就會退出 offboard。
        position=True 代表走位置控制 —— 這條路實機已經驗證過。
        """
        m = OffboardControlMode()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.timestamp = self.now_us()
        self.pub_ocm.publish(m)

    def publish_setpoint(self, ned_x, ned_y, ned_z, yaw=float("nan"), yawspeed=float("nan")):
        m = TrajectorySetpoint()
        m.position = [float(ned_x), float(ned_y), float(ned_z)]
        # NaN 代表「這一項不要控制」。速度/加速度留 NaN，
        # 由 PX4 自己依位置誤差算 —— 這是位置控制的標準用法。
        m.velocity = [float("nan")] * 3
        m.acceleration = [float("nan")] * 3
        m.yaw = float(yaw)
        m.yawspeed = float(yawspeed)
        m.timestamp = self.now_us()
        self.pub_sp.publish(m)

    def send_command(self, command, param1=0.0, param2=0.0):
        m = VehicleCommand()
        m.command = int(command)
        m.param1 = float(param1)
        m.param2 = float(param2)
        m.target_system = int(self.target_system)
        m.target_component = 1        # 1 = autopilot
        m.source_system = 1
        m.source_component = 1
        # from_external 一定要 True，否則 PX4 會把它當內部指令而拒絕
        m.from_external = True
        m.timestamp = self.now_us()
        self.pub_cmd.publish(m)

    # =====================================================================
    #  route_server
    # =====================================================================

    def request_route(self):
        if not self.route_client.server_is_ready():
            return
        goal = ComputeRoute.Goal()
        goal.start_id = int(self.start_node)
        goal.goal_id = int(self.goal_node)
        # 這兩個 False 是關鍵：用節點 id 查詢就不會去查 TF。
        # 設 True 的話 route_server 會要求 base_link->map 的轉換，
        # 而那是階段 1 的東西，現在還沒有。
        goal.use_start = False
        goal.use_poses = False
        self.get_logger().info(
            f"向 route_server 要路線： 節點 {self.start_node} -> {self.goal_node}")
        self.route_client.send_goal_async(goal).add_done_callback(self._on_goal_sent)
        self.route_requested = True

    def _on_goal_sent(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("route_server 拒絕了這個請求")
            self._goto(S_ABORT, "route 被拒絕")
            return
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_result(self, future):
        res = future.result().result
        ids = [n.nodeid for n in res.route.nodes]
        if not ids:
            self.get_logger().error("route_server 回了空路線")
            self._goto(S_ABORT, "空路線")
            return
        missing = [i for i in ids if i not in self.nodes]
        if missing:
            # route_server 讀的是同一份 geojson，理論上不該發生。
            # 真的發生就代表兩邊讀到不同版本的檔案 —— 這比飛錯更嚴重，直接停。
            self.get_logger().error(
                f"route_server 給的節點 {missing} 在我讀的圖檔裡不存在 —— "
                "兩邊可能讀到不同版本的 geojson")
            self._goto(S_ABORT, "圖檔不一致")
            return
        self.route = ids
        self.get_logger().info(
            f"拿到路線（cost {res.route.route_cost:.2f}）： "
            + " -> ".join(str(i) for i in ids))
        self._goto(S_WAIT_FCU, "路線已取得")

    # =====================================================================
    #  主迴圈
    # =====================================================================

    def _loop(self):
        self.ticks += 1

        if self.state == S_WAIT_ROUTE:
            if not self.route_requested:
                if self.ticks % 20 == 1:
                    self.request_route()
                    if not self.route_requested:
                        self.get_logger().info(
                            "等 /compute_route action server…（route_server 有開嗎？）",
                            throttle_duration_sec=5.0)
            return

        if self.state == S_WAIT_FCU:
            if self.pos is not None and self.pos.xy_valid and self.pos.z_valid:
                self._goto(S_WARMUP, "PX4 位置估計已收斂")
            else:
                self.get_logger().info(
                    "等 PX4 的 VehicleLocalPosition…"
                    "（沒反應的話檢查 MicroXRCEAgent 和 QoS）",
                    throttle_duration_sec=5.0)
            return

        # 從這裡開始，每一個 tick 都必須發 offboard_control_mode + setpoint。
        # 斷流超過大約 0.5 秒 PX4 就會退出 offboard。
        if self.state in (S_WARMUP, S_ARMING, S_TAKEOFF, S_FLY, S_YAW_SCAN, S_TURN_SETTLE):
            self._publish_current_setpoint()

        # 失效保護偵測：已經進過 offboard 卻又掉出來，代表 PX4 主動接管了
        # （低電量、位置估計失效、地面站斷線…）。這時候繼續灌 setpoint 沒有意義，
        # 而且會掩蓋真正的原因，所以直接停手並把 nav_state 印出來。
        if self.was_offboard and not self.is_offboard() and self.state in (S_TAKEOFF, S_FLY, S_YAW_SCAN, S_TURN_SETTLE):
            self.get_logger().error(
                f"被踢出 offboard！ nav_state={self.status.nav_state} "
                f"（14 才是 OFFBOARD）。多半是失效保護觸發，去看 PX4 的 log。")
            self._goto(S_ABORT, "失去 offboard 控制權")
            return
        if self.is_offboard():
            self.was_offboard = True

        handler = {
            S_WARMUP: self._do_warmup,
            S_ARMING: self._do_arming,
            S_TAKEOFF: self._do_takeoff,
            S_FLY: self._do_fly,
            S_YAW_SCAN: self._do_yaw_scan,
            S_TURN_SETTLE: self._do_turn_settle,
            S_LANDING: self._do_landing,
            S_DONE: self._do_done,
            S_ABORT: self._do_abort,
        }.get(self.state)
        if handler:
            handler()

    def _publish_current_setpoint(self):
        self.publish_offboard_mode()
        if self.state == S_WARMUP:
            # 預熱階段還沒解鎖，送「維持現在的位置」最安全。
            self.publish_setpoint(self.pos.x, self.pos.y, self.pos.z)
        elif self.state == S_ARMING or self.state == S_TAKEOFF:
            sx, sy = self.enu_to_ned(*self.nodes[self.route[0]])
            self.publish_setpoint(sx, sy, -self.flight_altitude)
        elif self.state == S_FLY:
            tx, ty = self.enu_to_ned(*self.nodes[self.route[self.leg]])
            cx, cy = self._limited_target(tx, ty)
            self.publish_setpoint(cx, cy, -self.flight_altitude, self._yaw_to(tx, ty))
        elif self.state == S_YAW_SCAN:
            # 固定在節點上，用 yawspeed 連續旋轉；不要用跳角度，避免 PX4 走最短角。
            x = self.scan_x if self.scan_x is not None else self.pos.x
            y = self.scan_y if self.scan_y is not None else self.pos.y
            yawspeed = self.scan_yaw_speed if self.ticks <= self.scan_total_ticks else 0.0
            self.publish_setpoint(x, y, -self.flight_altitude, float("nan"), yawspeed)
        elif self.state == S_TURN_SETTLE:
            x = self.turn_settle_x if self.turn_settle_x is not None else self.pos.x
            y = self.turn_settle_y if self.turn_settle_y is not None else self.pos.y
            self.publish_setpoint(x, y, -self.flight_altitude, self.turn_settle_yaw, 0.0)

    def _limited_target(self, target_x, target_y):
        if self.command_x is None or self.command_y is None:
            self.command_x = self.pos.x
            self.command_y = self.pos.y

        dx = target_x - self.command_x
        dy = target_y - self.command_y
        dist = math.hypot(dx, dy)
        speed = self._current_xy_speed(target_x, target_y)
        max_step = max(speed, 0.05) / LOOP_HZ
        if dist <= max_step or dist <= 1e-6:
            self.command_x = target_x
            self.command_y = target_y
        else:
            scale = max_step / dist
            self.command_x += dx * scale
            self.command_y += dy * scale
        return self.command_x, self.command_y

    def _current_xy_speed(self, target_x, target_y):
        if not self._is_turning_leg():
            return self.cruise_speed
        if self.command_x is None or self.command_y is None:
            return self.cruise_speed
        if math.hypot(target_x - self.command_x, target_y - self.command_y) > self.turn_slow_radius:
            return self.cruise_speed
        return min(self.cruise_speed, self.turn_slow_speed)

    def _is_turning_leg(self):
        if self.state != S_FLY or self.leg <= 0 or self.leg >= len(self.route) - 1:
            return False
        prev_id = self.route[self.leg - 1]
        curr_id = self.route[self.leg]
        next_id = self.route[self.leg + 1]
        px, py = self.enu_to_ned(*self.nodes[prev_id])
        cx, cy = self.enu_to_ned(*self.nodes[curr_id])
        nx, ny = self.enu_to_ned(*self.nodes[next_id])
        v1x, v1y = cx - px, cy - py
        v2x, v2y = nx - cx, ny - cy
        n1 = math.hypot(v1x, v1y)
        n2 = math.hypot(v2x, v2y)
        if n1 < 1e-6 or n2 < 1e-6:
            return False
        dot = (v1x * v2x + v1y * v2y) / (n1 * n2)
        dot = max(-1.0, min(1.0, dot))
        angle = math.acos(dot)
        return angle >= self.turn_angle_threshold

    def _yaw_to(self, ned_x, ned_y):
        """機頭朝向行進方向。NED 的 yaw 從北開始、順時針為正，
        所以是 atan2(往東的分量, 往北的分量)。

        現在下視相機用不到，但之後掛前視光達（階段 4 的 costmap）就必須朝對方向。
        距離很近的時候不要轉，否則到達前會亂甩。
        """
        if not self.face_travel:
            return float("nan")
        dx, dy = ned_x - self.pos.x, ned_y - self.pos.y
        if math.hypot(dx, dy) < max(self.arrival_radius, 0.5):
            return float("nan")
        return math.atan2(dy, dx)

    # ---------- 各狀態 ----------

    def _do_warmup(self):
        # PX4 規定：切進 offboard 之前必須已經看到一段 setpoint 串流，
        # 否則切換會被拒絕。10 個 tick（1 秒）足夠。
        if self.ticks >= 10:
            self._goto(S_ARMING, "setpoint 串流已建立")

    def _do_arming(self):
        # 每 10 個 tick 重送一次。PX4 可能因為預檢還沒過而忽略前幾次，
        # 重送比等待可靠，而且不會有副作用（重複 arm 是冪等的）。
        if self.ticks % 10 == 1:
            # DO_SET_MODE: param1=1 是 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED，
            #              param2=6 是 PX4_CUSTOM_MAIN_MODE_OFFBOARD
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if self.is_offboard() and self.is_armed():
            self._goto(S_TAKEOFF, "已進入 offboard 並解鎖")
            return

        if self.ticks > LOOP_HZ * 30:
            self.get_logger().error(
                f"30 秒內沒能解鎖。armed={self.is_armed()} offboard={self.is_offboard()}。"
                f"常見原因：預檢沒過（NAV_DLL_ACT / CBRK_SUPPLY_CHK）、"
                f"target_system({self.target_system}) 填錯、setpoint 串流不穩。")
            self._goto(S_ABORT, "解鎖逾時")

    def _do_takeoff(self):
        err = abs(-self.flight_altitude - self.pos.z)
        if err < 0.5:
            self.leg = 0
            self.hold_ticks = 0
            self.command_x = self.pos.x
            self.command_y = self.pos.y
            self.get_logger().info(f"已到巡航高度 {self.flight_altitude} m，開始飛節點")
            self._goto(S_FLY, "爬升完成")
            return
        if self.ticks > LOOP_HZ * self.leg_timeout:
            self.get_logger().error(f"爬升逾時，還差 {err:.2f} m")
            self._goto(S_LANDING, "爬升逾時")

    def _do_fly(self):
        nid = self.route[self.leg]
        tx, ty = self.enu_to_ned(*self.nodes[nid])
        d = math.hypot(tx - self.pos.x, ty - self.pos.y)

        if self.ticks % 20 == 0:
            self.get_logger().info(
                f"飛往節點 {nid}  剩 {d:5.2f} m  "
                f"（第 {self.leg + 1}/{len(self.route)} 段）")

        if d < self.arrival_radius:
            self.hold_ticks += 1
            if self.hold_ticks == 1:
                self.get_logger().info(f"  ✓ 到達節點 {nid}（誤差 {d:.2f} m）")
            if self.hold_ticks >= LOOP_HZ * self.hold_time:
                self.hold_ticks = 0
                if self._should_yaw_scan(nid):
                    self._start_yaw_scan(nid, tx, ty)
                elif self._should_turn_settle():
                    self._start_turn_settle(nid, tx, ty)
                else:
                    self._advance_to_next_leg()
                return
        else:
            self.hold_ticks = 0

        if self.ticks > LOOP_HZ * self.leg_timeout:
            self.get_logger().error(
                f"飛往節點 {nid} 逾時（{self.leg_timeout} 秒），還差 {d:.2f} m。"
                "可能是這一段被牆擋住了 —— 去跑 check_graph.py 看看。")
            self._goto(S_LANDING, "單段逾時")



    def _should_turn_settle(self):
        return self.turn_settle_time > 0.0 and self._is_turning_leg()

    def _start_turn_settle(self, node_id, target_x, target_y):
        next_id = self.route[self.leg + 1]
        next_x, next_y = self.enu_to_ned(*self.nodes[next_id])
        self.turn_settle_node = node_id
        self.turn_settle_x = target_x
        self.turn_settle_y = target_y
        self.turn_settle_yaw = math.atan2(next_y - target_y, next_x - target_x)
        self.turn_settle_total_ticks = int(math.ceil(self.turn_settle_time * LOOP_HZ))
        self.command_x = target_x
        self.command_y = target_y
        self.get_logger().info(
            f"節點 {node_id} 轉彎前停住 {self.turn_settle_time:.1f} 秒，"
            f"先對準下一段節點 {next_id}")
        self._goto(S_TURN_SETTLE, f"節點 {node_id} 轉彎等待")

    def _do_turn_settle(self):
        if self.ticks == 1 or self.ticks % 10 == 0:
            self.get_logger().info(
                f"節點 {self.turn_settle_node} 轉彎等待 "
                f"（{self.ticks}/{self.turn_settle_total_ticks} tick）")
        if self.ticks >= self.turn_settle_total_ticks:
            self.get_logger().info(f"節點 {self.turn_settle_node} 轉彎等待完成")
            self.turn_settle_node = None
            self.turn_settle_x = None
            self.turn_settle_y = None
            self.turn_settle_yaw = float("nan")
            self._advance_to_next_leg()

    def _should_yaw_scan(self, node_id):
        return node_id in self.scan_yaw_nodes and node_id not in self.scanned_nodes

    def _start_yaw_scan(self, node_id, target_x, target_y):
        speed = max(abs(self.scan_yaw_speed), math.radians(1.0))
        self.scan_node = node_id
        self.scan_x = target_x
        self.scan_y = target_y
        self.scan_total_ticks = int(math.ceil(
            (2.0 * math.pi * self.scan_yaw_turns / speed) * LOOP_HZ))
        self.scan_hold_total_ticks = int(math.ceil(self.scan_yaw_hold_time * LOOP_HZ))
        self.command_x = target_x
        self.command_y = target_y
        self.get_logger().info(
            f"節點 {node_id} 原地旋轉掃描 "
            f"{self.scan_yaw_turns:.1f} 圈，速度 {math.degrees(speed):.1f} deg/s")
        self._goto(S_YAW_SCAN, f"節點 {node_id} 掃描")

    def _do_yaw_scan(self):
        rotate_done = self.ticks >= self.scan_total_ticks
        total_done = self.ticks >= self.scan_total_ticks + self.scan_hold_total_ticks
        if self.ticks == 1 or self.ticks % 20 == 0:
            phase = "停住收斂" if rotate_done else "旋轉中"
            self.get_logger().info(
                f"節點 {self.scan_node} 原地掃描 {phase} "
                f"（{self.ticks}/{self.scan_total_ticks + self.scan_hold_total_ticks} tick）")
        if total_done:
            node_id = self.scan_node
            self.scanned_nodes.add(node_id)
            self.get_logger().info(f"節點 {node_id} 原地掃描完成")
            self._maybe_save_map(node_id)
            self.scan_node = None
            self.scan_x = None
            self.scan_y = None
            self._advance_to_next_leg()

    def _maybe_save_map(self, node_id):
        if node_id not in self.save_map_nodes or node_id in self.saved_map_nodes:
            return
        filename = os.path.abspath(os.path.expanduser(self.save_map_file))
        out_dir = os.path.dirname(filename)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        cmd = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            filename,
        ]
        self.get_logger().info(f"節點 {node_id} 觸發 Nav2 map_saver 存圖: {filename}")
        try:
            subprocess.Popen(cmd)
            self.saved_map_nodes.add(node_id)
        except OSError as exc:
            self.get_logger().error(f"啟動存圖指令失敗: {exc}")

    def _advance_to_next_leg(self):
        self.leg += 1
        self.command_x = self.pos.x
        self.command_y = self.pos.y
        self.ticks = 0
        if self.leg >= len(self.route):
            self.get_logger().info("全部節點飛完")
            self._goto(S_LANDING if self.land_at_goal else S_DONE, "路線完成")
        else:
            self._goto(S_FLY, "前往下一節點")

    def _do_landing(self):
        # 不自己算下降的 setpoint，送 NAV_LAND 交給 PX4 —— 它有著陸偵測，
        # 會自己判斷觸地並上鎖，比我們猜一個高度可靠。
        if self.ticks == 1 or self.ticks % 20 == 1:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if not self.is_armed() and self.ticks > LOOP_HZ * 2:
            self.get_logger().info("已著陸並上鎖")
            self._goto(S_DONE, "降落完成")
        elif self.ticks > LOOP_HZ * 60:
            self.get_logger().warn("降落逾時，不再等待")
            self._goto(S_DONE, "降落逾時")

    def _do_done(self):
        # 明確結束。之前 offboard_takeoff_node 進 DONE 之後不會退出，
        # 每次都要手動 Ctrl+C，跑批次測試很麻煩。
        self.get_logger().info("任務結束，關閉節點")
        self.timer.cancel()
        self.exit_code = 0
        self.finished = True

    def _do_abort(self):
        self.get_logger().error("已中止。沒有繼續送 setpoint —— "
                                "如果飛機還在空中，PX4 的失效保護會接手。")
        self.timer.cancel()
        self.exit_code = 1
        self.finished = True


def main():
    rclpy.init()
    try:
        node = FlyNodes()
    except SystemExit as e:
        rclpy.shutdown()
        return int(e.code or 1)

    code = 0
    try:
        # 用 spin_once 迴圈而不是 spin()：這樣結束條件在我們手上，
        # 不必從 callback 裡去戳 shutdown（那個會卡住）。
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
    # 回傳值讓 launch 和批次腳本判斷成敗：0 完成、1 中止
    return code


if __name__ == "__main__":
    sys.exit(main())
