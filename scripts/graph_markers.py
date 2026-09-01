#!/usr/bin/env python3
# =============================================================================
#  graph_markers.py — 把場地與拓樸圖畫進 RViz（T2）
#
#  發兩個 topic：
#      /arena_walls   場地的牆（從 .sdf 讀）
#      /route_graph   拓樸圖的節點與邊（從 .geojson 讀）
#
#  為什麼要有這個（跟 T1 抓的錯不一樣）：
#      T1 比對的是「檔案 vs 檔案」，它活在檔案的世界裡，不知道 ROS 執行起來
#      之後長什麼樣。如果階段 1 的 px4_tf_node 把 NED->ENU 轉錯（x/y 互換），
#      檔案完全沒問題、T1 全部通過、route_server 照常規劃 ——
#      但無人機在 RViz 裡會出現在沿著對角線鏡射過去的位置。
#      這一類錯誤唯一能一眼看出來的方法，就是把兩邊畫在同一張圖上。
#
#      這個專案已經踩過兩次座標系的坑（OptiTrack 的 NED 轉了 90 度、
#      start_3_px4.sh 的註解把 NED/ENU 講反），所以不是假設性的風險。
#
#  為什麼連牆也一起畫：
#      只畫圖的話，RViz 裡是一堆浮在空中的點線，看不出對錯。
#      有牆當背景，才能一眼判斷「節點有沒有在該在的地方」。
#      而且之後階段 4 的 costmap 出來，可以直接跟這裡的牆疊起來比對。
#
#  用法：
#      ros2 launch drone_nav2_apriltag view_graph.launch.py
# =============================================================================

import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
import xml.etree.ElementTree as ET

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

# px4_msgs 只有在要追蹤無人機時才需要。沒裝也應該還能單純看地圖，
# 所以這裡容忍 import 失敗，而不是讓整個節點起不來。
try:
    from px4_msgs.msg import VehicleLocalPosition
    HAVE_PX4 = True
except ImportError:
    HAVE_PX4 = False


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


C_WALL = rgba(0.35, 0.35, 0.42, 0.35)     # 半透明，才看得到裡面的節點
C_NODE = rgba(0.12, 0.47, 0.71, 1.0)
C_EDGE = rgba(0.12, 0.47, 0.71, 0.9)
C_SLOW = rgba(0.85, 0.31, 0.17, 1.0)      # 被加了代價的邊（束縮段）
C_TEXT = rgba(1.0, 1.0, 1.0, 1.0)
C_TAG = rgba(0.05, 0.05, 0.05, 1.0)
C_DRONE = rgba(0.18, 0.80, 0.44, 1.0)     # 無人機本體
C_TRAIL = rgba(1.00, 0.75, 0.10, 0.95)    # 飛過的軌跡，跟藍色的邊區分開


class GraphMarkers(Node):

    def __init__(self):
        super().__init__("graph_markers")

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        p = self.declare_parameter
        self.world_file = p("world_file",
                            os.path.join(pkg_dir, "gz", "worlds", "nav2_arena.sdf")).value
        self.graph_file = p("graph_file",
                            os.path.join(pkg_dir, "graphs", "nav2_arena.geojson")).value
        self.frame_id = p("frame_id", "map").value
        # 牆實際上 15 m 高，照實畫會擋住整個畫面。這裡只畫矮矮一截當輪廓，
        # 反正 Nav2 是 2D 的，高度在視覺化上沒有資訊量。
        self.wall_draw_height = p("wall_draw_height", 1.5).value
        self.period = p("republish_period", 2.0).value

        # ---- 即時追蹤無人機（T2 真正的價值所在）----
        # 只畫地圖的話，看到的永遠是「檔案畫出來的樣子」，跟執行中的系統對不對得上
        # 完全看不出來。把無人機的實際位置疊上去，座標系錯了就一眼看得到。
        self.track_vehicle = p("track_vehicle", True).value
        self.px4_ns = p("px4_namespace", "").value
        # PX4 的 local NED 原點 = EKF 啟動時飛機所在的位置 = spawn 點。
        # 預設用最小 id 的節點座標，跟 fly_nodes.py 的慣例一致
        # （start_arena_sitl.sh 就是把飛機 spawn 在那裡）。
        self.origin_x = p("origin_x", float("nan")).value
        self.origin_y = p("origin_y", float("nan")).value
        self.trail_min_step = p("trail_min_step", 0.25).value   # 移動超過這麼多才記一點
        self.trail_max_points = p("trail_max_points", 4000).value

        # TRANSIENT_LOCAL：晚一步才開的 RViz 也收得到最後一筆。
        # 沒設的話你會遇到「先開這個節點再開 RViz 就什麼都沒有」的情況。
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub_walls = self.create_publisher(MarkerArray, "arena_walls", qos)
        self.pub_graph = self.create_publisher(MarkerArray, "route_graph", qos)

        self.walls, self.tag = self._load_world(self.world_file)
        self.nodes, self.edges = self._load_graph(self.graph_file)

        if math.isnan(self.origin_x) and self.nodes:
            self.origin_x, self.origin_y = self.nodes[min(self.nodes)]

        # 無人機的位置是持續變動的即時資料，不需要（也不該）用 TRANSIENT_LOCAL 留存
        self.pub_vehicle = self.create_publisher(MarkerArray, "vehicle_marker", 5)
        self.vehicle_enu = None
        self.vehicle_heading = 0.0
        self.trail = []

        if self.track_vehicle:
            if not HAVE_PX4:
                self.get_logger().warn("找不到 px4_msgs，無法追蹤無人機，只畫地圖")
                self.track_vehicle = False
            else:
                # 訂閱 /fmu/out/* 一定要 BEST_EFFORT，PX4 的 uXRCE-DDS client 是用
                # BEST_EFFORT 發布的。用預設的 RELIABLE 會「安靜地」收不到任何東西。
                px4_qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST, depth=5)
                topic = self.px4_ns + "/fmu/out/vehicle_local_position_v1"
                self.create_subscription(VehicleLocalPosition, topic,
                                         self._on_vehicle, px4_qos)
                # 位置以 50 Hz 進來，但畫面沒必要那麼快，10 Hz 綽綽有餘
                self.create_timer(0.1, self._publish_vehicle)
                self.get_logger().info(f"追蹤無人機： {topic}")
                self.get_logger().info(
                    f"local NED 原點 (世界 ENU) = ({self.origin_x}, {self.origin_y})")

        self.get_logger().info(
            f"牆 {len(self.walls)} 面 / 節點 {len(self.nodes)} 個 / "
            f"邊 {len(self.edges)} 條   frame_id = {self.frame_id}")
        self.get_logger().info("RViz 裡加兩個 MarkerArray：/arena_walls 和 /route_graph，"
                               f"Fixed Frame 設成 {self.frame_id}")

        self._publish()
        # 週期重發：TRANSIENT_LOCAL 已經處理了晚加入的訂閱者，
        # 但 RViz 偶爾會在重連後漏掉快取，重發一次成本極低。
        self.create_timer(self.period, self._publish)

    # ---------- 讀檔 ----------

    def _load_world(self, path):
        """從 .sdf 撈方塊障礙物與 AprilTag。跟 check_graph.py 用同一套解析邏輯。"""
        root = ET.parse(path).getroot()
        world = root.find("world")
        walls = []
        for model in world.findall("model"):
            size_el = model.find(".//collision/geometry/box/size")
            if size_el is None:
                continue                      # ground_plane 之類的非方塊
            sx, sy, sz = [float(t) for t in size_el.text.split()]
            pose_el = model.find("pose")
            v = [float(t) for t in (pose_el.text if pose_el is not None
                                    else "0 0 0 0 0 0").split()]
            while len(v) < 6:
                v.append(0.0)
            walls.append((model.get("name", ""), v[0], v[1], sx, sy, v[5]))
        tag = None
        for inc in world.findall("include"):
            pose_el = inc.find("pose")
            if pose_el is not None:
                v = [float(t) for t in pose_el.text.split()]
                tag = (v[0], v[1])
        return walls, tag

    def _load_graph(self, path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        nodes, edges = {}, []
        for feat in data.get("features", []):
            props, geom = feat.get("properties", {}), feat.get("geometry", {})
            if geom.get("type") == "Point":
                xy = geom["coordinates"]
                nodes[int(props["id"])] = (float(xy[0]), float(xy[1]))
            elif geom.get("type") == "MultiLineString":
                meta = props.get("metadata", {})
                edges.append((props.get("startid"), props.get("endid"),
                              float(meta.get("speed_limit", 1.0)),
                              float(meta.get("penalty", 0.0))))
        return nodes, edges

    # ---------- 無人機 ----------

    def _on_vehicle(self, msg):
        if not (msg.xy_valid and msg.z_valid):
            return
        # NED -> ENU。這是 fly_nodes.py 裡 enu_to_ned() 的反向操作：
        #   當時是 ned_x = enu_y - origin_y、ned_y = enu_x - origin_x
        # 所以反過來就是下面兩行。z 要變號，因為 NED 的 z 是「向下為正」。
        self.vehicle_enu = (msg.y + self.origin_x,
                            msg.x + self.origin_y,
                            -msg.z)
        # PX4 的 heading 是 NED 的 yaw：從「北」起算、順時針為正。
        # RViz 在 ENU 裡的 yaw 是從「東」起算、逆時針為正，所以要換算。
        self.vehicle_heading = math.pi / 2.0 - msg.heading

        x, y, _ = self.vehicle_enu
        if not self.trail or math.hypot(x - self.trail[-1][0],
                                        y - self.trail[-1][1]) > self.trail_min_step:
            self.trail.append(self.vehicle_enu)
            if len(self.trail) > self.trail_max_points:
                self.trail.pop(0)

    def _publish_vehicle(self):
        if self.vehicle_enu is None:
            return
        x, y, z = self.vehicle_enu
        arr = MarkerArray()

        body = self._base("drone", 0, Marker.CYLINDER)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, z
        body.scale = Vector3(x=0.6, y=0.6, z=0.15)
        body.color = C_DRONE
        arr.markers.append(body)

        # 機頭方向。之後要接前視光達的話，這個箭頭指對不對很重要。
        head = self._base("drone", 1, Marker.ARROW)
        head.pose.position.x, head.pose.position.y, head.pose.position.z = x, y, z
        head.pose.orientation.z = math.sin(self.vehicle_heading / 2.0)
        head.pose.orientation.w = math.cos(self.vehicle_heading / 2.0)
        head.scale = Vector3(x=1.5, y=0.18, z=0.18)   # ARROW 的 x 是長度
        head.color = C_DRONE
        arr.markers.append(head)

        # 從地面拉一條垂直線到機身：俯視角度下高度看不出來，有這條線才判斷得出
        drop = self._base("drone", 2, Marker.LINE_LIST)
        drop.scale.x = 0.05
        drop.color = rgba(0.18, 0.80, 0.44, 0.5)
        drop.points.append(Point(x=x, y=y, z=0.0))
        drop.points.append(Point(x=x, y=y, z=z))
        arr.markers.append(drop)

        alt = self._base("drone", 3, Marker.TEXT_VIEW_FACING)
        alt.pose.position.x, alt.pose.position.y, alt.pose.position.z = x, y, z + 1.2
        alt.scale.z = 0.8
        alt.color = C_TEXT
        alt.text = f"{z:.1f} m"
        arr.markers.append(alt)

        # 飛過的軌跡。這是最有用的一條 —— 疊在藍色的邊上面，
        # 就能直接看出實際飛的路線跟拓樸圖規劃的路線一不一致。
        if len(self.trail) >= 2:
            tr = self._base("trail", 0, Marker.LINE_STRIP)
            tr.scale.x = 0.12
            tr.color = C_TRAIL
            for tx, ty, tz in self.trail:
                tr.points.append(Point(x=tx, y=ty, z=tz))
            arr.markers.append(tr)

        self.pub_vehicle.publish(arr)

    # ---------- 產生 marker ----------

    def _base(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns              # RViz 可以用 namespace 個別開關，所以分得細一點好用
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0     # 0 = 永不過期
        return m

    def _wall_markers(self):
        arr = MarkerArray()
        for i, (name, cx, cy, sx, sy, yaw) in enumerate(self.walls):
            m = self._base("walls", i, Marker.CUBE)
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = self.wall_draw_height / 2.0
            # 繞 z 軸轉 yaw 的四元數只有兩個分量不為零
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale = Vector3(x=sx, y=sy, z=float(self.wall_draw_height))
            m.color = C_WALL
            arr.markers.append(m)

        if self.tag:
            m = self._base("apriltag", 0, Marker.CUBE)
            m.pose.position.x, m.pose.position.y = self.tag
            m.pose.position.z = 0.05
            m.scale = Vector3(x=1.0, y=1.0, z=0.1)   # 1.0 m = 標記本體外緣
            m.color = C_TAG
            arr.markers.append(m)
        return arr

    def _graph_markers(self):
        arr = MarkerArray()

        # 節點用 SPHERE_LIST 而不是每個一個 Marker：同一種東西合成一個 marker
        # RViz 畫起來快得多，數量大的時候差別明顯。
        sph = self._base("nodes", 0, Marker.SPHERE_LIST)
        sph.scale = Vector3(x=0.9, y=0.9, z=0.9)
        sph.color = C_NODE
        for x, y in self.nodes.values():
            sph.points.append(Point(x=x, y=y, z=0.0))
        arr.markers.append(sph)

        # 邊用 LINE_LIST：points 兩兩一組構成一條線段。
        # 逐點指定顏色，這樣被加了代價的邊可以塗成紅色，一眼看出差別。
        lines = self._base("edges", 0, Marker.LINE_LIST)
        lines.scale.x = 0.18
        lines.color = C_EDGE
        drawn = set()
        for s, e, speed, penalty in self.edges:
            if s not in self.nodes or e not in self.nodes:
                continue
            key = (min(s, e), max(s, e))
            if key in drawn:
                continue          # 有向邊有來回兩筆，畫一次就好
            drawn.add(key)
            col = C_SLOW if (speed < 1.0 or penalty > 0.0) else C_EDGE
            for nid in (s, e):
                x, y = self.nodes[nid]
                lines.points.append(Point(x=x, y=y, z=0.0))
                lines.colors.append(col)
        arr.markers.append(lines)

        # 節點編號。TEXT_VIEW_FACING 會永遠正對鏡頭，轉視角也讀得到。
        for nid, (x, y) in self.nodes.items():
            t = self._base("labels", nid, Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 1.2
            t.scale.z = 1.0        # TEXT 只用 scale.z 當字高
            t.color = C_TEXT
            t.text = str(nid)
            arr.markers.append(t)
        return arr

    def _publish(self):
        self.pub_walls.publish(self._wall_markers())
        self.pub_graph.publish(self._graph_markers())


def main():
    rclpy.init()
    node = GraphMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
