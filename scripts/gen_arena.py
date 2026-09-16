#!/usr/bin/env python3
# =============================================================================
#  gen_arena.py — 場地幾何的唯一真實來源
#
#  用法：
#      python3 src/drone_nav2_apriltag/scripts/gen_arena.py
#
#  跑一次會重新產生三個檔案：
#      gz/worlds/nav2_arena.sdf     Gazebo 世界（牆 + AprilTag）
#      graphs/nav2_arena.geojson    nav2_route 拓樸圖（節點 + 邊）
#      docs/arena_preview.png       俯視圖（沒裝 matplotlib 就跳過）
#
#  為什麼要有這支腳本：
#      world 的牆和 graph 的節點必須對得上，但它們是兩個不同格式的檔案。
#      手動改兩邊遲早會改到不同步，而且不同步不會報錯 —— 只會在飛的時候
#      莫名其妙撞牆。把幾何寫在同一個地方、一次產生兩個檔，從源頭杜絕。
#
#  改地圖的流程：
#      1. 改下面「幾何定義」那一段
#      2. 跑這支腳本
#      3. 跑 scripts/check_graph.py 驗證（T1，還沒寫）
#      4. colcon build --packages-select drone_nav2_apriltag
#
#  注意：這支腳本會「覆寫」上面三個檔案。不要直接手改那三個檔，改了會被蓋掉。
# =============================================================================

import json
import math
import os
import sys

# =============================================================================
#  幾何定義 —— 要改地圖就改這一段，其他都不用動
# =============================================================================

WALL_HEIGHT = 15.0   # 牆高。飛行高度約 3~5 m，15 m 保證飛不過去
WALL_THICK = 0.5

# 通道寬度的依據（不要憑感覺改）：
#   三角編隊外接圓半徑 2 m -> 橫向跨距 4 m -> 加機身約 4.5 m
#   Nav2 costmap 的膨脹層會把牆「加胖」，左右還要各留 2 m 以上
#   => 主通道 9 m。北道被 pinch 夾到 6 m，左右只剩 0.75 m，
#      刻意留成「勉強過得去」，這樣北道貴、南道便宜才有物理依據。
#
# 牆的格式：(名稱, 中心x, 中心y, x向長度, y向長度, yaw角度, 說明)
# 長度比理論值多 0.5 是為了讓轉角互相重疊，避免接縫漏一條縫。
WALLS = [
    ("wall_west",     -6.0,   0.0,  WALL_THICK,  22.5,  0, "起飛區 A 的西界"),
    ("wall_south",     6.5, -11.0,  25.5,  WALL_THICK,  0, "南界，同時是南道的外牆"),
    ("wall_north",     4.5,  11.0,  21.5,  WALL_THICK,  0, "北界，東端止於 x=15 接上斜走廊"),
    ("wall_east",     19.0,  -5.0,  WALL_THICK,  12.5,  0, "匯合區 M 的東界，北端止於 y=1 接上斜走廊"),
    ("divider",        6.5,   0.0,   9.0,         4.0,  0, "把直走廊劈成南北兩道。東端只到 x=11，"
                                                           "刻意不伸到匯合區，免得擋住轉進斜走廊的路"),
    ("pinch_north",    6.5,   9.5,   3.0,         3.0,  0, "北道的束縮塊：把北道從 9 m 夾到 6 m，"
                                                           "讓兩條路真的有優劣差別"),
    ("wall_diag_nw",  18.0,  14.0,   8.8,  WALL_THICK, 45, "斜走廊西北側牆，(15,11)->(21,17)"),
    ("wall_diag_se",  23.5,   5.5,  13.0,  WALL_THICK, 45, "斜走廊東南側牆，(19,1)->(28,10)"),
    ("wall_d_west",   21.0,  19.5,  WALL_THICK,   5.5,  0, "空地 D 的西界"),
    ("wall_d_north",  27.0,  22.0,  12.5,  WALL_THICK,  0, "空地 D 的北界"),
    ("wall_d_east",   33.0,  16.0,  WALL_THICK,  12.5,  0, "空地 D 的東界"),
    ("wall_d_south",  30.5,  10.0,   5.5,  WALL_THICK,  0, "空地 D 的南界，西端止於 x=28 接上斜走廊"),
]

TAG_POS = (27.0, 16.0)   # 空地 D 的正中央
TAG_NAME = "apriltag_36h11"

# 節點：id -> (x, y, 說明)。座標是 ENU，跟 world 同一個座標系。
NODES = {
    0:  (-2.0,  0.0, "起飛點，南北兩道的分岔處"),
    # 北道這三個節點刻意壓在 y=5 而不是走廊中線 y=6.5：
    # pinch_north 是從北牆往下長的，路線貼著南側走才閃得開它。
    # 原本放 6.5 的時候，1→2 這條邊離 pinch 只剩 2.500 m（T1 抓到的）。
    1:  ( 0.0,  5.0, "北道入口"),
    2:  ( 6.5,  5.0, "北道束縮段（只剩 6 m 寬）"),
    3:  (13.0,  5.0, "北道出口"),
    4:  ( 1.0, -6.5, "南道入口"),
    5:  ( 6.5, -6.5, "南道中段（全寬 9 m）"),
    6:  (13.0, -6.5, "南道出口"),
    7:  (15.0,  0.0, "匯合點"),
    8:  (20.0,  9.0, "斜走廊中段"),
    9:  (25.0, 14.0, "斜走廊出口／空地入口"),
    10: (27.0, 16.0, "降落點，地面有 AprilTag"),
}

# 邊：(起點, 終點, 路線標籤)。這裡只寫單向，產生時自動補反向。
LINKS = [
    (0, 1, "north"), (1, 2, "north"), (2, 3, "north"), (3, 7, "north"),
    (0, 4, "south"), (4, 5, "south"), (5, 6, "south"), (6, 7, "south"),
    (7, 8, "common"), (8, 9, "common"), (9, 10, "common"),
]

# 走束縮段要付的代價。speed_limit 給 DistanceScorer 用（成本 = 距離 / 速限），
# penalty 給 PenaltyScorer 用。兩個鍵名可在 route_server 參數裡用
# .speed_tag / .penalty_tag 改，這裡用的是預設值。
PINCHED_LINKS = {(1, 2), (2, 3)}
PINCH_SPEED_LIMIT = 0.5
PINCH_PENALTY = 20.0

# 到達降落點要觸發的動作。type 必須跟 route_server 參數檔宣告的實例名一致，
# 否則 OperationsManager 查表查不到，什麼都不會發生（也不會報錯）。
LANDING_OPERATION = {
    "scan_apriltag": {
        "type": "scan_apriltag",
        "trigger": "NODE",
        "metadata": {"service_name": "/start_apriltag_scan"},
    }
}

# =============================================================================
#  以下是產生邏輯，改地圖不需要動
# =============================================================================

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def stable_seed(*parts):
    s = 2166136261
    for part in parts:
        for ch in str(part):
            s ^= ord(ch)
            s = (s * 16777619) & 0xFFFFFFFF
    return s


def feature_color(seed):
    palette = [
        (0.02, 0.02, 0.02),
        (0.97, 0.97, 0.97),
        (0.02, 0.28, 0.95),
        (0.95, 0.62, 0.02),
        (0.02, 0.68, 0.28),
        (0.82, 0.04, 0.12),
        (0.55, 0.12, 0.88),
        (0.00, 0.72, 0.72),
    ]
    return palette[seed % len(palette)]


def wall_texture_xml(name, sx, sy):
    """Return no extra wall markers; keep walls geometrically simple."""
    return ""


def wall_model_xml(name, cx, cy, sx, sy, yaw_deg, note):
    yaw = math.radians(yaw_deg)
    size = f"{sx} {sy} {WALL_HEIGHT}"
    texture = wall_texture_xml(name, sx, sy)
    return f"""
    <!-- {note} -->
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {WALL_HEIGHT / 2} 0 0 {yaw:.10f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>0.3 0.3 0.35 1</ambient>
            <diffuse>0.65 0.65 0.7 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>{texture}
      </link>
    </model>
"""


def floor_feature_rects():
    return [
        (-4.5, -2.0, -9.0, 9.0),
        (-2.0, 15.5, 2.3, 9.2),
        (-2.0, 15.5, -9.2, -2.3),
        (11.5, 19.0, -7.5, 1.8),
        (20.5, 32.0, 10.5, 21.0),
    ]


def floor_texture_xml():
    """Return no extra floor markers; keep the floor as a plain plane."""
    return ""


def bounds():
    """算出場地外框，只用來寫進註解讓人一眼看到規模。"""
    xs, ys = [], []
    for _, cx, cy, sx, sy, yaw, _ in WALLS:
        a = math.radians(yaw)
        # 旋轉後的四個角
        for ex, ey in ((sx / 2, sy / 2), (sx / 2, -sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2)):
            xs.append(cx + ex * math.cos(a) - ey * math.sin(a))
            ys.append(cy + ex * math.sin(a) + ey * math.cos(a))
    return min(xs), max(xs), min(ys), max(ys)


def write_world():
    x0, x1, y0, y1 = bounds()
    header = f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  === 這個檔案是產生出來的，不要手改 ===
  來源：scripts/gen_arena.py，改幾何請改那支再重跑。

  nav2_arena：給 Nav2 定高導航 + AprilTag 降落用的場地。

  路線
  ====
  起飛區 A -> 直走廊 B（北道窄 6 m / 南道寬 9 m，二選一）-> 匯合區 M
           -> 斜走廊 C（45 度，左下往右上）-> 空地 D（正中央地面有 AprilTag）

  設計原則
  ========
  1. 所有障礙物都是 {WALL_HEIGHT:.0f} m 高的牆 —— 不放矮的、可飛越的東西。
     「從上面飛過去」是地圖設計上的取巧，不是真的 3D 避障。
  2. 通道寬度由編隊決定，不是由美觀決定：三角編隊跨距 4 m + 機身 = 4.5 m，
     再加 Nav2 costmap 膨脹層左右各 2 m 以上 => 主通道 9 m。
  3. 起點到終點有兩條路，route_server 才有得選。
  4. 兩條路條件不同（北道被夾到 6 m），選擇才有物理依據而不是擲骰子。

  座標系：ENU（x 東、y 北、z 上）—— 見下方 <world_frame_orientation>。
  PX4 內部是 NED，兩者前兩軸對調，寫任何座標前先確認自己在哪個系統。

  外框：x [{x0:.1f}, {x1:.1f}]  y [{y0:.1f}, {y1:.1f}]   ({x1 - x0:.0f} x {y1 - y0:.0f} m)
  AprilTag：({TAG_POS[0]:.0f}, {TAG_POS[1]:.0f})
-->
<sdf version="1.9">
  <world name="nav2_arena">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 為什麼要有 spherical_coordinates：PX4 的 GPS 模擬需要一個原點經緯度，
         而 world_frame_orientation 決定 SDF 的 <pose> 怎麼解讀。數值沿用 PX4 內建 world。 -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ===== 牆 ===== -->"""

    body = floor_texture_xml() + "".join(wall_model_xml(*w) for w in WALLS)

    footer = f"""
    <!-- ===== 降落標記 =====
         z 給 0.01 而不是 0：跟地面同高會發生 z-fighting（兩個共面的多邊形互相
         閃爍），相機會拍到雜訊。抬高 1 cm 對偵測沒有影響。 -->
    <include>
      <uri>model://{TAG_NAME}</uri>
      <name>landing_tag</name>
      <pose>{TAG_POS[0]} {TAG_POS[1]} 0.01 0 0 0</pose>
    </include>

  </world>
</sdf>
"""
    path = os.path.join(PKG_DIR, "gz", "worlds", "nav2_arena.sdf")
    with open(path, "w", encoding="utf-8") as f:
        f.write(header + body + footer)
    return path


def write_graph():
    features = []
    for nid, (x, y, desc) in NODES.items():
        props = {"id": nid, "frame": "map", "metadata": {"description": desc}}
        if nid == max(NODES):          # 最後一個節點就是降落點
            props["operations"] = LANDING_OPERATION
        features.append({"type": "Feature", "properties": props,
                         "geometry": {"type": "Point", "coordinates": [x, y]}})

    # 邊是有向的，雙向通行要寫兩筆。id 從 100 起跳，跟節點 id 一眼分得開。
    eid = 100
    for a, b, lane in LINKS:
        for s, e in ((a, b), (b, a)):
            sx, sy, _ = NODES[s]
            ex, ey, _ = NODES[e]
            pinched = (min(s, e), max(s, e)) in PINCHED_LINKS
            meta = {"lane": lane,
                    "speed_limit": PINCH_SPEED_LIMIT if pinched else 1.0}
            if pinched:
                meta["penalty"] = PINCH_PENALTY
            props = {"id": eid, "startid": s, "endid": e, "overridable": True,
                     "cost": round(math.hypot(ex - sx, ey - sy), 3), "metadata": meta}
            if pinched:
                # ON_ENTER：一進入束縮段就把速限發出去，而不是等飛到中間才減速
                props["operations"] = {"slow_in_pinch": {
                    "type": "slow_in_pinch", "trigger": "ON_ENTER",
                    "metadata": {"speed_limit": PINCH_SPEED_LIMIT}}}
            features.append({"type": "Feature", "properties": props,
                             "geometry": {"type": "MultiLineString",
                                          "coordinates": [[[sx, sy], [ex, ey]]]}})
            eid += 1

    path = os.path.join(PKG_DIR, "graphs", "nav2_arena.geojson")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"type": "FeatureCollection", "name": "nav2_arena",
                   "features": features}, f, indent=2, ensure_ascii=False)
        f.write("\n")
    return path, len(NODES), len(features) - len(NODES)


def dist_point_to_wall(px, py, cx, cy, sx, sy, yaw_deg):
    """點到旋轉矩形的最短距離。點在矩形內回傳 -1。

    做法：把點轉進矩形自己的座標系，問題就變成點到「軸對齊矩形」的距離，
    那個有現成的公式（各軸超出半邊長的量，取歐氏距離）。
    """
    a = math.radians(yaw_deg)
    dx, dy = px - cx, py - cy
    lx = dx * math.cos(a) + dy * math.sin(a)
    ly = -dx * math.sin(a) + dy * math.cos(a)
    if abs(lx) <= sx / 2 and abs(ly) <= sy / 2:
        return -1.0
    return math.hypot(max(abs(lx) - sx / 2, 0.0), max(abs(ly) - sy / 2, 0.0))


def report_clearance():
    """粗略檢查：每個節點離最近的牆多遠。

    這只是隨手看一眼，不是正式驗證 —— 邊有沒有穿牆、圖連不連通都沒檢查。
    完整的驗證是 scripts/check_graph.py（T1）。
    """
    print(f"\n{'節點':>4} {'座標':>16} {'離牆':>9}  最近的牆")
    worst = 999.0
    for nid, (x, y, _) in sorted(NODES.items()):
        d, name = min((dist_point_to_wall(x, y, w[1], w[2], w[3], w[4], w[5]), w[0])
                      for w in WALLS)
        flag = "  <-- 卡在牆裡!" if d < 0 else ("  <-- 太近" if d < 2.5 else "")
        print(f"{nid:>4} ({x:6.1f},{y:6.1f}) {d:8.2f} m  {name}{flag}")
        worst = min(worst, d)
    print(f"\n最小淨空 {worst:.2f} m（編隊半徑 2 m，所以要 > 2.5 m 才安全）")
    return worst


def write_preview():
    if os.environ.get("ARENA_WRITE_PREVIEW", "0").lower() not in ("1", "true", "yes", "on"):
        print("（跳過俯視預覽圖。若要產生，設定 ARENA_WRITE_PREVIEW=1）")
        return None
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        from matplotlib.transforms import Affine2D
    except Exception as e:
        print(f"（matplotlib 無法使用，跳過俯視圖：{e}）")
        return None

    fig, ax = plt.subplots(figsize=(12, 10))
    for name, cx, cy, sx, sy, yaw, _ in WALLS:
        r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor="#4a4a55", edgecolor="none")
        r.set_transform(Affine2D().rotate_deg_around(cx, cy, yaw) + ax.transData)
        ax.add_patch(r)

    for a, b, _ in LINKS:
        (x1, y1, _), (x2, y2, _) = NODES[a], NODES[b]
        slow = (min(a, b), max(a, b)) in PINCHED_LINKS
        ax.plot([x1, x2], [y1, y2], color="#d94f2b" if slow else "#1f77b4",
                lw=3.0 if slow else 2.0, ls="--" if slow else "-", zorder=3)

    for nid, (x, y, _) in NODES.items():
        ax.plot(x, y, "o", ms=15, color="#fff", mec="#1f77b4", mew=2.5, zorder=4)
        ax.text(x, y, str(nid), ha="center", va="center", fontsize=9,
                color="#1f77b4", fontweight="bold", zorder=5)

    ax.plot(*TAG_POS, "s", ms=13, color="k", zorder=6)
    ax.text(TAG_POS[0] + 1.5, TAG_POS[1], "AprilTag", fontsize=10, va="center")
    for dy in (0.0, 3.0, -3.0):
        ax.plot(NODES[0][0], NODES[0][1] + dy, "^", ms=11, color="#2ca02c", zorder=6)

    x0, x1, y0, y1 = bounds()
    ax.set_xlim(x0 - 3, x1 + 3)
    ax.set_ylim(y0 - 3, y1 + 3)
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.35)
    ax.set_xlabel("x  East [m]")
    ax.set_ylabel("y  North [m]")
    ax.set_title(f"nav2_arena  ({x1 - x0:.0f} x {y1 - y0:.0f} m)\n"
                 "solid blue = normal edge   dashed red = pinched north lane")
    fig.tight_layout()
    path = os.path.join(PKG_DIR, "docs", "arena_preview.png")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    return path


def main():
    x0, x1, y0, y1 = bounds()
    print(f"場地外框： {x1 - x0:.0f} x {y1 - y0:.0f} m"
          f"   x [{x0:.1f}, {x1:.1f}]  y [{y0:.1f}, {y1:.1f}]")
    print("寫出 " + write_world())
    path, n_nodes, n_edges = write_graph()
    print(f"寫出 {path}   （{n_nodes} 節點 / {n_edges} 有向邊）")
    preview = write_preview()
    if preview:
        print("寫出 " + preview)
    worst = report_clearance()
    return 0 if worst > 2.5 else 1


if __name__ == "__main__":
    sys.exit(main())
