#!/usr/bin/env python3
# =============================================================================
#  check_graph.py — 場地與拓樸圖的一致性檢查（T1）
#
#  用法：
#      python3 src/drone_nav2_apriltag/scripts/check_graph.py
#      python3 .../check_graph.py --clearance 3.0        # 改淨空門檻
#      python3 .../check_graph.py --world A.sdf --graph B.geojson
#
#  回傳值：全部通過 0，有任何一項失敗 1。可以直接串進 CI 或 pre-commit。
#
#  為什麼需要這支：
#      world（牆在哪）和 graph（節點在哪）是兩個獨立的檔案，格式還不一樣。
#      它們不一致的時候，Gazebo 照常開、route_server 照常規劃、什麼都不會報錯
#      —— 只會在飛的時候莫名其妙撞牆。這種靜默失敗只能靠主動比對抓出來。
#
#      注意這支是「讀檔案」的，不是讀 gen_arena.py 的變數。所以就算有人
#      手改了 .sdf 或 .geojson，它一樣抓得到 —— 那才是真正的風險來源。
#
#  它抓不到什麼（誠實說明）：
#      - 整張圖在 ROS 座標系裡偏移／鏡射 —— 檔案自己是自洽的，要靠 T2
#      - 無人機實際飛不飛得過去 —— 要靠 T3
# =============================================================================

import argparse
import math
import os
import sys
import json
import xml.etree.ElementTree as ET
from collections import deque

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 預設淨空門檻。依據：三角編隊外接圓半徑 2 m，加機身半徑約 0.25 m，
# 再留一點餘裕給 Nav2 costmap 的膨脹層。
DEFAULT_CLEARANCE = 2.5

# 柵格連通性檢查的解析度。0.25 m 讓 0.5 m 厚的牆至少佔滿兩格，不會被穿透。
GRID_RES = 0.25


# =============================================================================
#  幾何工具
# =============================================================================

class Box:
    """一面牆：軸對齊的矩形先繞自己的中心轉 yaw，再平移到 (cx, cy)。

    所有距離計算都先把查詢點轉進「牆自己的座標系」，問題就退化成
    點對軸對齊矩形，那個有現成的閉式解。這比直接處理旋轉矩形簡單得多。
    """

    def __init__(self, name, cx, cy, sx, sy, yaw):
        self.name = name
        self.cx, self.cy = cx, cy
        self.hx, self.hy = sx / 2.0, sy / 2.0
        self.yaw = yaw
        self._c, self._s = math.cos(yaw), math.sin(yaw)

    def to_local(self, px, py):
        dx, dy = px - self.cx, py - self.cy
        return (dx * self._c + dy * self._s, -dx * self._s + dy * self._c)

    def corners(self):
        out = []
        for lx, ly in ((self.hx, self.hy), (self.hx, -self.hy),
                       (-self.hx, -self.hy), (-self.hx, self.hy)):
            out.append((self.cx + lx * self._c - ly * self._s,
                        self.cy + lx * self._s + ly * self._c))
        return out


def point_aabb_dist(px, py, hx, hy):
    """點到「以原點為中心、半邊長 hx/hy」的矩形的距離。點在裡面回傳 0。"""
    return math.hypot(max(abs(px) - hx, 0.0), max(abs(py) - hy, 0.0))


def point_in_aabb(px, py, hx, hy):
    return abs(px) <= hx and abs(py) <= hy


def point_seg_dist(px, py, ax, ay, bx, by):
    """點到線段的距離。"""
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    if L2 == 0.0:
        return math.hypot(px - ax, py - ay)
    # 把點投影到線段上，t 夾在 [0,1] 之內才算落在線段而不是延長線上
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def seg_aabb_intersect(ax, ay, bx, by, hx, hy):
    """線段與軸對齊矩形有沒有相交（含線段完全在矩形內的情況）。

    用 slab method：把矩形看成 x 方向和 y 方向兩條「板」的交集，
    分別算線段進入／離開每條板的參數 t，兩個區間有交集就是相交。
    """
    if point_in_aabb(ax, ay, hx, hy) or point_in_aabb(bx, by, hx, hy):
        return True
    t0, t1 = 0.0, 1.0
    for p, q, h in ((ax, bx - ax, hx), (ay, by - ay, hy)):
        if abs(q) < 1e-12:
            # 線段在這個軸上沒有移動：只要起點就在板外，永遠不會相交
            if abs(p) > h:
                return False
            continue
        ta, tb = (-h - p) / q, (h - p) / q
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


def seg_box_dist(ax, ay, bx, by, box):
    """線段到牆的最短距離。相交回傳 0。

    兩個凸形不相交時，最短距離一定發生在「其中一個的頂點」上，
    所以只要檢查：線段兩端點對矩形、矩形四個角對線段，取最小值。
    """
    la = box.to_local(ax, ay)
    lb = box.to_local(bx, by)
    if seg_aabb_intersect(la[0], la[1], lb[0], lb[1], box.hx, box.hy):
        return 0.0
    d = min(point_aabb_dist(la[0], la[1], box.hx, box.hy),
            point_aabb_dist(lb[0], lb[1], box.hx, box.hy))
    for cx, cy in box.corners():
        d = min(d, point_seg_dist(cx, cy, ax, ay, bx, by))
    return d


def point_box_dist(px, py, box):
    lx, ly = box.to_local(px, py)
    return point_aabb_dist(lx, ly, box.hx, box.hy)


def point_in_box(px, py, box):
    lx, ly = box.to_local(px, py)
    return point_in_aabb(lx, ly, box.hx, box.hy)


# =============================================================================
#  讀檔
# =============================================================================

def parse_pose(text):
    """SDF 的 <pose> 是 'x y z roll pitch yaw'。這裡只用得到 x, y, yaw，
    但 roll/pitch 不是 0 的話代表牆是傾斜的，2D 投影就不成立，必須警告。"""
    v = [float(t) for t in text.split()]
    while len(v) < 6:
        v.append(0.0)
    return v


def parse_world(path):
    """從 .sdf 撈出所有方塊障礙物，以及 AprilTag 的位置。

    只認 <box> 幾何。ground_plane 用的是 <plane>（無限大平面），
    當成障礙物的話整張圖都會是牆，所以自然被排除掉。
    """
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{path} 裡找不到 <world>")

    boxes, warnings = [], []
    for model in world.findall("model"):
        name = model.get("name", "<無名>")
        size_el = model.find(".//collision/geometry/box/size")
        if size_el is None:
            continue                      # ground_plane 之類的非方塊，跳過
        sx, sy, _sz = [float(t) for t in size_el.text.split()]
        pose_el = model.find("pose")
        x, y, _z, roll, pitch, yaw = parse_pose(pose_el.text if pose_el is not None else "0 0 0 0 0 0")
        if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
            warnings.append(f"{name} 的 roll/pitch 不是 0，這支腳本的 2D 投影會失準")
        boxes.append(Box(name, x, y, sx, sy, yaw))

    tag = None
    for inc in world.findall("include"):
        name_el = inc.find("name")
        pose_el = inc.find("pose")
        if pose_el is None:
            continue
        p = parse_pose(pose_el.text)
        tag = (name_el.text if name_el is not None else "<無名>", p[0], p[1])
    return boxes, tag, warnings


def parse_graph(path):
    """從 .geojson 撈節點和邊。Point 是節點，MultiLineString 是邊。"""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)

    nodes, edges, problems = {}, [], []
    for i, feat in enumerate(data.get("features", [])):
        props = feat.get("properties", {})
        geom = feat.get("geometry", {})
        gtype = geom.get("type")
        fid = props.get("id")

        if gtype == "Point":
            if fid is None:
                problems.append(f"第 {i} 個 feature 是節點但沒有 id")
                continue
            if fid in nodes:
                problems.append(f"節點 id {fid} 重複")
            xy = geom.get("coordinates", [])
            nodes[fid] = (float(xy[0]), float(xy[1]))
        elif gtype == "MultiLineString":
            edges.append((props, geom.get("coordinates", [[]])[0]))
        else:
            problems.append(f"第 {i} 個 feature 的 geometry 是 {gtype}，"
                            f"nav2_route 只認得 Point 和 MultiLineString")
    return nodes, edges, problems


# =============================================================================
#  檢查項目
# =============================================================================

class Report:
    def __init__(self):
        self.failed = 0
        self.warned = 0

    def section(self, title):
        print(f"\n── {title} " + "─" * max(0, 58 - len(title)))

    def ok(self, msg):
        print(f"   ✓ {msg}")

    def warn(self, msg):
        print(f"   ! {msg}")
        self.warned += 1

    def fail(self, msg):
        print(f"   ✗ {msg}")
        self.failed += 1


def check_nodes(rep, nodes, boxes, clearance):
    rep.section("C1 節點位置")
    worst, worst_id, worst_wall = 1e9, None, None
    for nid, (x, y) in sorted(nodes.items()):
        inside = [b.name for b in boxes if point_in_box(x, y, b)]
        if inside:
            rep.fail(f"節點 {nid} ({x}, {y}) 卡在牆裡： {', '.join(inside)}")
            continue
        d, wall = min((point_box_dist(x, y, b), b.name) for b in boxes)
        if d < clearance:
            rep.fail(f"節點 {nid} ({x}, {y}) 離 {wall} 只有 {d:.3f} m，"
                     f"低於門檻 {clearance:.3f} m")
        if d < worst:
            worst, worst_id, worst_wall = d, nid, wall
    if rep.failed == 0:
        rep.ok(f"{len(nodes)} 個節點都不在牆裡，最小淨空 {worst:.2f} m"
               f"（節點 {worst_id} 對 {worst_wall}）")


def check_edges(rep, nodes, edges, boxes, clearance):
    rep.section("C2 邊有沒有穿牆")
    before = rep.failed
    worst, worst_id, worst_wall = 1e9, None, None
    for props, coords in edges:
        eid = props.get("id")
        s, e = props.get("startid"), props.get("endid")
        if s not in nodes or e not in nodes:
            continue                       # 這種錯由 C3 負責報
        ax, ay = nodes[s]
        bx, by = nodes[e]
        hit = [b.name for b in boxes if seg_box_dist(ax, ay, bx, by, b) == 0.0]
        if hit:
            rep.fail(f"邊 {eid} ({s}→{e}) 穿過： {', '.join(hit)}")
            continue
        d, wall = min((seg_box_dist(ax, ay, bx, by, b), b.name) for b in boxes)
        if d < clearance:
            rep.fail(f"邊 {eid} ({s}→{e}) 離 {wall} 只有 {d:.3f} m，"
                     f"低於門檻 {clearance:.3f} m")
        if d < worst:
            worst, worst_id, worst_wall = d, f"{s}→{e}", wall
    if rep.failed == before:
        rep.ok(f"{len(edges)} 條邊都沒穿牆，最小淨空 {worst:.2f} m"
               f"（{worst_id} 對 {worst_wall}）")


def check_structure(rep, nodes, edges):
    rep.section("C3 圖的結構")
    before = rep.failed
    seen_eid, seen_pair = set(), set()
    for props, coords in edges:
        eid = props.get("id")
        s, e = props.get("startid"), props.get("endid")

        if eid is None:
            rep.fail(f"有一條邊（{s}→{e}）沒有 id")
        elif eid in seen_eid:
            rep.fail(f"邊 id {eid} 重複")
        else:
            seen_eid.add(eid)

        if s not in nodes:
            rep.fail(f"邊 {eid} 的 startid {s} 不存在")
            continue
        if e not in nodes:
            rep.fail(f"邊 {eid} 的 endid {e} 不存在")
            continue
        if s == e:
            rep.fail(f"邊 {eid} 是自環（{s}→{s}）")
        if (s, e) in seen_pair:
            rep.fail(f"{s}→{e} 有重複的邊")
        seen_pair.add((s, e))

        # geometry 的座標必須跟 startid/endid 指到的節點一致。
        # 這兩者是分開存的，不一致的話 route_server 用 id、
        # 視覺化工具用 coordinates，兩邊就會看到不同的圖。
        if len(coords) >= 2:
            for (gx, gy), (nx, ny), which in ((coords[0], nodes[s], "起點"),
                                              (coords[-1], nodes[e], "終點")):
                if abs(gx - nx) > 1e-6 or abs(gy - ny) > 1e-6:
                    rep.fail(f"邊 {eid} 的 geometry {which} ({gx}, {gy}) "
                             f"跟節點座標 ({nx}, {ny}) 對不上")

    # 反向邊缺漏只是警告 —— 單行道是合法設計，但多半是忘了寫
    for s, e in sorted(seen_pair):
        if (e, s) not in seen_pair:
            rep.warn(f"{s}→{e} 沒有反向邊，這條路只能單向通行")

    if rep.failed == before:
        rep.ok(f"{len(nodes)} 節點 / {len(edges)} 邊，id 唯一、指向有效、座標一致")


def check_connectivity(rep, nodes, edges, start, goal):
    rep.section("C4 圖的連通性（沿著有向邊走）")
    before = rep.failed
    fwd, bwd = {n: [] for n in nodes}, {n: [] for n in nodes}
    for props, _ in edges:
        s, e = props.get("startid"), props.get("endid")
        if s in nodes and e in nodes:
            fwd[s].append(e)
            bwd[e].append(s)

    def reach(adj, src):
        seen, q = {src}, deque([src])
        while q:
            for nxt in adj[q.popleft()]:
                if nxt not in seen:
                    seen.add(nxt)
                    q.append(nxt)
        return seen

    out = reach(fwd, start)
    back = reach(bwd, start)

    if goal not in out:
        rep.fail(f"從節點 {start} 走不到節點 {goal} —— 這張圖規劃不出任務路線")
    unreachable = sorted(set(nodes) - out)
    if unreachable:
        rep.fail(f"從節點 {start} 到不了這些節點： {unreachable}")
    no_return = sorted(set(nodes) - back)
    if no_return:
        rep.fail(f"這些節點回不了節點 {start}（有去無回）： {no_return}")

    if rep.failed == before:
        rep.ok(f"節點 {start} ↔ 全部 {len(nodes)} 個節點雙向可達")


def check_space(rep, nodes, boxes, clearance, start):
    rep.section("C5 實體空間連通性（柵格 flood fill）")
    before = rep.failed

    xs, ys = [], []
    for b in boxes:
        for cx, cy in b.corners():
            xs.append(cx)
            ys.append(cy)
    x0, x1 = min(xs) - 1.0, max(xs) + 1.0
    y0, y1 = min(ys) - 1.0, max(ys) + 1.0
    nx = int((x1 - x0) / GRID_RES) + 1
    ny = int((y1 - y0) / GRID_RES) + 1

    # 為什麼要用「格子中心到牆的距離 < 半格對角線」而不是「中心在牆內」：
    # 0.5 m 厚的牆只佔兩格，斜著擺的時候中心點判定會漏格，柵格就破洞了。
    half_diag = GRID_RES * math.sqrt(2) / 2

    def build(margin):
        g = [[False] * nx for _ in range(ny)]
        for j in range(ny):
            cy = y0 + j * GRID_RES
            for i in range(nx):
                cx = x0 + i * GRID_RES
                for b in boxes:
                    if point_box_dist(cx, cy, b) < margin:
                        g[j][i] = True
                        break
        return g

    def cell(x, y):
        return (int(round((x - x0) / GRID_RES)), int(round((y - y0) / GRID_RES)))

    def flood(grid, src):
        si, sj = cell(*src)
        if grid[sj][si]:
            return None                    # 起點本身就被佔住
        seen = {(si, sj)}
        q = deque([(si, sj)])
        while q:
            i, j = q.popleft()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b2 = i + di, j + dj
                if 0 <= a < nx and 0 <= b2 < ny and (a, b2) not in seen and not grid[b2][a]:
                    seen.add((a, b2))
                    q.append((a, b2))
        return seen

    # 第一輪：只把牆本身當障礙，看看節點在不在同一個封閉空間裡。
    # 這抓的是「節點被放到場地外面」—— 那種節點淨空很好、圖上也連通，
    # 但實際上被牆隔開，無人機根本過不去。
    plain = build(half_diag)
    free = flood(plain, nodes[start])
    if free is None:
        rep.fail(f"節點 {start} 落在牆上，無法當作起點")
    else:
        outside = [n for n, p in sorted(nodes.items()) if cell(*p) not in free]
        if outside:
            rep.fail(f"這些節點跟節點 {start} 不在同一個封閉空間裡： {outside}")
        else:
            rep.ok(f"全部節點與節點 {start} 實體連通"
                   f"（柵格 {nx}×{ny} @ {GRID_RES} m）")

    # 第二輪：把牆膨脹 clearance 之後再走一次。
    # 這是在模擬 Nav2 的膨脹層 —— 幾何上通得過的縫，編隊不一定塞得進去。
    inflated = build(clearance)
    free2 = flood(inflated, nodes[start])
    if free2 is None:
        rep.fail(f"膨脹 {clearance} m 後節點 {start} 沒有可用空間")
    else:
        blocked = [n for n, p in sorted(nodes.items()) if cell(*p) not in free2]
        if blocked:
            rep.fail(f"牆膨脹 {clearance} m 後到不了這些節點： {blocked}"
                     f"（通道對編隊來說太窄）")
        else:
            rep.ok(f"牆膨脹 {clearance} m 後仍然全部連通 —— 編隊過得去")

    if rep.failed == before:
        pass


def check_tag(rep, nodes, tag, goal):
    rep.section("C6 AprilTag 位置")
    if tag is None:
        rep.warn("world 裡沒有 <include>，找不到 AprilTag")
        return
    name, tx, ty = tag
    gx, gy = nodes[goal]
    d = math.hypot(tx - gx, ty - gy)
    if d > 0.5:
        rep.fail(f"{name} 在 ({tx}, {ty})，但降落節點 {goal} 在 ({gx}, {gy})，"
                 f"差 {d:.2f} m —— 飛到節點也看不到標記")
    else:
        rep.ok(f"{name} ({tx}, {ty}) 與降落節點 {goal} 對齊（差 {d:.2f} m）")


# =============================================================================
#  主程式
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="檢查 world 與拓樸圖是否一致")
    ap.add_argument("--world", default=os.path.join(PKG_DIR, "gz", "worlds", "nav2_arena.sdf"))
    ap.add_argument("--graph", default=os.path.join(PKG_DIR, "graphs", "nav2_arena.geojson"))
    ap.add_argument("--clearance", type=float, default=DEFAULT_CLEARANCE,
                    help=f"淨空門檻（公尺），預設 {DEFAULT_CLEARANCE}"
                         "（三角編隊半徑 2 m + 機身 + 餘裕）")
    ap.add_argument("--start", type=int, default=None, help="起點節點 id，預設最小的")
    ap.add_argument("--goal", type=int, default=None, help="終點節點 id，預設最大的")
    args = ap.parse_args()

    print(f"world : {args.world}")
    print(f"graph : {args.graph}")
    print(f"淨空門檻 : {args.clearance} m")

    boxes, tag, world_warn = parse_world(args.world)
    nodes, edges, graph_problems = parse_graph(args.graph)

    rep = Report()
    if not nodes:
        rep.section("讀檔")
        rep.fail("geojson 裡一個節點都沒有")
        print("\n結果：失敗")
        return 1

    start = args.start if args.start is not None else min(nodes)
    goal = args.goal if args.goal is not None else max(nodes)
    print(f"起點 → 終點 : 節點 {start} → 節點 {goal}")
    print(f"牆 {len(boxes)} 面 / 節點 {len(nodes)} 個 / 邊 {len(edges)} 條")

    if world_warn or graph_problems:
        rep.section("C0 讀檔")
        for w in world_warn:
            rep.warn(w)
        for p in graph_problems:
            rep.fail(p)

    check_nodes(rep, nodes, boxes, args.clearance)
    check_edges(rep, nodes, edges, boxes, args.clearance)
    check_structure(rep, nodes, edges)
    check_connectivity(rep, nodes, edges, start, goal)
    check_space(rep, nodes, boxes, args.clearance, start)
    check_tag(rep, nodes, tag, goal)

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        return 1
    print("結果：全部通過"
          + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
