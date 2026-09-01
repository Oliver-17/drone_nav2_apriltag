# =============================================================================
#  fly_nodes.launch.py — T3b：照拓樸圖飛節點
#
#  這個 launch 檔會起三個東西：
#      1. route_server          讀 geojson，回答「該走哪些節點」
#      2. lifecycle_manager     幫 route_server 做 configure + activate
#      3. fly_nodes             問路線、然後用 offboard 位置控制飛
#
#  前置條件（這個 launch 檔不會幫你開）：
#      終端 1： ./scripts/start_arena_sitl.sh
#      終端 2： MicroXRCEAgent udp4 -p 8888
#
#  用法：
#      ros2 launch drone_nav2_apriltag fly_nodes.launch.py
#      ros2 launch drone_nav2_apriltag fly_nodes.launch.py flight_altitude:=4.0
#      ros2 launch drone_nav2_apriltag fly_nodes.launch.py goal_node:=7
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    default_graph = os.path.join(share, "graphs", "nav2_arena.geojson")

    args = [
        DeclareLaunchArgument("graph", default_value=default_graph,
                              description="拓樸圖檔（route_server 和 fly_nodes 讀的是同一份）"),
        DeclareLaunchArgument("start_node", default_value="-1",
                              description="起點節點 id，-1 表示用最小的"),
        DeclareLaunchArgument("goal_node", default_value="-1",
                              description="終點節點 id，-1 表示用最大的"),
        DeclareLaunchArgument("flight_altitude", default_value="3.0"),
        DeclareLaunchArgument("arrival_radius", default_value="1.0"),
        DeclareLaunchArgument("px4_namespace", default_value="",
                              description="多機時填 /MAV1 這種"),
        DeclareLaunchArgument("target_system", default_value="1",
                              description="多機時是 instance+1"),
        DeclareLaunchArgument("use_route_server", default_value="true",
                              description="false 就改用 node_sequence 手動指定順序（T3a）"),
    ]

    graph = LaunchConfiguration("graph")

    route_server = Node(
        package="nav2_route",
        executable="route_server",
        name="route_server",
        output="screen",
        parameters=[{
            "graph_filepath": graph,
            # 這兩個是 route_server 查 TF 用的 frame 名稱。我們只用節點 id
            # 查詢（use_poses=false），所以不會真的去查 TF，填預設值即可。
            "route_frame": "map",
            "base_frame": "base_link",
        }],
    )

    # route_server 是 lifecycle node，開起來之後停在 unconfigured，
    # 要有人叫它 configure + activate 才會開始服務。
    # autostart=true 就是叫 lifecycle_manager 自動做完這兩步。
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_route",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["route_server"],
            # 不用模擬時間：這裡沒有 /clock 發布者，設 true 會讓計時器永遠不觸發
            "use_sim_time": False,
        }],
    )

    fly_nodes = Node(
        package=PKG,
        executable="fly_nodes.py",
        name="fly_nodes",
        output="screen",
        parameters=[{
            "graph_file": graph,
            "start_node": LaunchConfiguration("start_node"),
            "goal_node": LaunchConfiguration("goal_node"),
            "flight_altitude": LaunchConfiguration("flight_altitude"),
            "arrival_radius": LaunchConfiguration("arrival_radius"),
            "px4_namespace": LaunchConfiguration("px4_namespace"),
            "target_system": LaunchConfiguration("target_system"),
            "use_route_server": LaunchConfiguration("use_route_server"),
        }],
    )

    # fly_nodes 飛完會自己關閉，但 route_server 和 lifecycle_manager 是常駐的，
    # 不主動收掉的話整個 launch 會一直掛著，要手動 Ctrl+C。
    # 任務結束＝fly_nodes 退出，所以綁在它的 exit 事件上。
    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(target_action=fly_nodes,
                      on_exit=[Shutdown(reason="fly_nodes 已結束，關閉整組")]))

    return LaunchDescription(
        args + [route_server, lifecycle_manager, fly_nodes, shutdown_when_done])
