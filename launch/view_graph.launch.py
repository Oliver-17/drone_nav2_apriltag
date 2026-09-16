# =============================================================================
#  view_graph.launch.py — T2：在 RViz 裡看場地與拓樸圖
#
#  起三個東西：
#      1. graph_markers            把 .sdf 的牆和 .geojson 的圖發成 MarkerArray
#      2. static_transform_publisher   map -> arena（單位轉換，見下面說明）
#      3. rviz2                    載入 rviz/arena.rviz
#
#  用法：
#      ros2 launch drone_nav2_apriltag view_graph.launch.py
#      ros2 launch drone_nav2_apriltag view_graph.launch.py rviz:=false   # 只發 marker
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    share = get_package_share_directory(PKG)

    args = [
        DeclareLaunchArgument("world",
                              default_value=os.path.join(share, "gz", "worlds",
                                                         "nav2_arena.sdf")),
        DeclareLaunchArgument("graph",
                              default_value=os.path.join(share, "graphs",
                                                         "nav2_arena.geojson")),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1",
                              description="多機時填 /MAV1 這種；要看無人機就一定要填"),
        DeclareLaunchArgument("track_vehicle", default_value="true",
                              description="false = 只畫地圖，不訂閱 PX4"),
        DeclareLaunchArgument("rviz_config",
                              default_value=os.path.join(share, "rviz", "arena.rviz")),
    ]

    markers = Node(
        package=PKG,
        executable="graph_markers.py",
        name="graph_markers",
        output="screen",
        parameters=[{
            "world_file": LaunchConfiguration("world"),
            "graph_file": LaunchConfiguration("graph"),
            "frame_id": LaunchConfiguration("frame_id"),
            "px4_namespace": LaunchConfiguration("px4_namespace"),
            "track_vehicle": LaunchConfiguration("track_vehicle"),
        }],
    )

    # RViz 的 Fixed Frame 必須是「TF 樹裡真的存在」的 frame，否則畫面全空
    # 而且只會顯示一行 Fixed Frame [map] does not exist。
    # 現在還沒有人發布 map（那是階段 1 的 px4_tf_node 要做的），
    # 所以這裡發一個 map -> arena 的單位轉換，純粹讓 map 這個 frame 存在。
    # 階段 1 上線之後這行就可以拿掉 —— 到時候 map 會有真正的內容，
    # 無人機的位置也會出現在同一張圖上，那才是 T2 真正要驗的東西。
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arena_frame",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "arena"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(args + [markers, static_tf, rviz])
