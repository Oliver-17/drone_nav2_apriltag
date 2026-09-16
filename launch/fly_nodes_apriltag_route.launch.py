# =============================================================================
# fly_nodes_apriltag_route.launch.py -- Manual mapping route for AprilTag end area
#
# Route: 1 -> 4 -> 6 -> 3 -> 1 -> 3 -> 10
# At node 10, do one slow in-place yaw scan. Landing is disabled.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    default_graph = os.path.join(share, "graphs", "nav2_arena.geojson")

    return LaunchDescription([
        DeclareLaunchArgument("graph", default_value=default_graph),
        DeclareLaunchArgument("flight_altitude", default_value="3.0"),
        DeclareLaunchArgument("arrival_radius", default_value="1.5"),
        DeclareLaunchArgument("cruise_speed", default_value="0.20"),
        DeclareLaunchArgument("turn_slow_speed", default_value="0.08"),
        DeclareLaunchArgument("turn_slow_radius", default_value="2.0"),
        DeclareLaunchArgument("turn_angle_threshold_deg", default_value="20.0"),
        DeclareLaunchArgument("turn_settle_time", default_value="1.5"),
        DeclareLaunchArgument("leg_timeout", default_value="180.0"),
        DeclareLaunchArgument("scan_yaw_speed_deg_s", default_value="18.0"),
        DeclareLaunchArgument("scan_yaw_turns", default_value="1.0"),
        DeclareLaunchArgument("scan_yaw_hold_time", default_value="0.5"),
        DeclareLaunchArgument("save_map_file", default_value="/home/zhg/ncrl_mqtt/maps/arena"),
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument("target_system", default_value="1"),

        Node(
            package=PKG,
            executable="fly_nodes.py",
            name="fly_nodes_apriltag_route",
            output="screen",
            parameters=[{
                "graph_file": LaunchConfiguration("graph"),
                "start_node": 1,
                "goal_node": 10,
                "use_route_server": False,
                "node_sequence": [1, 4, 6, 3, 1, 3, 10],
                "flight_altitude": LaunchConfiguration("flight_altitude"),
                "arrival_radius": LaunchConfiguration("arrival_radius"),
                "cruise_speed": LaunchConfiguration("cruise_speed"),
                "turn_slow_speed": LaunchConfiguration("turn_slow_speed"),
                "turn_slow_radius": LaunchConfiguration("turn_slow_radius"),
                "turn_angle_threshold_deg": LaunchConfiguration("turn_angle_threshold_deg"),
                "turn_settle_time": LaunchConfiguration("turn_settle_time"),
                "leg_timeout": LaunchConfiguration("leg_timeout"),
                "scan_yaw_nodes": [10],
                "scan_yaw_speed_deg_s": LaunchConfiguration("scan_yaw_speed_deg_s"),
                "scan_yaw_turns": LaunchConfiguration("scan_yaw_turns"),
                "scan_yaw_hold_time": LaunchConfiguration("scan_yaw_hold_time"),
                "save_map_nodes": [10],
                "save_map_file": LaunchConfiguration("save_map_file"),
                "px4_namespace": LaunchConfiguration("px4_namespace"),
                "target_system": LaunchConfiguration("target_system"),
                "land_at_goal": False,
            }],
        ),
    ])
