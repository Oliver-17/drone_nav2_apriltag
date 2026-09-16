# =============================================================================
#  scan_map.launch.py -- 起飛並用 lawnmower path 掃 RTAB-Map 地圖
#
#  前置條件：
#      終端 1：DRONES=1 ./scripts/start_arena_sitl.sh
#      終端 2：MicroXRCEAgent udp4 -p 8888
#      終端 3：ros2 launch drone_nav2_apriltag cameras.launch.py
#      終端 4：ros2 launch drone_nav2_apriltag rtabmap_arena.launch.py
#
#  用法：
#      ros2 launch drone_nav2_apriltag scan_map.launch.py
#      ros2 launch drone_nav2_apriltag scan_map.launch.py flight_altitude:=4.0 scan_length:=25.0
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1",
                              description="PX4 ROS namespace，例如 /MAV1"),
        DeclareLaunchArgument("target_system", default_value="1",
                              description="PX4 MAV_SYS_ID，單機通常是 1"),
        DeclareLaunchArgument("flight_altitude", default_value="3.0",
                              description="掃圖高度，PX4 local NED 會用 -altitude"),
        DeclareLaunchArgument("scan_length", default_value="22.0",
                              description="掃描長度，ENU 東向，相對起飛點，單位 m"),
        DeclareLaunchArgument("scan_width", default_value="10.0",
                              description="掃描寬度，ENU 北向，相對起飛點，單位 m"),
        DeclareLaunchArgument("lane_spacing", default_value="2.5",
                              description="掃描排距，單位 m"),
        DeclareLaunchArgument("arrival_radius", default_value="0.8"),
        DeclareLaunchArgument("hold_time", default_value="1.0",
                              description="每個掃描點停留秒數"),
        DeclareLaunchArgument("leg_timeout", default_value="60.0"),
        DeclareLaunchArgument("face_travel_direction", default_value="true",
                              description="true 時機頭朝向下一個掃描點"),
        DeclareLaunchArgument("land_at_end", default_value="false",
                              description="true 掃完自動降落；false 保持最後位置"),
        DeclareLaunchArgument("start_east", default_value="0.0",
                              description="掃描起點相對起飛點的東向偏移，單位 m"),
        DeclareLaunchArgument("start_north", default_value="0.0",
                              description="掃描中心線相對起飛點的北向偏移，單位 m"),
    ]

    scan_map = Node(
        package=PKG,
        executable="scan_map.py",
        name="scan_map",
        output="screen",
        parameters=[{
            "px4_namespace": LaunchConfiguration("px4_namespace"),
            "target_system": LaunchConfiguration("target_system"),
            "flight_altitude": LaunchConfiguration("flight_altitude"),
            "scan_length": LaunchConfiguration("scan_length"),
            "scan_width": LaunchConfiguration("scan_width"),
            "lane_spacing": LaunchConfiguration("lane_spacing"),
            "arrival_radius": LaunchConfiguration("arrival_radius"),
            "hold_time": LaunchConfiguration("hold_time"),
            "leg_timeout": LaunchConfiguration("leg_timeout"),
            "face_travel_direction": LaunchConfiguration("face_travel_direction"),
            "land_at_end": LaunchConfiguration("land_at_end"),
            "start_east": LaunchConfiguration("start_east"),
            "start_north": LaunchConfiguration("start_north"),
        }],
    )

    return LaunchDescription(args + [scan_map])
