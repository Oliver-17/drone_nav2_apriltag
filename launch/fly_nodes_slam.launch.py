# =============================================================================
#  fly_nodes_slam.launch.py -- Fly a route, then start Cartographer after takeoff.
#
#  Preconditions:
#      Terminal 1: DRONES=1 ./scripts/start_arena_sitl.sh
#      Terminal 2: MicroXRCEAgent udp4 -p 8888
#
#  Usage:
#      ros2 launch drone_nav2_apriltag fly_nodes_slam.launch.py
#      ros2 launch drone_nav2_apriltag fly_nodes_slam.launch.py start_altitude:=2.5
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    default_graph = os.path.join(share, "graphs", "nav2_arena.geojson")
    fly_launch = os.path.join(share, "launch", "fly_nodes.launch.py")
    cameras_launch = os.path.join(share, "launch", "cameras.launch.py")

    args = [
        DeclareLaunchArgument("graph", default_value=default_graph),
        DeclareLaunchArgument("start_node", default_value="-1"),
        DeclareLaunchArgument("goal_node", default_value="-1"),
        DeclareLaunchArgument("flight_altitude", default_value="3.0"),
        DeclareLaunchArgument("arrival_radius", default_value="1.0"),
        DeclareLaunchArgument("cruise_speed", default_value="0.3"),
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument("target_system", default_value="1"),
        DeclareLaunchArgument("use_route_server", default_value="true"),
        DeclareLaunchArgument(
            "start_altitude",
            default_value="2.5",
            description="Start SLAM after reaching this relative takeoff altitude",
        ),
        DeclareLaunchArgument("world", default_value="nav2_arena"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("namespace", default_value="MAV1"),
        DeclareLaunchArgument("model_prefix", default_value="x500_depth_nav2"),
        DeclareLaunchArgument("view", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("depth", default_value="true"),
        DeclareLaunchArgument("points", default_value="true"),
        DeclareLaunchArgument("imu", default_value="false"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument(
            "gz_imu_topic",
            default_value="/world/nav2_arena/model/x500_depth_nav2_0/link/base_link/sensor/imu_sensor/imu",
        ),
        DeclareLaunchArgument("px4_imu_topic", default_value="/MAV1/fmu/out/sensor_combined"),
        DeclareLaunchArgument("px4_odom_topic", default_value="/MAV1/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("imu_frame", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_package", default_value=PKG),
        DeclareLaunchArgument("slam_launch_file", default_value="cartographer_3d_depth.launch.py"),
        DeclareLaunchArgument("raw_points_topic", default_value="/MAV1/camera_front/depth/points"),
    ]

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cameras_launch),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "drone_id": LaunchConfiguration("drone_id"),
            "namespace": LaunchConfiguration("namespace"),
            "model_prefix": LaunchConfiguration("model_prefix"),
            "view": LaunchConfiguration("view"),
            "lidar": LaunchConfiguration("lidar"),
            "depth": LaunchConfiguration("depth"),
            "points": LaunchConfiguration("points"),
            "imu": LaunchConfiguration("imu"),
            "imu_topic": LaunchConfiguration("imu_topic"),
        }.items(),
    )

    fly_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fly_launch),
        launch_arguments={
            "graph": LaunchConfiguration("graph"),
            "start_node": LaunchConfiguration("start_node"),
            "goal_node": LaunchConfiguration("goal_node"),
            "flight_altitude": LaunchConfiguration("flight_altitude"),
            "arrival_radius": LaunchConfiguration("arrival_radius"),
            "cruise_speed": LaunchConfiguration("cruise_speed"),
            "px4_namespace": LaunchConfiguration("px4_namespace"),
            "target_system": LaunchConfiguration("target_system"),
            "use_route_server": LaunchConfiguration("use_route_server"),
        }.items(),
    )

    start_slam = Node(
        package=PKG,
        executable="start_slam_when_airborne.py",
        name="start_slam_when_airborne",
        output="screen",
        parameters=[{
            "px4_namespace": LaunchConfiguration("px4_namespace"),
            "start_altitude": LaunchConfiguration("start_altitude"),
            "slam_package": LaunchConfiguration("slam_package"),
            "slam_launch_file": LaunchConfiguration("slam_launch_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "raw_points_topic": LaunchConfiguration("raw_points_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "gz_imu_topic": LaunchConfiguration("gz_imu_topic"),
            "px4_imu_topic": LaunchConfiguration("px4_imu_topic"),
            "px4_odom_topic": LaunchConfiguration("px4_odom_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "imu_frame": LaunchConfiguration("imu_frame"),
        }],
    )

    return LaunchDescription(args + [cameras, start_slam, fly_nodes])
