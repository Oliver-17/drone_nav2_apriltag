import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drone_nav2_apriltag")
    nav2_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_map = os.path.join(pkg_share, "maps", "empty_map.yaml")
    bringup_launch = os.path.join(nav2_share, "launch", "bringup_launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    px4_odom_topic = LaunchConfiguration("px4_odom_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    stamp_with_ros_time = LaunchConfiguration("stamp_with_ros_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("px4_odom_topic", default_value="/MAV1/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("stamp_with_ros_time", default_value="true"),
        Node(
            package="drone_nav2_apriltag",
            executable="px4_vehicle_odometry_to_odom.py",
            name="px4_odom_for_nav2",
            parameters=[{
                "input_topic": px4_odom_topic,
                "output_topic": odom_topic,
                "odom_frame_id": "odom",
                "child_frame_id": "base_link",
                "publish_tf": True,
                "stamp_with_ros_time": stamp_with_ros_time,
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "map": map_file,
                "slam": "False",
                "autostart": "True",
            }.items(),
        ),
    ])
