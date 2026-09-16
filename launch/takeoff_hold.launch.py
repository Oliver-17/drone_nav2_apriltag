from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument("target_system", default_value="1"),
        DeclareLaunchArgument("takeoff_altitude", default_value="3.0"),
        DeclareLaunchArgument("altitude_tolerance", default_value="0.3"),
        DeclareLaunchArgument("arm_timeout", default_value="30.0"),
        DeclareLaunchArgument("takeoff_timeout", default_value="30.0"),
        DeclareLaunchArgument("release_after_takeoff", default_value="true"),
        DeclareLaunchArgument("land_on_shutdown", default_value="false"),

        Node(
            package=PKG,
            executable="takeoff_hold.py",
            name="takeoff_hold",
            output="screen",
            parameters=[{
                "px4_namespace": LaunchConfiguration("px4_namespace"),
                "target_system": LaunchConfiguration("target_system"),
                "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                "altitude_tolerance": LaunchConfiguration("altitude_tolerance"),
                "arm_timeout": LaunchConfiguration("arm_timeout"),
                "takeoff_timeout": LaunchConfiguration("takeoff_timeout"),
                "release_after_takeoff": LaunchConfiguration("release_after_takeoff"),
                "land_on_shutdown": LaunchConfiguration("land_on_shutdown"),
            }],
        ),
    ])
