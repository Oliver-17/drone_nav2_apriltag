from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument("target_system", default_value="1"),
        DeclareLaunchArgument("fixed_altitude", default_value="2.0"),
        DeclareLaunchArgument("max_xy_speed", default_value="0.7"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        Node(
            package="drone_nav2_apriltag",
            executable="cmd_vel_to_px4.py",
            name="cmd_vel_to_px4",
            output="screen",
            parameters=[{
                "px4_namespace": LaunchConfiguration("px4_namespace"),
                "target_system": LaunchConfiguration("target_system"),
                "fixed_altitude": LaunchConfiguration("fixed_altitude"),
                "max_xy_speed": LaunchConfiguration("max_xy_speed"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }],
        ),
    ])
