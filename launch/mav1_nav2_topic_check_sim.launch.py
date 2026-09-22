from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    require_yaw_good = LaunchConfiguration("require_yaw_good")
    topic_timeout = LaunchConfiguration("topic_timeout")
    base_launch = PathJoinSubstitution([
        FindPackageShare("drone_nav2_apriltag"),
        "launch",
        "mav1_nav2_topic_check.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("require_yaw_good", default_value="false"),
        DeclareLaunchArgument("topic_timeout", default_value="2.5"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "use_sim_time": "true",
                "stamp_with_ros_time": "true",
                "require_mocap": "false",
                "require_yaw_good": require_yaw_good,
                "exit_when_ready": "false",
                "topic_timeout": topic_timeout,
            }.items(),
        ),
    ])
