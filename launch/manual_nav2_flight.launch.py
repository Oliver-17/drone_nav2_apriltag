import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drone_nav2_apriltag")
    nav2_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_map = os.path.join(pkg_share, "maps", "empty_map.yaml")
    default_rviz_config = os.path.join(pkg_share, "rviz", "nav2_empty_local.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    map_file = LaunchConfiguration("map")
    px4_namespace = LaunchConfiguration("px4_namespace")
    target_system = LaunchConfiguration("target_system")
    takeoff_altitude = LaunchConfiguration("takeoff_altitude")
    altitude_tolerance = LaunchConfiguration("altitude_tolerance")
    px4_odom_topic = LaunchConfiguration("px4_odom_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    manual_goal_topic = LaunchConfiguration("manual_goal_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument("target_system", default_value="1"),
        DeclareLaunchArgument("takeoff_altitude", default_value="0.5"),
        DeclareLaunchArgument("altitude_tolerance", default_value="0.1"),
        DeclareLaunchArgument("max_xy_speed", default_value="0.7"),
        DeclareLaunchArgument("land_after_goal", default_value="true"),
        DeclareLaunchArgument("land_on_abort", default_value="true"),
        DeclareLaunchArgument("px4_odom_topic", default_value="/MAV1/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("manual_goal_topic", default_value="/manual_nav2_start_goal"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
            output="screen",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),

        Node(
            package="drone_nav2_apriltag",
            executable="px4_vehicle_odometry_to_odom.py",
            name="px4_odom_for_manual_nav2",
            parameters=[{
                "input_topic": px4_odom_topic,
                "output_topic": odom_topic,
                "odom_frame_id": "odom",
                "child_frame_id": "base_link",
                "publish_tf": True,
                "stamp_with_ros_time": True,
                "use_sim_time": False,
            }],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "map": map_file,
                "slam": "False",
                "autostart": "True",
            }.items(),
        ),

        Node(
            package="drone_nav2_apriltag",
            executable="takeoff_manual_nav2.py",
            name="takeoff_manual_nav2",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "px4_namespace": px4_namespace,
                "target_system": target_system,
                "takeoff_altitude": takeoff_altitude,
                "altitude_tolerance": altitude_tolerance,
                "max_xy_speed": LaunchConfiguration("max_xy_speed"),
                "odom_topic": odom_topic,
                "cmd_vel_topic": "/cmd_vel",
                "goal_action": "/navigate_to_pose",
                "manual_goal_topic": manual_goal_topic,
                "require_valid_local_position": True,
                "land_after_goal": LaunchConfiguration("land_after_goal"),
                "land_on_abort": LaunchConfiguration("land_on_abort"),
                "use_sim_time": False,
            }],
        ),
    ])
