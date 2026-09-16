# =============================================================================
#  slam_after_takeoff.launch.py -- Only start Cartographer after takeoff.
#
#  This launch does not start Gazebo, camera bridge, or flight control.
#  Manual step-by-step flow:
#      1. start_arena_sitl.sh
#      2. MicroXRCEAgent
#      3. cameras.launch.py
#      4. slam_after_takeoff.launch.py
#      5. fly_nodes.launch.py
#
#  Start this before takeoff so the current PX4 local NED z is recorded as the
#  initial ground altitude.
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_nav2_apriltag"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument(
            "start_altitude",
            default_value="2.5",
            description="Start SLAM after reaching this relative takeoff altitude",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_package", default_value=PKG),
        DeclareLaunchArgument("slam_launch_file", default_value="cartographer_3d_depth.launch.py"),
        DeclareLaunchArgument("depth_image_topic", default_value="/MAV1/camera_front/depth/image_raw"),
        DeclareLaunchArgument("depth_camera_info_topic", default_value="/MAV1/camera_front/depth/camera_info"),
        DeclareLaunchArgument("raw_points_topic", default_value="/cartographer/depth_points_raw"),
        DeclareLaunchArgument("points_topic", default_value="/cartographer/depth_points"),
        DeclareLaunchArgument("pointcloud_frame", default_value="camera_front_link"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("px4_imu_topic", default_value="/MAV1/fmu/out/sensor_combined"),
        DeclareLaunchArgument("imu_frame", default_value="base_link"),

        Node(
            package=PKG,
            executable="start_slam_when_airborne.py",
            name="start_slam_when_airborne",
            output="screen",
            parameters=[{
                "px4_namespace": LaunchConfiguration("px4_namespace"),
                "start_altitude": LaunchConfiguration("start_altitude"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "slam_package": LaunchConfiguration("slam_package"),
                "slam_launch_file": LaunchConfiguration("slam_launch_file"),
                "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                "depth_camera_info_topic": LaunchConfiguration("depth_camera_info_topic"),
                "raw_points_topic": LaunchConfiguration("raw_points_topic"),
                "points_topic": LaunchConfiguration("points_topic"),
                "pointcloud_frame": LaunchConfiguration("pointcloud_frame"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "px4_imu_topic": LaunchConfiguration("px4_imu_topic"),
                "imu_frame": LaunchConfiguration("imu_frame"),
            }],
        ),
    ])
