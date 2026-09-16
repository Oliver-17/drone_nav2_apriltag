#!/usr/bin/env python3
"""
cameras_depth_proc.launch.py

Gazebo -> ROS 2 camera bridge using depth_image_proc for front depth points.

This is the alternate "方案 B" pipeline:

  Gazebo depth_image + camera_info
        -> ros_gz_bridge
        -> ROS depth Image + CameraInfo
        -> depth_image_proc/point_cloud_xyz_node
        -> /MAV1/camera_front/depth/points

The generated point cloud keeps camera_front_optical_frame numeric axes.
TF connects it back to camera_front_link/base_link.

It intentionally does not bridge Gazebo's depth_image/points topic, so the heavy
PointCloudPacked -> PointCloud2 conversion does not happen inside ros_gz_bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gz_sensor_topic(world, model, link, sensor, leaf):
    return (
        f"/world/{world}"
        f"/model/{model}"
        f"/link/{link}"
        f"/sensor/{sensor}"
        f"/{leaf}"
    )


def as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in (
        "true",
        "1",
        "yes",
        "on",
    )


def bridge_image(actions, name, gz_topic, ros_topic):
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            arguments=[
                f"{gz_topic}"
                "@sensor_msgs/msg/Image"
                "[gz.msgs.Image"
            ],
            remappings=[
                (gz_topic, ros_topic),
            ],
            output="screen",
        )
    )


def bridge_camera_info(actions, name, gz_topic, ros_topic):
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            arguments=[
                f"{gz_topic}"
                "@sensor_msgs/msg/CameraInfo"
                "[gz.msgs.CameraInfo"
            ],
            remappings=[
                (gz_topic, ros_topic),
            ],
            output="screen",
        )
    )


def bridge_lidar(actions, name, gz_topic, ros_topic):
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            arguments=[
                f"{gz_topic}"
                "@sensor_msgs/msg/LaserScan"
                "[gz.msgs.LaserScan"
            ],
            remappings=[
                (gz_topic, ros_topic),
            ],
            output="screen",
        )
    )


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    drone_id = LaunchConfiguration("drone_id").perform(context)
    prefix = LaunchConfiguration("model_prefix").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    front_depth_enabled = as_bool(context, "front_depth")
    front_rgb_enabled = as_bool(context, "front_rgb")
    down_rgb_enabled = as_bool(context, "down_rgb")
    lidar_enabled = as_bool(context, "lidar")

    model = f"{prefix}_{drone_id}"
    actions = []

    gz_clock = f"/world/{world}/clock"
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=[
                f"{gz_clock}"
                "@rosgraph_msgs/msg/Clock"
                "[gz.msgs.Clock"
            ],
            remappings=[
                (gz_clock, "/clock"),
            ],
            output="screen",
        )
    )

    if front_depth_enabled:
        front_depth_image_gz = gz_sensor_topic(
            world,
            model,
            "camera_front_link",
            "StereoOV7251",
            "depth_image",
        )
        front_depth_info_gz = gz_sensor_topic(
            world,
            model,
            "camera_front_link",
            "StereoOV7251",
            "camera_info",
        )
        front_depth_image_ros = f"/{ns}/camera_front/depth/image_raw"
        front_depth_info_ros = f"/{ns}/camera_front/depth/camera_info"
        front_depth_points_ros = f"/{ns}/camera_front/depth/points"

        bridge_image(
            actions,
            "front_depth_image_bridge",
            front_depth_image_gz,
            front_depth_image_ros,
        )
        bridge_camera_info(
            actions,
            "front_depth_info_bridge",
            front_depth_info_gz,
            front_depth_info_ros,
        )
        actions.append(
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="front_depth_to_points",
                parameters=[{
                    "queue_size": 1,
                    "use_sim_time": use_sim_time,
                }],
                remappings=[
                    ("image_rect", front_depth_image_ros),
                    ("camera_info", front_depth_info_ros),
                    ("points", front_depth_points_ros),
                ],
                output="screen",
            )
        )

    if front_rgb_enabled:
        front_rgb_gz = gz_sensor_topic(
            world,
            model,
            "camera_front_link",
            "IMX214",
            "image",
        )
        front_rgb_info_gz = gz_sensor_topic(
            world,
            model,
            "camera_front_link",
            "IMX214",
            "camera_info",
        )
        bridge_image(
            actions,
            "front_rgb_bridge",
            front_rgb_gz,
            f"/{ns}/camera_front/rgb/image_raw",
        )
        bridge_camera_info(
            actions,
            "front_rgb_info_bridge",
            front_rgb_info_gz,
            f"/{ns}/camera_front/rgb/camera_info",
        )

    if down_rgb_enabled:
        down_rgb_gz = gz_sensor_topic(
            world,
            model,
            "camera_down_link",
            "IMX214_down",
            "image",
        )
        down_rgb_info_gz = gz_sensor_topic(
            world,
            model,
            "camera_down_link",
            "IMX214_down",
            "camera_info",
        )
        bridge_image(
            actions,
            "down_rgb_bridge",
            down_rgb_gz,
            f"/{ns}/camera_down/rgb/image_raw",
        )
        bridge_camera_info(
            actions,
            "down_rgb_info_bridge",
            down_rgb_info_gz,
            f"/{ns}/camera_down/rgb/camera_info",
        )

    if lidar_enabled:
        lidar_gz = gz_sensor_topic(
            world,
            model,
            "lidar_link",
            "lidar_2d",
            "scan",
        )
        bridge_lidar(
            actions,
            "lidar_bridge",
            lidar_gz,
            f"/{ns}/scan",
        )

    if front_depth_enabled or front_rgb_enabled:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_front_tf",
                arguments=[
                    "0.28",
                    "0.03",
                    "0.22",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "camera_front_link",
                ],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_front_optical_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "-0.5",
                    "0.5",
                    "-0.5",
                    "0.5",
                    "camera_front_link",
                    "camera_front_optical_frame",
                ],
                output="screen",
            )
        )

    if down_rgb_enabled:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_down_tf",
                arguments=[
                    "0",
                    "0",
                    "0.10",
                    "0",
                    "1.5707",
                    "0",
                    "base_link",
                    "camera_down_link",
                ],
                output="screen",
            )
        )

    if lidar_enabled:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_tf",
                arguments=[
                    "0",
                    "0",
                    "0.30",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "lidar_link",
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="nav2_arena",
            description="Gazebo world name",
        ),
        DeclareLaunchArgument(
            "drone_id",
            default_value="0",
            description="PX4 instance id: 0/1/2",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="MAV1",
            description="ROS namespace",
        ),
        DeclareLaunchArgument(
            "model_prefix",
            default_value="x500_depth_nav2",
            description="Gazebo model name prefix",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock for depth_image_proc",
        ),
        DeclareLaunchArgument(
            "front_depth",
            default_value="true",
            description="Bridge front depth image/info and create PointCloud2 in ROS",
        ),
        DeclareLaunchArgument(
            "front_rgb",
            default_value="false",
            description="Bridge front RGB image + CameraInfo for AprilTag",
        ),
        DeclareLaunchArgument(
            "down_rgb",
            default_value="false",
            description="Bridge down RGB image + CameraInfo for AprilTag",
        ),
        DeclareLaunchArgument(
            "lidar",
            default_value="false",
            description="Bridge 2D lidar for Nav2",
        ),
        OpaqueFunction(function=launch_setup),
    ])
