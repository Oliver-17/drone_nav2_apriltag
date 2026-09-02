#!/usr/bin/env python3
"""
cameras.launch.py - bridge x500_depth_nav2 cameras/depth/lidar from Gazebo to ROS 2.

Default topics match the x500_depth_nav2 model:
  front OakD-Lite: camera_link / IMX214 + StereoOV7251
  down  OakD-Lite: camera_down_link / IMX214_down + StereoOV7251_down
  lidar: lidar_link / lidar_2d
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gz_sensor_topic(world, model, link, sensor, leaf):
    return f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/{leaf}"


def as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes", "on")


def bridge_image(actions, name, gz_topic, ros_topic):
    actions.append(Node(
        package="ros_gz_image",
        executable="image_bridge",
        name=name,
        arguments=[gz_topic],
        remappings=[(gz_topic, ros_topic)],
        output="screen",
    ))


def bridge_camera_info(actions, name, gz_topic, ros_topic):
    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name,
        arguments=[f"{gz_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings=[(gz_topic, ros_topic)],
        output="screen",
    ))


def bridge_points(actions, name, gz_topic, ros_topic):
    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name,
        arguments=[f"{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        remappings=[(gz_topic, ros_topic)],
        output="screen",
    ))


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    drone_id = LaunchConfiguration("drone_id").perform(context)
    prefix = LaunchConfiguration("model_prefix").perform(context)
    view = as_bool(context, "view")
    lidar = as_bool(context, "lidar")
    depth = as_bool(context, "depth")
    points = as_bool(context, "points")

    # PX4 spawn names models as "${MODEL_NAME}_${px4_instance}".
    model = f"{prefix}_{drone_id}"

    actions = []
    rgb_image_topics = []

    rgb_cameras = [
        ("front_rgb", "camera_link", "IMX214", f"/{ns}/camera_front/rgb/image_raw", f"/{ns}/camera_front/rgb/camera_info"),
        ("down_rgb", "camera_down_link", "IMX214_down", f"/{ns}/camera_down/rgb/image_raw", f"/{ns}/camera_down/rgb/camera_info"),
    ]
    for name, link, sensor, ros_img, ros_info in rgb_cameras:
        gz_img = gz_sensor_topic(world, model, link, sensor, "image")
        gz_info = gz_sensor_topic(world, model, link, sensor, "camera_info")
        bridge_image(actions, f"image_bridge_{name}", gz_img, ros_img)
        bridge_camera_info(actions, f"info_bridge_{name}", gz_info, ros_info)
        rgb_image_topics.append(ros_img)

    if depth:
        depth_cameras = [
            ("front_depth", "camera_link", "StereoOV7251", f"/{ns}/camera_front/depth/image_raw", f"/{ns}/camera_front/depth/camera_info", f"/{ns}/camera_front/depth/points"),
            ("down_depth", "camera_down_link", "StereoOV7251_down", f"/{ns}/camera_down/depth/image_raw", f"/{ns}/camera_down/depth/camera_info", f"/{ns}/camera_down/depth/points"),
        ]
        for name, link, sensor, ros_depth, ros_info, ros_points in depth_cameras:
            gz_depth = gz_sensor_topic(world, model, link, sensor, "depth_image")
            gz_info = gz_sensor_topic(world, model, link, sensor, "camera_info")
            bridge_image(actions, f"depth_bridge_{name}", gz_depth, ros_depth)
            bridge_camera_info(actions, f"depth_info_bridge_{name}", gz_info, ros_info)
            if points:
                gz_points = gz_sensor_topic(world, model, link, sensor, "depth_image/points")
                bridge_points(actions, f"points_bridge_{name}", gz_points, ros_points)

    if lidar:
        gz_scan = gz_sensor_topic(world, model, "lidar_link", "lidar_2d", "scan")
        actions.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge",
            arguments=[f"{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            remappings=[(gz_scan, f"/{ns}/scan")],
            output="screen",
        ))

    # Static sensor mounts from x500_depth_nav2/model.sdf. Dynamic odom->base_link
    # still comes from PX4 odometry / SLAM odometry, not from this launch file.
    static_tfs = [
        ("camera_front_tf", ["0.12", "0.03", "0.002", "0", "0", "0", "base_link", "camera_link"]),
        ("camera_down_tf", ["0", "0", "-0.14", "0", "1.5707", "0", "base_link", "camera_down_link"]),
        ("lidar_tf", ["0", "0", "0.06", "0", "0", "0", "base_link", "lidar_link"]),
    ]
    for name, arguments in static_tfs:
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            arguments=arguments,
            output="screen",
        ))

    if view:
        for i, topic in enumerate(rgb_image_topics):
            actions.append(Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name=f"image_view_{i}",
                arguments=[topic],
                output="log",
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="nav2_arena",
                              description="Gazebo world name"),
        DeclareLaunchArgument("drone_id", default_value="0",
                              description="PX4 instance id: 0/1/2"),
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="ROS namespace prefix matching PX4_UXRCE_DDS_NS"),
        DeclareLaunchArgument("model_prefix", default_value="x500_depth_nav2",
                              description="Gazebo model name prefix / SIM_MODEL"),
        DeclareLaunchArgument("view", default_value="true",
                              description="Open rqt_image_view for RGB images"),
        DeclareLaunchArgument("lidar", default_value="true",
                              description="Bridge 2D lidar scan"),
        DeclareLaunchArgument("depth", default_value="true",
                              description="Bridge RGB-D depth image and depth camera_info"),
        DeclareLaunchArgument("points", default_value="true",
                              description="Bridge depth point clouds"),
        OpaqueFunction(function=launch_setup),
    ])
