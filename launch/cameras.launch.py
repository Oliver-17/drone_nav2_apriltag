# #!/usr/bin/env python3
# """
# cameras.launch.py

# Minimal Gazebo -> ROS 2 sensor bridge for x500_depth_nav2.

# Used by the current navigation stack:

#   Front RGB:
#     -> AprilTag front detector

#   Down RGB:
#     -> AprilTag down detector

#   Front depth PointCloud:
#     -> Cartographer 3D SLAM

#   2D lidar:
#     -> Nav2 local costmap / obstacle detection

#   Gazebo clock:
#     -> ROS 2 simulation time

# Unused streams are intentionally NOT bridged:
#   - front depth image
#   - down depth image
#   - front depth camera_info
#   - down depth camera_info
#   - down depth PointCloud
#   - Gazebo IMU
#   - rqt_image_view
# """

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def gz_sensor_topic(world, model, link, sensor, leaf):
#     return (
#         f"/world/{world}"
#         f"/model/{model}"
#         f"/link/{link}"
#         f"/sensor/{sensor}"
#         f"/{leaf}"
#     )


# def as_bool(context, name):
#     return (
#         LaunchConfiguration(name)
#         .perform(context)
#         .lower()
#         in ("true", "1", "yes", "on")
#     )


# def bridge_image(actions, name, gz_topic, ros_topic):
#     """
#     Bridge Gazebo Image directly to sensor_msgs/Image.

#     Use ros_gz_bridge instead of ros_gz_image so we only publish
#     the raw ROS Image required by AprilTag, without loading
#     image_transport / ffmpeg H.264 publishers.
#     """
#     actions.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             name=name,
#             arguments=[
#                 f"{gz_topic}"
#                 "@sensor_msgs/msg/Image"
#                 "[gz.msgs.Image"
#             ],
#             remappings=[
#                 (gz_topic, ros_topic)
#             ],
#             output="screen",
#         )
#     )


# def bridge_camera_info(actions, name, gz_topic, ros_topic):
#     actions.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             name=name,
#             arguments=[
#                 f"{gz_topic}"
#                 "@sensor_msgs/msg/CameraInfo"
#                 "[gz.msgs.CameraInfo"
#             ],
#             remappings=[
#                 (gz_topic, ros_topic)
#             ],
#             output="screen",
#         )
#     )


# def bridge_points(actions, name, gz_topic, ros_topic):
#     actions.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             name=name,
#             arguments=[
#                 f"{gz_topic}"
#                 "@sensor_msgs/msg/PointCloud2"
#                 "[gz.msgs.PointCloudPacked"
#             ],
#             remappings=[
#                 (gz_topic, ros_topic)
#             ],
#             output="screen",
#         )
#     )


# def bridge_lidar(actions, name, gz_topic, ros_topic):
#     actions.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             name=name,
#             arguments=[
#                 f"{gz_topic}"
#                 "@sensor_msgs/msg/LaserScan"
#                 "[gz.msgs.LaserScan"
#             ],
#             remappings=[
#                 (gz_topic, ros_topic)
#             ],
#             output="screen",
#         )
#     )


# def launch_setup(context, *args, **kwargs):

#     world = LaunchConfiguration("world").perform(context)
#     ns = LaunchConfiguration("namespace").perform(context)
#     drone_id = LaunchConfiguration("drone_id").perform(context)
#     prefix = LaunchConfiguration("model_prefix").perform(context)

#     lidar_enabled = as_bool(context, "lidar")

#     # PX4 SITL model name:
#     # x500_depth_nav2_0
#     # x500_depth_nav2_1
#     # ...
#     model = f"{prefix}_{drone_id}"

#     actions = []

#     # ============================================================
#     # Gazebo clock
#     # ============================================================

#     gz_clock = f"/world/{world}/clock"

#     actions.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             name="clock_bridge",
#             arguments=[
#                 f"{gz_clock}"
#                 "@rosgraph_msgs/msg/Clock"
#                 "[gz.msgs.Clock"
#             ],
#             remappings=[
#                 (gz_clock, "/clock")
#             ],
#             output="screen",
#         )
#     )

#     # ============================================================
#     # Front RGB camera
#     #
#     # Used by:
#     #   AprilTag front detector
#     # ============================================================

#     front_rgb_gz = gz_sensor_topic(
#         world,
#         model,
#         "camera_front_link",
#         "IMX214",
#         "image",
#     )

#     front_rgb_info_gz = gz_sensor_topic(
#         world,
#         model,
#         "camera_front_link",
#         "IMX214",
#         "camera_info",
#     )

#     bridge_image(
#         actions,
#         "front_rgb_bridge",
#         front_rgb_gz,
#         f"/{ns}/camera_front/rgb/image_raw",
#     )

#     bridge_camera_info(
#         actions,
#         "front_rgb_info_bridge",
#         front_rgb_info_gz,
#         f"/{ns}/camera_front/rgb/camera_info",
#     )

#     # ============================================================
#     # Down RGB camera
#     #
#     # Used by:
#     #   AprilTag down detector
#     # ============================================================

#     down_rgb_gz = gz_sensor_topic(
#         world,
#         model,
#         "camera_down_link",
#         "IMX214_down",
#         "image",
#     )

#     down_rgb_info_gz = gz_sensor_topic(
#         world,
#         model,
#         "camera_down_link",
#         "IMX214_down",
#         "camera_info",
#     )

#     bridge_image(
#         actions,
#         "down_rgb_bridge",
#         down_rgb_gz,
#         f"/{ns}/camera_down/rgb/image_raw",
#     )

#     bridge_camera_info(
#         actions,
#         "down_rgb_info_bridge",
#         down_rgb_info_gz,
#         f"/{ns}/camera_down/rgb/camera_info",
#     )

#     # ============================================================
#     # Front depth PointCloud
#     #
#     # Used by:
#     #   Cartographer 3D
#     #
#     # We intentionally do NOT bridge:
#     #   front depth image
#     #   front depth camera_info
#     # ============================================================

#     front_points_gz = gz_sensor_topic(
#         world,
#         model,
#         "camera_front_link",
#         "StereoOV7251",
#         "depth_image/points",
#     )

#     bridge_points(
#         actions,
#         "front_depth_points_bridge",
#         front_points_gz,
#         f"/{ns}/camera_front/depth/points",
#     )

#     # ============================================================
#     # 2D lidar
#     #
#     # Used later by:
#     #   Nav2 local costmap / obstacle layer
#     # ============================================================

#     if lidar_enabled:

#         lidar_gz = gz_sensor_topic(
#             world,
#             model,
#             "lidar_link",
#             "lidar_2d",
#             "scan",
#         )

#         bridge_lidar(
#             actions,
#             "lidar_bridge",
#             lidar_gz,
#             f"/{ns}/scan",
#         )

#     # ============================================================
#     # Static sensor transforms
#     #
#     # Dynamic map -> base_link is still owned by Cartographer.
#     # PX4 odometry remains a ROS message input, not a TF publisher.
#     # ============================================================

#     static_tfs = [
#         (
#             "camera_front_tf",
#             [
#                 "0.28",
#                 "0.03",
#                 "0.22",
#                 "0",
#                 "0",
#                 "0",
#                 "base_link",
#                 "camera_front_link",
#             ],
#         ),
#         (
#             "camera_down_tf",
#             [
#                 "0",
#                 "0",
#                 "0.10",
#                 "0",
#                 "1.5707",
#                 "0",
#                 "base_link",
#                 "camera_down_link",
#             ],
#         ),
#         (
#             "lidar_tf",
#             [
#                 "0",
#                 "0",
#                 "0.30",
#                 "0",
#                 "0",
#                 "0",
#                 "base_link",
#                 "lidar_link",
#             ],
#         ),
#     ]

#     for name, arguments in static_tfs:

#         # Don't publish lidar TF if lidar is disabled.
#         if name == "lidar_tf" and not lidar_enabled:
#             continue

#         actions.append(
#             Node(
#                 package="tf2_ros",
#                 executable="static_transform_publisher",
#                 name=name,
#                 arguments=arguments,
#                 output="screen",
#             )
#         )

#     return actions


# def generate_launch_description():

#     return LaunchDescription([

#         DeclareLaunchArgument(
#             "world",
#             default_value="nav2_arena",
#             description="Gazebo world name",
#         ),

#         DeclareLaunchArgument(
#             "drone_id",
#             default_value="0",
#             description="PX4 instance id: 0/1/2",
#         ),

#         DeclareLaunchArgument(
#             "namespace",
#             default_value="MAV1",
#             description=(
#                 "ROS namespace matching "
#                 "PX4_UXRCE_DDS_NS"
#             ),
#         ),

#         DeclareLaunchArgument(
#             "model_prefix",
#             default_value="x500_depth_nav2",
#             description=(
#                 "Gazebo model name prefix / SIM_MODEL"
#             ),
#         ),

#         DeclareLaunchArgument(
#             "lidar",
#             default_value="true",
#             description=(
#                 "Bridge 2D lidar for Nav2."
#             ),
#         ),

#         OpaqueFunction(
#             function=launch_setup
#         ),
#     ])

#!/usr/bin/env python3
"""
cameras.launch.py

Gazebo -> ROS 2 sensor bridge for x500_depth_nav2.

Debug / production-friendly version.

Each sensor stream can be enabled independently:

  front_points:
    Front depth PointCloud -> Cartographer

  front_rgb:
    Front RGB + CameraInfo -> AprilTag front

  down_rgb:
    Down RGB + CameraInfo -> AprilTag down

  lidar:
    2D lidar -> Nav2

Default DEBUG configuration:
  front_points = true
  front_rgb    = false
  down_rgb     = false
  lidar        = false

This lets us test the PointCloud bridge in isolation first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ============================================================
# Helpers
# ============================================================

def gz_sensor_topic(world, model, link, sensor, leaf):
    return (
        f"/world/{world}"
        f"/model/{model}"
        f"/link/{link}"
        f"/sensor/{sensor}"
        f"/{leaf}"
    )


def as_bool(context, name):
    return (
        LaunchConfiguration(name)
        .perform(context)
        .lower()
        in ("true", "1", "yes", "on")
    )


# ============================================================
# Bridge helpers
# ============================================================

def bridge_image(actions, name, gz_topic, ros_topic):
    """
    Gazebo Image -> ROS sensor_msgs/Image

    Direct ros_gz_bridge is used instead of ros_gz_image,
    so image_transport / FFmpeg publishers are not started.
    """
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
                (gz_topic, ros_topic)
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
                (gz_topic, ros_topic)
            ],
            output="screen",
        )
    )


def bridge_points(actions, name, gz_topic, ros_topic):
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            arguments=[
                f"{gz_topic}"
                "@sensor_msgs/msg/PointCloud2"
                "[gz.msgs.PointCloudPacked"
            ],
            remappings=[
                (gz_topic, ros_topic)
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
                (gz_topic, ros_topic)
            ],
            output="screen",
        )
    )


# ============================================================
# Launch setup
# ============================================================

def launch_setup(context, *args, **kwargs):

    world = LaunchConfiguration("world").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    drone_id = LaunchConfiguration("drone_id").perform(context)
    prefix = LaunchConfiguration("model_prefix").perform(context)

    front_points_enabled = as_bool(context, "front_points")
    front_rgb_enabled = as_bool(context, "front_rgb")
    down_rgb_enabled = as_bool(context, "down_rgb")
    lidar_enabled = as_bool(context, "lidar")

    # PX4 SITL model names:
    #
    # x500_depth_nav2_0
    # x500_depth_nav2_1
    # x500_depth_nav2_2
    #
    model = f"{prefix}_{drone_id}"

    actions = []

    # ========================================================
    # /clock
    # ========================================================

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
                (gz_clock, "/clock")
            ],
            output="screen",
        )
    )

    # ========================================================
    # Front Depth PointCloud
    #
    # Gazebo:
    #   StereoOV7251/depth_image/points
    #
    # ROS:
    #   /MAV1/camera_front/depth/points
    #
    # Used by:
    #   Cartographer
    # ========================================================

    if front_points_enabled:

        front_points_gz = gz_sensor_topic(
            world,
            model,
            "camera_front_link",
            "StereoOV7251",
            "depth_image/points",
        )

        bridge_points(
            actions,
            "front_depth_points_bridge",
            front_points_gz,
            f"/{ns}/camera_front/depth/points",
        )

    # ========================================================
    # Front RGB
    #
    # Used by:
    #   AprilTag front detector
    # ========================================================

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

    # ========================================================
    # Down RGB
    #
    # Used by:
    #   AprilTag down detector
    # ========================================================

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

    # ========================================================
    # 2D LiDAR
    #
    # Used later by:
    #   Nav2 local costmap
    # ========================================================

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

    # ========================================================
    # Static TF
    # ========================================================

    # Front camera TF is required whenever the front PointCloud
    # or front RGB camera is enabled.
    if front_points_enabled or front_rgb_enabled:

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

    # Down camera TF only needed when down RGB is enabled.
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

    # LiDAR TF only needed when LiDAR is enabled.
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


# ============================================================
# Launch description
# ============================================================

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

        # ----------------------------------------------------
        # Sensor switches
        # ----------------------------------------------------

        DeclareLaunchArgument(
            "front_points",
            default_value="true",
            description=(
                "Bridge front depth PointCloud "
                "for Cartographer"
            ),
        ),

        DeclareLaunchArgument(
            "front_rgb",
            default_value="false",
            description=(
                "Bridge front RGB image + CameraInfo "
                "for AprilTag"
            ),
        ),

        DeclareLaunchArgument(
            "down_rgb",
            default_value="false",
            description=(
                "Bridge down RGB image + CameraInfo "
                "for AprilTag"
            ),
        ),

        DeclareLaunchArgument(
            "lidar",
            default_value="false",
            description=(
                "Bridge 2D lidar for Nav2"
            ),
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])