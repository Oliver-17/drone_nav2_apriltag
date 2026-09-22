from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    px4_namespace = LaunchConfiguration("px4_namespace")
    px4_odom_topic = LaunchConfiguration("px4_odom_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    mocap_pose_topic = LaunchConfiguration("mocap_pose_topic")
    stamp_with_ros_time = LaunchConfiguration("stamp_with_ros_time")
    require_mocap = LaunchConfiguration("require_mocap")
    require_yaw_good = LaunchConfiguration("require_yaw_good")
    exit_when_ready = LaunchConfiguration("exit_when_ready")
    topic_timeout = LaunchConfiguration("topic_timeout")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("px4_namespace", default_value="/MAV1"),
        DeclareLaunchArgument("px4_odom_topic", default_value="/MAV1/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("mocap_pose_topic", default_value="/vrpn_mocap/MAV1/pose_reliable"),
        DeclareLaunchArgument("stamp_with_ros_time", default_value="true"),
        DeclareLaunchArgument("require_mocap", default_value="false"),
        DeclareLaunchArgument("require_yaw_good", default_value="false"),
        DeclareLaunchArgument("exit_when_ready", default_value="false"),
        DeclareLaunchArgument("topic_timeout", default_value="2.5"),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom_for_check",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),

        Node(
            package="drone_nav2_apriltag",
            executable="px4_vehicle_odometry_to_odom.py",
            name="px4_odom_for_topic_check",
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
            package="drone_nav2_apriltag",
            executable="mav1_nav2_topic_check.py",
            name="mav1_nav2_topic_check",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "px4_namespace": px4_namespace,
                "mocap_pose_topic": mocap_pose_topic,
                "odom_frame": "odom",
                "base_frame": "base_link",
                "map_frame": "map",
                "topic_timeout": topic_timeout,
                "tf_timeout": 0.2,
                "report_period": 1.0,
                "require_mocap": require_mocap,
                "require_yaw_good": require_yaw_good,
                "exit_when_ready": exit_when_ready,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
