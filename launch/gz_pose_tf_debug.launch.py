from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    pose_topic = f'/world/{world}/dynamic_pose/info'
    ros_pose_topic = LaunchConfiguration('ros_pose_topic').perform(context)

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_pose_info_bridge',
            arguments=[f'{pose_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[(pose_topic, ros_pose_topic)],
            output='screen',
        ),
        Node(
            package='drone_nav2_apriltag',
            executable='gz_pose_tf_debug.py',
            name='gz_pose_tf_debug',
            parameters=[{
                'input_topic': ros_pose_topic,
                'target_name': LaunchConfiguration('target_name'),
                'target_index': LaunchConfiguration('target_index'),
                'parent_frame': LaunchConfiguration('parent_frame'),
                'child_frame': LaunchConfiguration('child_frame'),
                'queue_size': 10,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='nav2_arena'),
        DeclareLaunchArgument('ros_pose_topic', default_value='/gz_pose_info'),
        DeclareLaunchArgument('target_name', default_value='x500_depth_nav2_0'),
        DeclareLaunchArgument('target_index', default_value='0'),
        DeclareLaunchArgument('parent_frame', default_value='world'),
        DeclareLaunchArgument('child_frame', default_value='gt_base_link'),
        OpaqueFunction(function=launch_setup),
    ])
