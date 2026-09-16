from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    common_params = {
        'frame_id': 'base_link',
        'approx_sync': True,
        'approx_sync_max_interval': 0.2,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        # ros_gz_image / ros_gz_bridge camera topics often use sensor-data QoS.
        # qos=2 makes RTAB-Map subscribe with best-effort QoS instead of missing data silently.
        'qos': 2,
        'wait_for_transform': 1.0,
        'use_sim_time': use_sim_time,
        'odom_frame_id': 'odom',
        'publish_tf': True,
    }

    odom_params = {
        **common_params,
        'Vis/MinInliers': '8',
        'Odom/ResetCountdown': '1',
    }

    slam_params = {
        **common_params,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
    }

    camera_remaps = [
        (
            'rgb/image',
            '/MAV1/camera_front/rgb/image_raw'
        ),
        (
            'rgb/camera_info',
            '/MAV1/camera_front/rgb/camera_info'
        ),
        (
            'depth/image',
            '/MAV1/camera_front/depth/image_raw'
        ),
        (
            'depth/camera_info',
            '/MAV1/camera_front/depth/camera_info'
        ),
    ]

    rgbd_odom_remaps = camera_remaps + [
        ('odom', '/odom'),
    ]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use Gazebo simulation clock'
        ),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='--delete_db_on_start',
            description='RTAB-Map command-line args. Default clears old bad maps on each run.'
        ),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[odom_params],
            remappings=rgbd_odom_remaps,
            output='screen',
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[slam_params],
            remappings=rgbd_odom_remaps,
            arguments=[LaunchConfiguration('rtabmap_args')],
            output='screen',
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[slam_params],
            remappings=rgbd_odom_remaps,
            output='screen',
        ),
    ])