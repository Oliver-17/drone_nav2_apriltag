from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    raw_points_topic = LaunchConfiguration('raw_points_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    configuration_directory = PathJoinSubstitution([
        FindPackageShare('drone_nav2_apriltag'),
        'config',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument(
            'raw_points_topic',
            default_value='/MAV1/camera_front/depth/points',
            description='Gazebo native depth PointCloud2 topic used directly by Cartographer',
        ),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', 'cartographer_3d_depth.lua',
            ],
            remappings=[
                ('points2', raw_points_topic),
                ('imu', imu_topic),
            ],
            output='screen',
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
            }],
            output='screen',
        ),
    ])
