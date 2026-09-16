from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    raw_points_topic = LaunchConfiguration('raw_points_topic')
    gz_imu_topic = LaunchConfiguration('gz_imu_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    px4_odom_topic = LaunchConfiguration('px4_odom_topic')
    odom_topic = LaunchConfiguration('odom_topic')

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
        DeclareLaunchArgument(
            'gz_imu_topic',
            default_value='/world/nav2_arena/model/x500_depth_nav2_0/link/base_link/sensor/imu_sensor/imu',
            description='Gazebo base_link IMU topic bridged to ROS for Cartographer',
        ),
        # Kept only so older wrappers that still pass px4_imu_topic do not fail.
        DeclareLaunchArgument(
            'px4_imu_topic',
            default_value='/MAV1/fmu/out/sensor_combined',
            description='Deprecated: Cartographer now uses gz_imu_topic instead',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='base_link',
            description='Deprecated: Gazebo IMU already carries frame_id from Gazebo',
        ),
        DeclareLaunchArgument(
            'px4_odom_topic',
            default_value='/MAV1/fmu/out/vehicle_odometry',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_base_imu_bridge_for_cartographer',
            arguments=[[
                gz_imu_topic,
                '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ]],
            remappings=[
                (gz_imu_topic, imu_topic),
            ],
            output='screen',
        ),

        Node(
            package='drone_nav2_apriltag',
            executable='px4_vehicle_odometry_to_odom.py',
            name='px4_odom_for_cartographer',
            parameters=[{
                'input_topic': px4_odom_topic,
                'output_topic': odom_topic,
                'odom_frame_id': 'odom',
                'child_frame_id': 'base_link',
                'queue_size': 30,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # Cartographer 3D SLAM consumes Gazebo native PointCloud2 directly.

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
                ('odom', odom_topic),
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
