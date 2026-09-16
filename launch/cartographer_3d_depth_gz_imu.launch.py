from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_camera_info_topic = LaunchConfiguration('depth_camera_info_topic')
    raw_points_topic = LaunchConfiguration('raw_points_topic')
    points_topic = LaunchConfiguration('points_topic')
    pointcloud_frame = LaunchConfiguration('pointcloud_frame')
    pointcloud_conversion_mode = LaunchConfiguration('pointcloud_conversion_mode')
    gz_imu_topic = LaunchConfiguration('gz_imu_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic')

    configuration_directory = PathJoinSubstitution([
        FindPackageShare('drone_nav2_apriltag'),
        'config',
    ])

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_base_imu_bridge_for_cartographer',
            arguments=[f'{gz_imu_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[(gz_imu_topic, imu_topic)],
            output='screen',
        ),

        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_to_points',
            parameters=[{
                'queue_size': 30,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('image_rect', depth_image_topic),
                ('camera_info', depth_camera_info_topic),
                ('points', raw_points_topic),
            ],
            output='screen',
        ),

        Node(
            package='drone_nav2_apriltag',
            executable='pointcloud_axis_converter.py',
            name='front_depth_axis_converter',
            parameters=[{
                'input_topic': raw_points_topic,
                'output_topic': points_topic,
                'output_frame_id': pointcloud_frame,
                'mode': pointcloud_conversion_mode,
                'queue_size': 30,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

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
                ('points2', points_topic),
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/MAV1/camera_front/depth/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/MAV1/camera_front/depth/camera_info',
        ),
        DeclareLaunchArgument(
            'raw_points_topic',
            default_value='/cartographer/depth_points_raw',
        ),
        DeclareLaunchArgument(
            'points_topic',
            default_value='/cartographer/depth_points',
        ),
        DeclareLaunchArgument(
            'pointcloud_frame',
            default_value='camera_front_link',
        ),
        DeclareLaunchArgument(
            'pointcloud_conversion_mode',
            default_value='optical_to_link',
        ),
        DeclareLaunchArgument(
            'gz_imu_topic',
            default_value='/world/nav2_arena/model/x500_depth_nav2_0/link/base_link/sensor/imu_sensor/imu',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
        ),
        OpaqueFunction(function=launch_setup),
    ])
