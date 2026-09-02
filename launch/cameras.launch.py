#!/usr/bin/env python3
"""
cameras.launch.py —— 把 x500_nav2 的前視／下視相機與 2D 光達從 Gazebo 橋到 ROS 2，
                     並（預設）開兩個 rqt_image_view 視窗，讓你在無人機走節點的
                     過程中同時看到兩顆相機的畫面。

用法（先跑 start_arena_sitl.sh）：
    ros2 launch drone_nav2_apriltag cameras.launch.py
    ros2 launch drone_nav2_apriltag cameras.launch.py drone_id:=1 namespace:=MAV2
    ros2 launch drone_nav2_apriltag cameras.launch.py view:=false     # 只橋接不開視窗

為什麼需要「橋」這一層：
Gazebo 走的是 gz-transport，ROS 2 走的是 DDS，兩邊是完全不同的傳輸層，
不會自動互通。ros_gz_image / ros_gz_bridge 就是同時掛在兩邊的轉接程序。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gz_sensor_topic(world, model, link, sensor, leaf):
    """組出 gz 自動生成的感測器 topic 名稱。

    為什麼要自己組這一長串：x500_nav2/model.sdf 裡刻意沒有寫 <topic>。
    寫死的話 PX4 spawn 出來的三台會全部發到同一個 topic 上互相蓋掉，
    自動生成的名字含 model instance 編號（x500_nav2_0 / _1 / _2）才分得開。
    格式取自實機執行時 `gz topic -l` 的輸出，不是猜的。
    """
    return f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/{leaf}"


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    drone_id = LaunchConfiguration("drone_id").perform(context)
    prefix = LaunchConfiguration("model_prefix").perform(context)
    view = LaunchConfiguration("view").perform(context).lower() in ("true", "1", "yes")
    lidar = LaunchConfiguration("lidar").perform(context).lower() in ("true", "1", "yes")

    # PX4 spawn 時的命名規則是 "${MODEL_NAME}_${px4_instance}"
    # （ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim:111）
    model = f"{prefix}_{drone_id}"

    cams = [
        ("front", "camera_front_link", "imager_front"),
        ("down",  "camera_down_link",  "imager_down"),
    ]

    actions = []
    ros_image_topics = []

    for short, link, sensor in cams:
        gz_img = gz_sensor_topic(world, model, link, sensor, "image")
        gz_info = gz_sensor_topic(world, model, link, sensor, "camera_info")
        ros_img = f"/{ns}/camera_{short}/image_raw"
        ros_info = f"/{ns}/camera_{short}/camera_info"
        ros_image_topics.append(ros_img)

        # 影像用 image_bridge 而不是 parameter_bridge：它走 image_transport，
        # 會順便提供 /compressed 等傳輸方式，rqt_image_view 和之後的
        # apriltag_ros 都吃這一套。
        actions.append(Node(
            package="ros_gz_image", executable="image_bridge",
            name=f"image_bridge_{short}",
            arguments=[gz_img],
            # image_bridge 發出來的 ROS topic 名稱就等於 gz topic 名稱，
            # 用 remap 換成好念的短名字，三台才不會互相干擾。
            remappings=[(gz_img, ros_img)],
            output="screen",
        ))

        # camera_info 是一般訊息，image_bridge 不管，要另外橋。
        # 之後 apriltag_ros 要靠它做位姿估計，現在先接起來免得將來忘記。
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name=f"info_bridge_{short}",
            arguments=[f"{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            remappings=[(gz_info, ros_info)],
            output="screen",
        ))

    if lidar:
        gz_scan = gz_sensor_topic(world, model, "lidar_link", "lidar_2d", "scan")
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="lidar_bridge",
            arguments=[f"{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            remappings=[(gz_scan, f"/{ns}/scan")],
            output="screen",
        ))

    if view:
        for i, topic in enumerate(ros_image_topics):
            # rqt_image_view 一個視窗只看一個 topic，所以開兩個。
            # 想擠在同一個視窗看的話，改用 view_graph.launch.py 的 RViz，
            # rviz/arena.rviz 裡已經放好兩個 Image display。
            actions.append(Node(
                package="rqt_image_view", executable="rqt_image_view",
                name=f"image_view_{i}",
                arguments=[topic],
                output="log",   # rqt 的 Qt 警告很吵，丟到 log 就好
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="nav2_arena",
                              description="Gazebo 世界名稱，要跟 start_arena_sitl.sh 一致"),
        DeclareLaunchArgument("drone_id", default_value="0",
                              description="PX4 instance 編號（0/1/2），決定 gz 那邊的模型名"),
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="ROS 這邊的前綴，跟 PX4_UXRCE_DDS_NS 對齊"),
        DeclareLaunchArgument("model_prefix", default_value="x500_nav2",
                              description="機體模型名，跟 start_arena_sitl.sh 的 SIM_MODEL 一致"),
        DeclareLaunchArgument("view", default_value="true",
                              description="是否開 rqt_image_view 視窗"),
        DeclareLaunchArgument("lidar", default_value="true",
                              description="是否一併橋接 2D 光達"),
        OpaqueFunction(function=launch_setup),
    ])
