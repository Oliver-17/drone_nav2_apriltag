cmake_minimum_required(VERSION 3.8)
project(drone_nav2_apriltag)

find_package(ament_cmake REQUIRED)

# 為什麼把 gz/ 整包裝進 share：Gazebo 是透過 GZ_SIM_RESOURCE_PATH 找 world 與 model 的，
# 目錄結構必須原封不動保留（model://apriltag_36h11 會展開成 <某個路徑>/apriltag_36h11/model.sdf）。
install(DIRECTORY gz graphs launch docs rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_arena_sitl.sh
  scripts/gen_arena.py
  scripts/check_graph.py
  scripts/fly_nodes.py
  scripts/scan_map.py
  scripts/takeoff_hold.py
  scripts/start_slam_when_airborne.py
  scripts/graph_markers.py
  scripts/pointcloud_axis_converter.py
  scripts/px4_sensor_combined_to_imu.py
  scripts/px4_vehicle_odometry_to_odom.py
  scripts/check_slam_inputs.py
  scripts/check_yaw_topics.py
  scripts/gz_pose_tf_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
