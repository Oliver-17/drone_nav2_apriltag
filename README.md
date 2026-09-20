# drone_nav2_apriltag

ROS 2 Humble package for PX4 Gazebo simulation, depth-camera Cartographer 3D SLAM, topology-based waypoint flight, and AprilTag-area map capture.

This repository is a ROS 2 package, not a full workspace. Clone it into an existing workspace under `src/`.

## Features

- PX4 SITL Gazebo arena for an x500 depth-camera UAV.
- Custom `x500_depth_nav2` model with front depth camera, optional RGB cameras, lidar, and static sensor TF.
- Gazebo to ROS 2 camera bridge using `ros_gz_bridge` and `depth_image_proc`.
- Cartographer 3D SLAM using front depth point cloud, Gazebo IMU, and PX4 odometry.
- Manual topology route flight through `fly_nodes.py`.
- AprilTag-area route:
  - flies `1 -> 4 -> 6 -> 3 -> 1 -> 3 -> 10`
  - slows near turns
  - waits at turn nodes for IMU/attitude settling
  - rotates once at node `10`
  - does not land at node `10`
  - saves `/map` using `nav2_map_server map_saver_cli`

## Tested Environment

| Component | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4 | SITL / uXRCE-DDS workflow |
| Gazebo | Harmonic / `gz sim` |
| Mapping | `cartographer_ros`, `nav2_map_server` |

## Package Layout

```text
config/
  cartographer_3d_depth.lua          Cartographer 3D configuration

graphs/
  nav2_arena.geojson                 Topology node graph

gz/
  worlds/nav2_arena.sdf              Gazebo arena
  models/x500_depth_nav2/model.sdf   UAV sensor model

launch/
  cameras_depth_proc.launch.py       Recommended camera/depth bridge
  cartographer_3d_depth.launch.py    Current Cartographer 3D SLAM launch
  fly_nodes_apriltag_route.launch.py AprilTag-area route and map saving
  fly_nodes.launch.py                Generic route-server waypoint flight
  scan_map.launch.py                 Lawnmower-style scan flight

scripts/
  fly_nodes.py                       PX4 offboard waypoint flight controller
  start_arena_sitl.sh                PX4/Gazebo startup helper
  px4_vehicle_odometry_to_odom.py    PX4 odometry to nav_msgs/Odometry
  check_slam_inputs.py               SLAM input sanity checker
  check_yaw_topics.py                Yaw/topic timing diagnostics
```

## Build

From your workspace root:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
colcon build --packages-select drone_nav2_apriltag
source install/setup.bash
```

## Recommended Runtime Flow

Use separate terminals. This keeps failures easy to isolate.

### 1. Start Gazebo and PX4 SITL

```bash
cd /home/zhg/ncrl_mqtt
DRONES=1 ./scripts/start_arena_sitl.sh
```

### 2. Start Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3. Start the Camera/Depth Bridge

Use the depth-image pipeline, not the native Gazebo point-cloud bridge:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag cameras_depth_proc.launch.py
```

This publishes:

```text
/MAV1/camera_front/depth/image_raw
/MAV1/camera_front/depth/camera_info
/MAV1/camera_front/depth/points
/tf_static: base_link -> camera_front_link -> camera_front_optical_frame
```

Check it:

```bash
ros2 topic echo /MAV1/camera_front/depth/points --field header --once
ros2 topic hz /MAV1/camera_front/depth/points
ros2 run tf2_ros tf2_echo camera_front_link camera_front_optical_frame
```

Expected point cloud frame:

```text
camera_front_optical_frame
```

The point cloud uses the optical frame convention and is transformed through TF. The Python point-cloud axis converter is not used in the recommended flow.

### 4. Start Cartographer 3D SLAM

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag cartographer_3d_depth.launch.py
```

Current SLAM inputs:

```text
points2 <- /MAV1/camera_front/depth/points
imu     <- /imu          # bridged from Gazebo base_link IMU
odom    <- /odom         # converted from PX4 vehicle_odometry
```

Check services and topics:

```bash
ros2 topic echo /imu --field header --once
ros2 topic echo /odom --field header --once
ros2 topic echo /map --field info.resolution --once
ros2 topic hz /map
```

### 5. Fly the AprilTag Route and Save the Map

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag fly_nodes_apriltag_route.launch.py
```

Default route:

```text
1 -> 4 -> 6 -> 3 -> 1 -> 3 -> 10
```

At node `10`, the UAV rotates in place once, saves the occupancy grid map, and finishes without landing.

Default map save command is equivalent to:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/zhg/ncrl_mqtt/maps/arena
```

Expected output:

```text
/home/zhg/ncrl_mqtt/maps/arena.yaml
/home/zhg/ncrl_mqtt/maps/arena.pgm
```

Override save path:

```bash
ros2 launch drone_nav2_apriltag fly_nodes_apriltag_route.launch.py \
  save_map_file:=/home/zhg/ncrl_mqtt/maps/test_arena
```

## Useful Launch Arguments

### `cameras_depth_proc.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `world` | `nav2_arena` | Gazebo world name |
| `drone_id` | `0` | PX4/Gazebo model suffix |
| `namespace` | `MAV1` | ROS namespace for camera topics |
| `model_prefix` | `x500_depth_nav2` | Gazebo model prefix |
| `front_depth` | `true` | Bridge depth image/info and generate point cloud |
| `front_rgb` | `false` | Bridge front RGB image/info |
| `down_rgb` | `false` | Bridge downward RGB image/info |
| `lidar` | `false` | Bridge 2D lidar |

### `cartographer_3d_depth.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `raw_points_topic` | `/MAV1/camera_front/depth/points` | Depth point cloud input |
| `gz_imu_topic` | `/world/nav2_arena/model/x500_depth_nav2_0/link/base_link/sensor/imu_sensor/imu` | Gazebo IMU source |
| `imu_topic` | `/imu` | ROS IMU topic for Cartographer |
| `px4_odom_topic` | `/MAV1/fmu/out/vehicle_odometry` | PX4 odometry source |
| `odom_topic` | `/odom` | ROS odometry topic for Cartographer |

### `fly_nodes_apriltag_route.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `flight_altitude` | `3.0` | Flight altitude in meters |
| `arrival_radius` | `1.5` | Radius for considering a node reached |
| `cruise_speed` | `0.20` | Normal XY setpoint speed |
| `turn_slow_speed` | `0.08` | XY speed near turning nodes |
| `turn_settle_time` | `1.5` | Stop time at turn nodes before moving again |
| `leg_timeout` | `180.0` | Per-leg timeout in seconds |
| `scan_yaw_speed_deg_s` | `18.0` | In-place yaw scan speed at node 10 |
| `scan_yaw_turns` | `1.0` | Number of yaw rotations at node 10 |
| `save_map_file` | `/home/zhg/ncrl_mqtt/maps/arena` | Output prefix for `map_saver_cli` |

## Empty World Nav2 Manual Flight Test

This flow is for testing Nav2 path planning and PX4 offboard motion in an empty Gazebo world, without Cartographer or SLAM. It uses a fake empty static map, a static `map -> odom` transform, PX4 odometry as `/odom`, and manual ENU start/goal input.

Current behavior:

- takes off to `0.5 m`
- waits for manual start and goal coordinates
- sends a Nav2 `NavigateToPose` goal
- converts Nav2 `/cmd_vel` into PX4 NED offboard setpoints
- lands automatically after Nav2 reports the goal is finished
- does not use camera point cloud or local obstacle avoidance in this test flow

### 1. Start Micro XRCE-DDS Agent

Use a separate terminal:

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. Start PX4/Gazebo Empty World

Use the empty Nav2 world:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws/src/drone_nav2_apriltag/scripts
PX4_GZ_WORLD=empty_nav2 DRONES=1 ./start_arena_sitl.sh
```

Before launching Nav2, confirm PX4 local position is valid:

```bash
source /home/zhg/ncrl_mqtt/catkin_ws/install/setup.bash
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /MAV1/fmu/out/estimator_status_flags --once
```

For takeoff, `vehicle_local_position_v1` should have valid local position, especially:

```text
xy_valid: true
z_valid: true
```

If `xy_valid` is still `false`, the takeoff node will keep waiting. Check PX4/Gazebo, XRCE-DDS, GPS/magnetometer/yaw alignment, or OptiTrack/mocap input.

### 3. Start Manual Nav2 Flight

Use another terminal:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag manual_nav2_flight.launch.py rviz:=true
```

Useful launch arguments:

| Argument | Default | Meaning |
|---|---:|---|
| `takeoff_altitude` | `0.5` | Fixed flight altitude in meters |
| `altitude_tolerance` | `0.1` | Takeoff altitude tolerance in meters |
| `max_xy_speed` | `0.7` | Maximum Nav2 XY velocity sent to PX4 |
| `land_after_goal` | `true` | Send PX4 land command after Nav2 finishes |
| `land_on_abort` | `true` | Send PX4 land command if the flow aborts while armed |
| `rviz` | `false` | Open the included Nav2 RViz config |

This launch provides the TF chain Nav2 needs:

```text
map -> odom -> base_link
```

`map -> odom` is a static identity transform for the fake empty map. `odom -> base_link` comes from PX4 `vehicle_odometry` converted to ROS `/odom`.

Do not run `nav2_empty_world.launch.py` or `px4_nav2_bridge.launch.py` at the same time as `manual_nav2_flight.launch.py`; the manual launch already starts Nav2, odometry TF, and the PX4 command bridge.

### 4. Enter Start and Goal Coordinates

After the UAV reaches takeoff altitude and waits for input, open another terminal:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 run drone_nav2_apriltag manual_nav2_goal_terminal.py
```

Input format is ENU:

```text
start> east north yaw_deg
goal> east north yaw_deg
```

Example:

```text
start> 0 0 0
goal> 2 0 0
```

The entered coordinates can be OptiTrack-style ENU coordinates. The program uses the entered start and goal to compute the relative goal from the current odometry pose. Nav2 then decides whether the UAV has arrived by continuously checking `/odom` / TF, not by the typed coordinates alone.

Default Nav2 goal tolerance is:

```text
xy_goal_tolerance: 0.35 m
yaw_goal_tolerance: 0.35 rad
```

When the UAV is within tolerance, Nav2 reports success and the takeoff/manual node sends the land command.

### 5. RViz

The included RViz config is:

```text
rviz/nav2_empty_local.rviz
```

It is opened automatically with `rviz:=true`, or manually with:

```bash
rviz2 -d /home/zhg/ncrl_mqtt/catkin_ws/install/drone_nav2_apriltag/share/drone_nav2_apriltag/rviz/nav2_empty_local.rviz
```

Main displays:

- `/map`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/plan`
- `/plan_smoothed`
- `/local_plan`
- `/odom`
- TF

Because this test uses a fake empty map and no point-cloud obstacle layer, Gazebo obstacles are not used for avoidance in this flow. Use it to verify Nav2 planning, TF, `/cmd_vel`, PX4 offboard control, and OptiTrack/PX4 odometry feedback.

## Diagnostics

Check SLAM inputs:

```bash
ros2 run drone_nav2_apriltag check_slam_inputs.py
```

Check yaw, topic rate, and timestamp alignment:

```bash
ros2 run drone_nav2_apriltag check_yaw_topics.py --ros-args -p use_sim_time:=true
```

Check point cloud rate:

```bash
ros2 topic hz /MAV1/camera_front/depth/points
```

Check Gazebo raw sensor rate:

```bash
gz topic -f -d 10 -t /world/nav2_arena/model/x500_depth_nav2_0/link/camera_front_link/sensor/StereoOV7251/depth_image
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo base_link camera_front_optical_frame
ros2 run tf2_ros tf2_echo map base_link
```

## Notes and Current Design Choices

- Do not run `cameras.launch.py` and `cameras_depth_proc.launch.py` at the same time. They may publish the same camera topics.
- The recommended depth pipeline bridges `depth_image` and `camera_info`, then uses `depth_image_proc` to generate `PointCloud2` in ROS.
- Point cloud numeric axes remain in `camera_front_optical_frame`; TF handles the transform to `camera_front_link` and `base_link`.
- Cartographer uses Gazebo IMU because the PX4 `sensor_combined` IMU was observed to be less stable for this SLAM setup.
- PX4 odometry is still used as `/odom` input to Cartographer.
- The saved `.yaml/.pgm` map comes from `/map`, not from Cartographer `.pbstream` state.

## Common Problems

### Cartographer says `camera_front_optical_frame` does not exist

Make sure `cameras_depth_proc.launch.py` is running, because it publishes:

```text
camera_front_link -> camera_front_optical_frame
```

### `/MAV1/camera_front/depth/points` has low or unstable rate

Compare each stage:

```bash
gz topic -f -d 10 -t /world/nav2_arena/model/x500_depth_nav2_0/link/camera_front_link/sensor/StereoOV7251/depth_image
ros2 topic hz /MAV1/camera_front/depth/image_raw
ros2 topic hz /MAV1/camera_front/depth/points
```

If Gazebo is stable but ROS points are not, the bottleneck is likely bridge / DDS / `depth_image_proc` / CPU load.

### Route stops and lands before finishing

`fly_nodes.py` lands when a leg exceeds `leg_timeout`. Increase timeout or arrival radius:

```bash
ros2 launch drone_nav2_apriltag fly_nodes_apriltag_route.launch.py \
  arrival_radius:=1.8 \
  leg_timeout:=240.0
```

## License

MIT
