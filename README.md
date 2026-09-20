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

## Empty World Nav2 Test

For a simple Nav2 test without Cartographer, start PX4/Gazebo with the empty world:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws/src/drone_nav2_apriltag/scripts
PX4_GZ_WORLD=empty_nav2 DRONES=1 ./start_arena_sitl.sh
```

Start the XRCE-DDS agent in another terminal:

```bash
MicroXRCEAgent udp4 -p 8888
```

Then start Nav2 from the workspace:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag nav2_empty_world.launch.py
```

This launch uses the empty static map, publishes a temporary static `map -> odom`, and starts the PX4 odometry converter with `publish_tf:=true` so Nav2 can see `map -> odom -> base_link`. Remove the static `map -> odom` when Cartographer or another localization source publishes it.

Start the Nav2-to-PX4 velocity bridge:

```bash
ros2 launch drone_nav2_apriltag px4_nav2_bridge.launch.py
```

Send a simple goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Manual Takeoff + Nav2 Flight

This flow keeps the existing takeoff tools untouched and uses a new node, `takeoff_manual_nav2.py`. It takes off to a fixed altitude, waits for manual start/goal input, publishes `map -> odom` from the entered start pose, sends a Nav2 goal, and converts Nav2 `/cmd_vel` to PX4 offboard setpoints.

Start PX4/Gazebo first, then launch the combined Nav2 flight flow:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag manual_nav2_flight.launch.py
```

After takeoff, open another terminal and publish the poses:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 run drone_nav2_apriltag manual_nav2_goal_terminal.py
```

Then enter:

```text
start> 0 0 0  # east north yaw_deg
goal> 5 0 0  # east north yaw_deg
```

The format is `east north yaw_deg` in the ENU `map` frame. PX4 setpoints are converted to NED automatically. Do not run another static `map -> odom` publisher with this launch, because this node publishes `map -> odom` from the manual start pose.

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
