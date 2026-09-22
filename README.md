# drone_nav2_apriltag

ROS 2 Humble package for PX4 Gazebo simulation, depth-camera SLAM, topology waypoint flight, AprilTag-area map capture, and empty-world Nav2 testing for MAV1.

This repository is a ROS 2 package, not a full workspace. Clone it under an existing workspace `src/` directory.

## Documents

| Document | Purpose |
|---|---|
| [docs/slam.md](docs/slam.md) | Gazebo camera/depth bridge, Cartographer 3D SLAM, AprilTag route, and map saving |
| [docs/nav2_sim.md](docs/nav2_sim.md) | Nav2 empty-map simulation / bag replay checks for topics, TF, `/plan`, and `/cmd_vel` |
| [docs/nav2_real.md](docs/nav2_real.md) | Real-machine OptiTrack/PX4 preflight check and manual Nav2 flight flow |

## Main Features

- PX4 SITL Gazebo arena for an `x500_depth_nav2` UAV.
- Front depth camera pipeline through `ros_gz_bridge` and `depth_image_proc`.
- Cartographer 3D SLAM using front depth points, Gazebo IMU, and PX4 odometry.
- AprilTag-area route flight with map saving.
- Empty-world Nav2 test with fake static map.
- MAV1 topic-check tools split for bag/simulation and real-machine tests.
- Manual Nav2 flight flow with default takeoff altitude `0.5 m` and automatic landing after goal completion.

## Tested Environment

| Component | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4 | SITL / uXRCE-DDS workflow |
| Gazebo | Harmonic / `gz sim` |
| Mapping | `cartographer_ros`, `nav2_map_server` |
| Navigation | `nav2_bringup` |

## Package Layout

```text
config/
  cartographer_3d_depth.lua          Cartographer 3D configuration
  nav2_params.yaml                   Empty-world Nav2 configuration

docs/
  slam.md                            SLAM and map-saving workflow
  nav2_sim.md                        Nav2 simulation / bag replay workflow
  nav2_real.md                       Real-machine Nav2 workflow

graphs/
  nav2_arena.geojson                 Topology node graph

gz/
  worlds/nav2_arena.sdf              Gazebo arena
  worlds/empty_nav2.sdf              Empty Nav2 test world
  models/x500_depth_nav2/model.sdf   UAV sensor model

launch/
  cameras_depth_proc.launch.py       Recommended camera/depth bridge
  cartographer_3d_depth.launch.py    Cartographer 3D SLAM launch
  fly_nodes_apriltag_route.launch.py AprilTag-area route and map saving
  nav2_empty_world.launch.py         Empty-map Nav2 bringup for sim/bag checks
  manual_nav2_flight.launch.py       Real-machine/Gazebo manual Nav2 flight
  mav1_nav2_topic_check_sim.launch.py  MAV1 sim/bag topic and TF check
  mav1_nav2_topic_check_real.launch.py MAV1 real-machine topic and TF check

scripts/
  start_arena_sitl.sh                PX4/Gazebo startup helper
  fly_nodes.py                       PX4 offboard waypoint flight controller
  px4_vehicle_odometry_to_odom.py    PX4 odometry to nav_msgs/Odometry + TF
  mav1_nav2_topic_check.py           MAV1 topic/TF readiness checker
  takeoff_manual_nav2.py             Takeoff, manual goal, Nav2, PX4 command bridge
  manual_nav2_goal_terminal.py       Terminal ENU start/goal input helper
```

## Build

From the workspace root:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
colcon build --packages-select drone_nav2_apriltag
source install/setup.bash
```

## Quick Entry Points

SLAM / map capture:

```bash
# See docs/slam.md
ros2 launch drone_nav2_apriltag cartographer_3d_depth.launch.py
```

Nav2 bag / simulation topic check:

```bash
# See docs/nav2_sim.md
ros2 launch drone_nav2_apriltag mav1_nav2_topic_check_sim.launch.py
```

Real-machine preflight topic check:

```bash
# See docs/nav2_real.md
ros2 launch drone_nav2_apriltag mav1_nav2_topic_check_real.launch.py
```

Real-machine manual Nav2 flight, default altitude `0.5 m`:

```bash
# See docs/nav2_real.md
ros2 launch drone_nav2_apriltag manual_nav2_flight.launch.py rviz:=true
```

## Notes

- The empty-world Nav2 flow uses a fake static map and does not use camera point clouds for obstacle avoidance.
- Bag replay is open-loop: it can validate topics, coordinate frames, TF, `/plan`, and `/cmd_vel`, but it cannot prove the UAV will reach a goal.
- Real-machine flight should only proceed after OptiTrack/PX4 odometry and TF are verified.
- For stricter real-machine checking, run the topic checker with `require_yaw_good:=true` and confirm PX4 yaw readiness before flight.

## License

MIT
