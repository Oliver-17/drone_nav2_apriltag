# Nav2 Real-Machine Flow

This document is for OptiTrack/PX4 real-machine testing. Use the topic checker first, then run the manual Nav2 flight flow only after odometry and TF are ready.

The default manual flight altitude is `0.5 m`.

## 1. Start Real-Machine Sources

Start the systems that provide real MAV1 data:

```text
OptiTrack / VRPN
PX4
Micro XRCE-DDS Agent
```

Do not start bag replay, Gazebo SITL, or another odometry publisher at the same time as the real machine.

## 2. Basic Topic Check

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag mav1_nav2_topic_check_real.launch.py
```

This checks:

```text
/MAV1/fmu/out/vehicle_odometry
/MAV1/fmu/out/vehicle_local_position_v1
/MAV1/fmu/out/estimator_status_flags
/MAV1/fmu/out/vehicle_status_v1
/vrpn_mocap/MAV1/pose_reliable
/odom
odom -> base_link
map -> base_link
```

Expected basic result:

```text
Nav2 readiness: READY
[topic] vehicle_odometry: ok
[topic] vehicle_local_position_v1: ok
[topic] mocap_pose: ok
[tf] odom -> base_link: ok
[tf] map -> base_link: ok
```

## 3. Strict Yaw Readiness Check

Before flying, run the stricter check:

```bash
ros2 launch drone_nav2_apriltag mav1_nav2_topic_check_real.launch.py require_yaw_good:=true
```

The strict check expects:

```text
xy_valid: true
z_valid: true
cs_ev_pos: true
cs_ev_hgt: true
cs_yaw_align: true
heading_good_for_control: true
```

If this reports `NOT READY`, fix PX4/EKF/mocap yaw alignment before flying. The coordinate-axis bag is not a valid yaw-readiness test because that bag was recorded to measure axes, not to prove stable yaw.

## 4. Manual Nav2 Flight

After the real-machine topic check is ready, run:

```bash
cd /home/zhg/ncrl_mqtt/catkin_ws
source install/setup.bash
ros2 launch drone_nav2_apriltag manual_nav2_flight.launch.py rviz:=true
```

Current behavior:

- takes off to `0.5 m` by default
- waits for manual start and goal coordinates
- sends a Nav2 `NavigateToPose` goal
- converts Nav2 `/cmd_vel` into PX4 NED offboard setpoints
- lands automatically after Nav2 reports the goal is finished

Useful launch arguments:

| Argument | Default | Meaning |
|---|---:|---|
| `takeoff_altitude` | `0.5` | Fixed flight altitude in meters |
| `altitude_tolerance` | `0.1` | Takeoff altitude tolerance in meters |
| `max_xy_speed` | `0.7` | Maximum Nav2 XY velocity sent to PX4 |
| `land_after_goal` | `true` | Send PX4 land command after Nav2 finishes |
| `land_on_abort` | `true` | Send PX4 land command if the flow aborts while armed |
| `rviz` | `false` | Open the included Nav2 RViz config |

## 5. Enter Start And Goal

After takeoff reaches altitude and the node waits for input, open another terminal:

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

Start with a short test:

```text
start> 0 0 0
goal> 0.3 0 0
```

If the direction is correct, test a longer move:

```text
start> 0 0 0
goal> 1.0 0 0
```

The typed coordinates define the desired ENU displacement. Nav2 decides arrival from `/odom` and TF, not from the typed coordinates alone.

Default tolerances:

```text
xy_goal_tolerance: 0.35 m
yaw_goal_tolerance: 0.35 rad
```

## 6. Coordinate Frame Reminder

PX4 local odometry is NED. The ROS/Nav2 side uses ENU:

```text
ROS ENU x = PX4 NED y
ROS ENU y = PX4 NED x
ROS ENU z = -PX4 NED z
```

Nav2 must have:

```text
map -> odom -> base_link
```

## 7. RViz

The included RViz config is:

```text
rviz/nav2_empty_local.rviz
```

Open with the flight launch:

```bash
ros2 launch drone_nav2_apriltag manual_nav2_flight.launch.py rviz:=true
```

Or manually:

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

## 8. Important Notes

- Do not run `nav2_empty_world.launch.py`, `px4_nav2_bridge.launch.py`, bag replay, or the topic-check launch at the same time as `manual_nav2_flight.launch.py`.
- The manual launch already starts Nav2, odometry TF, and the PX4 command bridge.
- This flow uses a fake empty map and no point-cloud obstacle layer, so real obstacles are not used for avoidance.
- Use very small goals first, such as `0.3 m`, to confirm direction before increasing distance.
