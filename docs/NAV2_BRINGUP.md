# Nav2 on the live GLIM map (solo run)

> **Superseded by `LIVE_MISSION_TEST.md`.** Out-of-date values below: lidar_z is now 0.60, the height band is -0.40/1.20 in BOTH files, and "Waypoints → Nav2" is done (`waypoint_mission.py`). Keep this doc only for the background/design explanations.

## The picture

```
MID-360 ─► driver ─► /livox/lidar ─┬─► angle filter ─► /livox/lidar_filtered ─► GLIM
                                   │                                             │
                                   │           TF map→odom→base_link ◄───────────┤
                                   │           /glim_ros/map (every ~10 s) ◄─────┘
                                   │                     │
                                   │                  pcd2pgm ─► /map (fixed 50×50 m grid)
                                   │                                  │
                                   └──► Nav2 local+global obstacle    ▼
                                        layers                Nav2 global static layer
                                                                      │
                  nav.launch.py: static TF base_link→livox_frame      ▼
                                                        planner → controller → /cmd_vel
```

Nav2 normally needs `map_server` (a saved map) and `AMCL` (localization on that map).
We run **neither**: pcd2pgm feeds `/map` live, GLIM supplies where the robot is.

### TF tree, and who owns each arrow

```
map ──(GLIM)──► odom ──(GLIM)──► base_link ──(nav.launch.py, static)──► livox_frame
```

Every frame has exactly one parent. GLIM computes `odom→base_link` by looking up
`livox_frame↔base_link`, so **the static TF must be running before GLIM starts**
or GLIM logs `Failed to lookup transform from livox_frame to base_link` and Nav2
never sees the robot. That's why nav.launch.py is Terminal 3, before GLIM.

---

## Terminals (VM). Order matters.

Every terminal starts with:
```bash
source /opt/ros/humble/setup.bash
```

**T1: driver**
```bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

**T2: angle filter** (GLIM listens to its output, not the raw cloud)
```bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch lidar_angle_filter angle_filter.launch.py
```

**T3: Nav2 + LiDAR static TF**
```bash
source ~/Kratos/kratos_nav_ws/install/setup.bash
ros2 launch kratos_nav nav.launch.py
```
It will complain about missing `map` frame / no map until GLIM + pcd2pgm are up. Normal.

**T4: GLIM** (all three sources, or waypoint_manager fails to load)
```bash
source ~/Kratos/auto_fastlio2/install/setup.bash
source ~/Kratos/glim_ext_ws/install/setup.bash
source ~/Kratos/glim_ros_fix_ws/install/local_setup.bash  # patched GLIM rviz_viewer (1.2.2 /glim_ros/map bug), must be last
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/Kratos/glim_config)
```

**T5: pcd2pgm live**
```bash
source ~/Kratos/pcd2pgm_live_ws/install/setup.bash
ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file ~/Kratos/pcd2pgm_live_ws/src/pcd2pgm/config/pcd2pgm_live.yaml
```

**T6: RViz**
```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Fixed Frame = `map`. The config has Map, costmaps, path, and the **Nav2 Goal** button.

---

## Checks, in order (stop at the first failure)

| # | Command | Pass |
|---|---|---|
| 1 | `ros2 run tf2_ros tf2_echo base_link livox_frame` | prints the placeholder `0.0 0.0 0.40` |
| 2 | `ros2 run tf2_ros tf2_echo map base_link` | changes when you push the rover |
| 3 | `ros2 topic echo /map --field info --once` | width/height 1000, origin −25 |
| 4 | `ros2 lifecycle get /bt_navigator` | `active [3]` |
| 5 | RViz: global costmap | walls from `/map`, grey inflation around them |
| 6 | RViz: local costmap | a 6×6 m square following the rover, no blob stuck ON the rover |
| 7 | `ros2 topic echo /cmd_vel` + click **Nav2 Goal** 2 m ahead | `linear.x` > 0, and never `linear.x = 0` with `angular.z ≠ 0` for long |

**Test 7 with wheels OFF the ground** until you trust it.

Tip: `ros2 run tf2_tools view_frames` writes `frames.pdf` showing the whole TF tree.

---

## Placeholders you MUST replace with real measurements

| What | Where | Placeholder | How to measure |
|---|---|---|---|
| LiDAR mount height/offset | `nav.launch.py` args `lidar_x`, `lidar_z` | x 0.0, z 0.40 | tape: base_link point → MID-360 centre. Override without editing: `ros2 launch kratos_nav nav.launch.py lidar_z:=0.55` |
| Ground height in map frame | `nav2_params.yaml` `min/max_obstacle_height` (4 places) | −0.41 / 1.19 | ground = −(lidar_z + 0.21). min = ground + 0.2, max = ground + 1.8 |
| **pcd2pgm Z band** | `pcd2pgm_live.yaml` `thre_z_min/max` | 0.2 / 1.8 | **Same issue**: GLIM's z=0 is the LiDAR, not the floor. Today's band keeps 0.2–1.8 m *above the LiDAR*, which is fine for walls, but **low rocks are invisible**. Set to the same min/max as the costmap. |
| Footprint | `nav2_params.yaml` `footprint` (2 places) | 0.74 × 0.74 m | outermost points of the real rover + 5 cm |
| Min turning radius | `planner_server.GridBased.minimum_turning_radius` | 0.6 m | drive full lock in a circle, measure radius to the centre |
| Speed | `FollowPath.desired_linear_vel`, `velocity_smoother.max_velocity` | 0.4 / 0.5 m/s | start slow |

---

## Key design choices (so you can defend them)

- **Regulated Pure Pursuit, `use_rotate_to_heading: false`, no Spin recovery,
  `yaw_goal_tolerance: 6.28`.** The double-Ackermann controller turns `linear.x=0,
  angular.z≠0` into ~146° steering, past the ±90° joint limit. Nothing in this
  config ever asks the rover to turn in place.
- **Smac Hybrid-A\* with DUBIN motion.** Plans only curves the steering can drive.
  NavFn would happily plan a 90° corner the rover can't take.
- **`allow_unknown: true`.** Most of the arena is unknown until you've driven it.
- **Obstacles from raw `/livox/lidar`, not `/livox/lidar_filtered`.** The angle
  filter removes a 30° cone straight ahead: good for GLIM, fatal for obstacle
  avoidance. Self-hits are dropped with `obstacle_min_range: 0.5`.
- **`map_subscribe_transient_local: true`.** pcd2pgm publishes `/map` as
  transient_local; mismatched QoS = silent no-connection.
- **Global costmap has an obstacle layer too.** `/map` lags ~10 s; live obstacles
  fill that gap.

---

## NOT done yet (known gaps)

1. ~~Real rover doesn't listen to `/cmd_vel`.~~ Done: `rover_bridge.py` (see
   `LIVE_MISSION_TEST.md`). Kinematics and the PWM-to-m/s scale are still placeholders.
2. ~~Waypoints → Nav2.~~ Done: `waypoint_mission.py` queries GLIM live and uses
   `navigate_to_pose` (not `follow_waypoints`), so loop-closure moves are followed.
3. **Grid size.** 50×50 m fixed; size to the arena (or grow-on-demand, undecided).

---

## Troubleshooting

| Symptom | Cause → Fix |
|---|---|
| GLIM: `Failed to lookup transform from livox_frame to base_link` | T3 (static TF) not running or started after GLIM → start T3, restart GLIM |
| Nav2: `Timed out waiting for transform from base_link to map` | GLIM not running / not initialized yet (needs a still IMU start) |
| Global costmap empty, `/map` fine | QoS mismatch or wrong topic → `ros2 topic info /map -v`, both sides must be RELIABLE + TRANSIENT_LOCAL |
| Local costmap: permanent blob on the rover | Self-hits → raise `obstacle_min_range` |
| Floor shows up as obstacles everywhere | Ground height wrong → fix `min_obstacle_height` (see placeholders) |
| Planner: `failed to create plan` | Goal inside inflation / turning radius too big for the space → goal further from walls, check radius |
| Rover crawls, never reaches goal | Progress checker: 0.5 m in 15 s; RPP slowing for curvature → check `regulated_linear_scaling_min_speed` |
| Revert GLIM to old TF (no base_link) | `cp ~/Kratos/glim_config/config_ros.json.bak_before_base_link ~/Kratos/glim_config/config_ros.json` |
