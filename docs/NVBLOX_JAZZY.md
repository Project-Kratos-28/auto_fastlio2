# GLIM + nvblox on the Orin (ROS 2 Jazzy)

This branch (`jazzy-nvblox`) runs the whole rover stack on the Jetson AGX Orin
(JetPack 7.2, Ubuntu 24.04, ROS 2 Jazzy) and adds a ZED 2i depth camera fused by
nvblox to the Nav2 local costmap. Everything else (GLIM mission, waypoints, pcd2pgm,
Nav2 tuning) is unchanged from `LIVE_MISSION_TEST.md`.

## Why nvblox, and what it does

The MID-360's vertical field of view is -7 deg to +52 deg. At `lidar_z` 0.60 m its lowest beam
reaches the ground about 4.9 m out, so a 30 cm rock 2 m ahead is invisible to it (an
obstacle at distance d is seen only if it is taller than 0.60 - d * tan 7 deg). The local
costmap also ignores the first 0.5 m (`obstacle_min_range`). A ZED 2i tilted down covers
that near field. nvblox fuses its depth into a 3 cm TSDF in GLIM's `odom` frame and
publishes a 2D slice that Nav2's local costmap reads through `nvblox_layer`.

```
MID-360 -> driver -> angle filter -> GLIM (CPU) -> TF map->odom->base_link, /glim_ros/map -> pcd2pgm -> /map
                  \-> /livox/lidar -> Nav2 obstacle_layer (local + global)        (unchanged)
ZED 2i (no tracking, no TF) -> ESS depth (or ZED NEURAL) -> nvblox (odom, 3 cm, camera only)
        -> /nvblox_node/static_map_slice -> Nav2 local_costmap nvblox_layer         (new)
nav.launch.py: static base_link->livox_frame and base_link->zed_camera_link          (camera TF new)
```

## What changed from the Humble stack

| Area | Change | Why |
|---|---|---|
| Container | Runs as the host user with its `video`/`render` groups; shares the ZED SDK model cache with `~/kratos_nvblox/zed` | Without `video`, CUDA/TensorRT and GLIM's viewer cannot open the GPU (`NvRmMemInitNvmap failed: Permission denied`). |
| Platform | Everything runs in one container, `kratos/glim_nvblox` (`docker/`), built on the kratos nvblox image | nvblox's costmap layer is a Jazzy plugin that loads inside Nav2's process, so Nav2 must be Jazzy too. The Orin has no Humble. |
| GLIM | apt `ros-jazzy-glim-ros` 1.2.2 (CPU) + the same `glim_ros_fix` overlay, built into the image | Same version as before. CPU keeps the GPU for depth + nvblox. |
| Livox | Livox-SDK2 built in the image; driver built with `-DDISTRO_ROS=jazzy` | The driver already supports Jazzy. |
| `kratos_nav/launch/nav.launch.py` | Starts the 7 Nav2 nodes itself; adds the static `base_link -> zed_camera_link` | Jazzy's `navigation_launch.py` also starts route_server, collision_monitor and docking_server, which abort the bringup without their own config. |
| `nav2_params.yaml` | `bt_navigator`: built-in BT plugins no longer listed, `navigators` declared. `controller_server`: `progress_checker_plugins`. `behavior_server`: `local_/global_costmap_topic`, `local_frame`. Local costmap: `nvblox_layer` added. | Nav2 1.3 (Jazzy) parameter names. |
| Behavior trees | Rebuilt from Jazzy's default trees (BehaviorTree.CPP v4), Spin still removed | The v3 files do not load in Jazzy. |
| `kratos_perception` (new) | ZED 2i + ESS + nvblox launch and config | See below. |
| `tools/` (new) | `check_time_sync.py`, `setup_ptp.sh` (optional) | Checks every sensor is on the Orin's clock. |
| `MID360_config.json` | LiDAR IP `192.168.1.162` | The unit connected to the Orin. |

## One-time setup

```bash
# 1. Images (host). The kratos nvblox image first (~/kratos_nvblox), then this one.
~/kratos_nvblox/docker/build_image.sh
~/kratos_glim/docker/build_image.sh

# 2. Build the workspace (in the container)
~/kratos_glim/docker/run_container.sh docker/build_ws.sh

# 3. Optional: PTP master for the MID-360 (stamps without arrival jitter; see below)
sudo ~/kratos_glim/tools/setup_ptp.sh end0 --persist
```

**Time sync: already on the Orin's clock, PTP optional.** Without PTP/GPS sync,
`livox_ros_driver2` stamps every LiDAR/IMU packet with the Orin's clock when it arrives
(`GetEthPacketTimestamp` in `src/comm/pub_handler.cpp`), so GLIM's TF, the ZED images and
Nav2 share one clock. Measured on the Orin (`tools/check_time_sync.py`): `/livox/imu` 0.9 ms
old (max 7.5 ms), `/livox/lidar` 104 ms (a 10 Hz scan is stamped at its start). PTP
(`tools/setup_ptp.sh`) only removes the network/arrival jitter from the stamps, which slightly
helps GLIM's deskewing; it is not needed for nvblox or Nav2. Rerun `check_time_sync.py` after
any driver change: every topic must stay within a few tens of ms (the LiDAR ~100 ms).

The Orin's Ethernet port (`end0`) needs the address the driver config expects
(`192.168.1.50/24`). **The MID-360 on the Orin is `192.168.1.162`** (factory default
192.168.1.1XX from its serial number); the team VM's unit was `.125`. On this branch
`src/livox_ros_driver2/config/MID360_config.json` points at `.162`; change it back for `.125`.
The driver reprograms the LiDAR's destination (it was sending to `192.168.1.5`) at startup.

## Run: one command (over SSH from the laptop)

```bash
ssh <orin>
~/kratos_glim/start.sh                    # MID-360, GLIM, pcd2pgm, Nav2 + nvblox, ZED 2i + ESS
~/kratos_glim/start.sh depth:=zed         # ZED NEURAL depth instead of ESS
~/kratos_glim/start.sh depth:=none        # LiDAR only
~/kratos_glim/start.sh lidar_z:=0.62 cam_x:=0.35 cam_z:=0.45 cam_pitch:=0.30   # real mounts
~/kratos_glim/start.sh --no-follow ...    # start and return to the prompt
~/kratos_glim/logs.sh                     # show the log again (logs.sh 'glim|ess' filters)
~/kratos_glim/stop.sh                     # stop; saves GLIM's session; releases the ZED and GPU
```
- **The stack runs detached** in the `kratos_glim` container. Ctrl+C in `start.sh`/`logs.sh`, closing
  the terminal or a dropped SSH/Wi-Fi link only stops showing the log; the rover keeps running.
  Only `stop.sh` stops it. Logs: `~/kratos_glim/log/bringup/<date_time>.log` (`latest.log`).
- **Headless over SSH:** GUIs start only with a local X display (the Orin's desktop). Without one,
  RViz is not started and GLIM runs without its OpenGL viewer (it crashes at start without a
  display); `/glim_ros/map` is still published. Force with `gui:=true|false`. On the laptop:
  `rviz2 -d <repo>/src/kratos_bringup/rviz/laptop.rviz` (map, costmaps, TF, plan, odometry only).
- Keep the rover still for the first ~10 s (GLIM's IMU initialization).
- `start.sh` checks that the LiDAR answers and, unless `depth:=none`, that the ZED is plugged in and
  no other container is running it (only one process can open it). Then it starts the container,
  builds the workspace if needed and launches `kratos_bringup/launch/kratos.launch.py`.
- `start.sh` and `stop.sh` run `tools/zed_hid_rebind.sh`: the ZED SDK can leave the camera's HID
  interface (2b03:f881) detached from `usbhid` when it exits, and a container started then has
  no `/dev/hidraw` node for the ZED. The script rebinds it (`sudo -n`; it prints the command if
  sudo needs a password).
- `stop.sh` sends Ctrl+C to the launch; GLIM writes its session to `/tmp/dump`, which `stop.sh`
  moves to `~/kratos_glim/maps/<date_time>`.
- `/map` (pcd2pgm) appears after GLIM's first submap, which needs the rover to move. Until then
  the global costmap is a 5x5 m placeholder and warns "Sensor origin ... out of map bounds".
- On the Orin's desktop, RViz (`kratos_bringup/rviz/kratos.rviz`) shows everything, including
  the GLIM map, nvblox mesh, ESS depth and ZED color. "2D Goal Pose" sends a Nav2 goal.
- The mission (`waypoint_mission.py`) is started separately, in a
  `~/kratos_glim/docker/run_container.sh` shell (over SSH, inside `tmux` so a dropped link does
  not stop it). The stack ends at Nav2's `/cmd_vel`.

## Operating from the laptop (Ubiquiti link)

The container's ROS networking is in `~/kratos_glim/ros_network.env` (restart the stack after
editing it): `ROS_DOMAIN_ID` (must equal the laptop's), `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
(the laptop can see the rover), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and an optional
`ROS_STATIC_PEERS=<laptop IP>` for links that drop multicast (DDS discovery uses it).

On the laptop: same `ROS_DOMAIN_ID`, same RMW (Fast DDS), and, on Humble,
`ROS_LOCALHOST_ONLY` unset or 0 (on Jazzy: `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`). To call the
waypoint services it needs `waypoint_interfaces` built (it is in this repo), e.g.

```bash
ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
```

`/add_waypoint` tags the rover's pose at the moment GLIM handles the call, whoever calls it.

- **Humble laptop, Jazzy rover:** ROS does not officially support mixing distributions. Plain
  topics and services with unchanged message definitions (this one, `/cmd_vel`, TF, `/map`)
  generally work over Fast DDS; test before relying on it.
- **Bandwidth:** anything the laptop subscribes to crosses the radio. Avoid ZED images
  (1280x720 color is tens of MB/s), raw LiDAR clouds and the nvblox mesh in the laptop's RViz;
  `/map`, costmaps, TF, `/plan` and odometry are light.
- **Check from the laptop:** `ros2 node list` shows `/glim_ros`, `/bt_navigator`, `/nvblox_node`...;
  `ros2 service list | grep waypoint` shows the four waypoint services.

## Run: one terminal per component (each terminal: `~/kratos_glim/docker/run_container.sh`)

Order matters, as in `LIVE_MISSION_TEST.md`. Every shell in the container has ROS, the ZED
wrapper, this workspace and the GLIM fix overlay sourced already.

```bash
# T1 driver
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
# T2 angle filter
ros2 launch lidar_angle_filter angle_filter.launch.py
# T3 Nav2 + static TFs (LiDAR and camera mounts), BEFORE GLIM
ros2 launch kratos_nav nav.launch.py lidar_z:=0.60 cam_x:=0.30 cam_z:=0.45 cam_pitch:=0.30
# T4 GLIM (keep the rover still until "initial IMU state estimation result")
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/workspaces/kratos_glim/glim/glim_config
# T5 pcd2pgm live
ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file /workspaces/kratos_glim/src/pcd2pgm/config/pcd2pgm_live.yaml
# T6 camera -> nvblox (lidar_z MUST equal T3's)
ros2 launch kratos_perception perception.launch.py lidar_z:=0.60             # ESS depth
ros2 launch kratos_perception perception.launch.py lidar_z:=0.60 depth:=zed   # ZED NEURAL depth
# checks
python3 tools/check_time_sync.py
ros2 topic hz /nvblox_node/static_map_slice            # ~10 Hz
ros2 run tf2_ros tf2_echo odom zed_left_camera_frame_optical   # moves with the rover (wait ~5 s)
```

Then continue with the mission steps of `LIVE_MISSION_TEST.md` (waypoints,
`waypoint_mission.py`); they are unchanged.

## Invariants added

- **`lidar_z` is now in four places:** the three in `AGENTS.md`, plus
  `perception.launch.py lidar_z:=`. The launch file computes nvblox's obstacle band from it
  (`esdf_slice_min/max_height` = 0.2 / 1.8 m above ground = `0.2 - lidar_z` / `1.8 - lidar_z`
  in odom), so only the argument has to match.
- **The camera mount (`cam_*` in `nav.launch.py`) is a placeholder.** Measure it, or calibrate
  the ZED against the MID-360. A 1 deg pitch error moves the ground by 3.5 cm at 2 m, and
  ground that rises into the band becomes an obstacle.
- **The ZED must not track or publish TF** (`kratos_perception/config/zed2i_glim.yaml`). GLIM
  owns `map -> odom -> base_link`; a second publisher gives `zed_camera_link` two parents.
- **nvblox's global frame is `odom`, and `nvblox_layer` is in the local costmap only.** GLIM's
  loop closures move `map -> odom`, which would tear an nvblox map kept in `map`.
- **The ESS runtime comes from `~/kratos_nvblox/ess`** (node, TensorRT engines, venv), mounted
  into the container. `kratos_perception/config/ess_zed2i.yaml` is a copy of kratos_nvblox's.

## Known limits

- **Ramps:** nvblox marks every surface between 0.2 and 1.8 m above the *start* ground as an
  obstacle. A 10 deg ramp shows as a wall about 1.1 m up it (5 deg: 2.3 m). On a hilltop the
  ground itself can enter the band. nvblox 4.6 has `experimental_use_ground_plane_estimation`,
  but it fits a single plane, so it does not fix a ramp next to flat ground.
- **Ditches and drops are not detected** by nvblox, by the LiDAR obstacle layers
  (`min_obstacle_height` is ground + 0.2 m) or by pcd2pgm, and the planner has
  `allow_unknown: true`. A traversability or drop detector is needed for rough terrain.
- **GPU:** ESS (15 Hz, TTA on) plus nvblox at 3 cm. If the GPU saturates, set `tta: false` in
  `ess_zed2i.yaml` or use `depth:=zed`.

## Status (2026-09-24)

Verified on the Orin, in the `kratos_glim` container:
- Image `kratos/glim_nvblox` builds (GLIM 1.2.2 CPU, Livox-SDK2, patched glim_ros overlay);
  `ros2 pkg prefix glim_ros` is the overlay.
- Workspace builds (8 packages, Jazzy).
- Unit tests: waypoint_manager 13/13, lidar_angle_filter.
- `e2e_test.sh` (fake GLIM + real Nav2 Jazzy + waypoint_mission): passed. Nav2 active in 7 s,
  detour around the wall, re-sent goal after the fake loop closure, pending wp2 waited for,
  mission complete. `nvblox_layer` loaded in the local costmap.
- `mission_edge_test.sh` and the pcd2pgm live suite: passed.

- Full bring-up with the real MID-360 and ZED 2i on the Orin (bench, sensors not on the rover):
  - driver 10.0 Hz points / 200 Hz IMU, angle filter ~10 Hz, stamps on the Orin's clock
    without PTP (see above)
  - GLIM (CPU): IMU initialization OK, `/glim_ros/odom` 10 Hz, `map -> base_link` z = -0.64
    (= -lidar_z, as the mission runbook expects); its TF arrives ~130 ms after its stamp
  - Nav2: all 7 nodes active
  - ZED 2i with depth NONE and no tracking; camera 14.8 Hz; ESS ~11 Hz (41-43 ms/frame with
    TTA, ~85 % fill: it skips about a quarter of the frames; `tta: false` would keep up)
  - TF `odom -> zed_left_camera_frame_optical` resolves through GLIM + the camera mount
  - nvblox `/nvblox_node/static_map_slice` 9.4 Hz in `odom`; its obstacles reach
    `/local_costmap/costmap` (bench scene: 1383 lethal cells in the 6x6 m window)

- Headless start as over SSH (no DISPLAY, no terminal, `depth:=none`): `start.sh` returned and
  the stack kept running; GLIM initialized without its viewer, odometry 10 Hz, `/glim_ros/map`
  published, Nav2 active, no RViz; `stop.sh` saved GLIM's session to `maps/`.
- `start.sh` / `kratos.launch.py` with both sensors: every component up, no errors; all topics
  on one clock (GLIM TF ~120 ms behind, max 131 ms); load CPU 30-60 % per core, GPU 37 %,
  RAM 11 GB of 62 GB.

Not yet run: `depth:=zed`, and anything on the moving rover (the camera/LiDAR mount values are
still placeholders).
