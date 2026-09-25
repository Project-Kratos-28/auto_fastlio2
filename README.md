# Kratos rover autonomy (branch `jazzy-nvblox`)

LiDAR SLAM, waypoints and Nav2 for the Kratos rover, on a Jetson AGX Orin (JetPack 7.2,
Ubuntu 24.04, ROS 2 Jazzy), with a ZED 2i adding near-field obstacles. The stack ends at
Nav2's `/cmd_vel`; whatever drives the wheels subscribes to it and is not part of this repo.
(The repo name `auto_fastlio2` is historical; FAST-LIO is not used.)

```
Livox MID-360 ─► livox_ros_driver2 ─► /livox/lidar ─┬─► lidar_angle_filter ─► /livox/lidar_filtered ─► GLIM
                                   └► /livox/imu ───┼──────────────────────────────────────────────────► GLIM
                                                    └─► Nav2 obstacle_layer (local + global costmap)
GLIM (+ waypoint_manager) ─► TF map→odom→base_link, /glim_ros/odom, /glim_ros/map, waypoint services
/glim_ros/map ─► pcd2pgm (live) ─► /map ─► Nav2 global costmap
ZED 2i ─► ESS depth (or ZED NEURAL) ─► nvblox (odom frame) ─► /nvblox_node/static_map_slice ─► Nav2 local costmap
waypoint_mission.py ─► Nav2 navigate_to_pose / navigate_through_poses ─► /cmd_vel
```

- **GLIM** is the SLAM: pose (`map → odom → base_link`) and a 3D map, from the MID-360's points
  and IMU. It runs on the CPU; the GPU is kept for depth and nvblox.
- **pcd2pgm** turns GLIM's 3D map into a 2D `/map` (fixed 50×50 m grid, 5 cm) for the global costmap.
- **nvblox** covers what the LiDAR can't see: the MID-360's field of view is −7° to +52°, so at
  0.60 m height its lowest beam reaches the ground ~4.9 m out, and anything shorter than
  `0.60 − d·tan 7°` at distance `d` (a 30 cm rock at 2 m) is invisible to it. The ZED, tilted
  down, is fused by nvblox into a 3 cm map in GLIM's `odom` frame, whose 2D slice feeds the local
  costmap.
- **waypoint_manager** (a GLIM extension) tags waypoints relative to GLIM submaps, so they move
  with loop-closure corrections. **waypoint_mission.py** drives to them through Nav2.

Everything runs in one Docker container (`kratos_glim`), started by `start.sh`.

## Hardware and network

| Device | Connection | Address / notes |
|---|---|---|
| Livox MID-360 | Orin Ethernet `end0` (100 Mbit/s link, ~24 Mbit/s used) | LiDAR `192.168.1.162`, Orin `192.168.1.50/24` |
| ZED 2i | USB 3 | needs the udev rule below |
| Laptop | Ubiquiti link | same `ROS_DOMAIN_ID`; see [Laptop](#laptop) |

The LiDAR's IP is in `src/livox_ros_driver2/config/MID360_config.json` (`lidar_configs[0].ip`).
A MID-360's factory address is `192.168.1.1XX`, XX = the last two digits of its serial number.
The driver tells the LiDAR where to send data (`host_net_info`, `192.168.1.50`) at startup.

## Setup (once)

```bash
# 1. ZED udev rule (host): lets the SDK reach the camera's sensors
sudo cp ~/kratos_nvblox/docker/99-slabs.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger

# 2. Images: the kratos nvblox image (Isaac ROS 4.6, nvblox, Nav2, ZED SDK 5.4.1), then this one on top
~/kratos_nvblox/docker/build_image.sh
~/kratos_glim/docker/build_image.sh

# 3. Build the workspace (in the container; start.sh also does this if it was never built)
~/kratos_glim/docker/run_container.sh docker/build_ws.sh
```

`docker/Dockerfile.glim` adds GLIM 1.2.2 (CPU, koide3 PPA), Livox-SDK2, Nav2 and the patched
`glim_ros` overlay ([`glim/glim_ros_fix`](glim/glim_ros_fix/README.md)) to the kratos image.
ESS (TensorRT engines, node, Python venv) comes from `~/kratos_nvblox/ess`, mounted into the
container; the ZED SDK's optimized models and calibration from `~/kratos_nvblox/zed`.

## Run

```bash
~/kratos_glim/start.sh                      # everything, ESS depth
~/kratos_glim/start.sh depth:=zed           # ZED SDK NEURAL depth instead of ESS
~/kratos_glim/start.sh depth:=none          # LiDAR only (no camera, no nvblox)
~/kratos_glim/start.sh lidar_z:=0.62 cam_x:=0.35 cam_z:=0.45 cam_pitch:=0.30
~/kratos_glim/start.sh --no-follow ...      # start and return to the prompt
~/kratos_glim/logs.sh                       # follow the log again; logs.sh 'glim|ess' filters
~/kratos_glim/stop.sh                       # stop; files GLIM's session; frees the ZED and GPU
```

**Keep the rover still for the first ~10 s**: GLIM estimates the IMU state at start
(`initial IMU state estimation result` in the log). Moving then gives wrong IMU bias and orientation.

- The stack runs **detached** in the container. Ctrl+C in `start.sh`/`logs.sh`, closing the
  terminal or losing SSH only stops showing the log. Only `stop.sh` stops the stack.
- `start.sh` checks the LiDAR answers, and (unless `depth:=none`) that the ZED is plugged in and
  no other container runs it (only one process can open the camera). It also re-binds the ZED's
  HID interface to `usbhid` if a previous ZED SDK run left it detached (`tools/zed_hid_rebind.sh`):
  otherwise the container gets no `/dev/hidraw` node for the camera's sensors.
- `stop.sh` sends Ctrl+C to the launch (GLIM writes its session to `/tmp/dump`), moves the dump to
  `~/kratos_glim/maps/<date_time>`, stops the container and re-binds the ZED HID interface.
- Logs: `~/kratos_glim/log/bringup/<date_time>.log` (`latest.log` links to the newest).
- **GUIs start only with a local X display** (the Orin's desktop). Over SSH, RViz is not started
  and GLIM runs without its OpenGL viewer: GLIM crashes at start when the viewer cannot open a
  display. Force with `gui:=true|false`.

### Launch arguments (`src/kratos_bringup/launch/kratos.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `depth` | `ess` | `ess`, `zed` (SDK NEURAL) or `none` |
| `gui` | `auto` | `auto`: RViz + GLIM viewer only with a local display |
| `lidar_z` | `0.60` | MID-360 height above ground (m). **Placeholder** |
| `lidar_x` | `0.0` | MID-360 forward of `base_link` (m) |
| `cam_x`, `cam_y`, `cam_z` | `0.30`, `0.0`, `0.45` | ZED (`zed_camera_link`) position from `base_link` (m). **Placeholder** |
| `cam_pitch`, `cam_yaw` | `0.30`, `0.0` | ZED down-tilt and yaw (rad). **Placeholder** |

`base_link` is on the ground, under the rover's turning centre.

### What starts

| Order | Component | Output |
|---|---|---|
| 1 | `livox_ros_driver2` | `/livox/lidar` (PointCloud2, 10 Hz), `/livox/imu` (200 Hz) |
| 2 | `lidar_angle_filter` | `/livox/lidar_filtered`: ±15° front and back removed (antenna) |
| 3 | `kratos_nav/nav.launch.py` | static TF `base_link → livox_frame`, `base_link → zed_camera_link`; Nav2 |
| 4 | GLIM (3 s later, after the static TF) | TF `map → odom → base_link`, `/glim_ros/odom`, `/glim_ros/map`, waypoint services |
| 5 | `pcd2pgm` (live) | `/map` |
| 6 | `kratos_perception` | ZED 2i, ESS, nvblox → `/nvblox_node/static_map_slice` |
| 7 | RViz (`kratos_bringup/rviz/kratos.rviz`) | only with a GUI |

Nav2 is 7 nodes (controller, smoother, planner, behaviors, bt_navigator, waypoint_follower,
velocity_smoother) started directly: Jazzy's `navigation_launch.py` also starts route, collision
monitor and docking servers that abort the bringup without their own config. Controller and
behaviors publish `cmd_vel_nav`; the velocity smoother publishes the final `/cmd_vel`.

Measured on the Orin with both sensors: GLIM odometry 10 Hz, its TF ~120 ms behind real time;
ZED 15 Hz; ESS ~40 ms/frame with TTA (processes ~11–15 Hz), ~20 ms without (27–28 Hz); nvblox
slice ~9–12 Hz; CPU 30–60 % per core, GPU ~37 %, RAM 11 GB of 62 GB.

## Waypoints and missions

Waypoints are tagged at the rover's current pose while GLIM runs. Call the services from any
shell in the container (`~/kratos_glim/docker/run_container.sh`) or from the laptop:

```bash
ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
ros2 service call /list_waypoints waypoint_interfaces/srv/ListWaypoints
ros2 service call /get_waypoint waypoint_interfaces/srv/GetWaypoint "{name: wp1}"
ros2 service call /save_waypoints waypoint_interfaces/srv/SaveWaypoints "{path: /workspaces/kratos_glim/maps/waypoints.yaml}"
```

- Stop facing the direction you want to arrive in: the waypoint's heading is the goal heading.
- A new tag is **pending** until GLIM finishes its submap (~every 5 m of travel): `/get_waypoint`
  says `found=False`, `/list_waypoints` shows `wp1 (pending)`, loop closure doesn't move it yet.
  After the last tag, keep driving a few metres.
- `/add_waypoint` fails with `no odometry yet` until GLIM has processed a frame.
- GLIM autosaves waypoints to `/tmp/waypoints_autosave.yaml` every 10 s.
- **Waypoints exist only in the running GLIM session.** GLIM 1.2.2 cannot relocalize in a saved
  map, and saved waypoint files cannot be loaded back. Restarting the stack starts a new map at a
  new origin. Keep it running from mapping through the mission.
- Waypoint markers: `/glim_ros/waypoints` (MarkerArray).

Drive the waypoints:

```bash
ros2 run kratos_nav waypoint_mission.py                                  # all tagged, in tag order, one at a time
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp3']"
ros2 run kratos_nav waypoint_mission.py --ros-args -p mode:=through      # one route through all
```

- `mode:=pose` sends one `navigate_to_pose` goal per waypoint and stops at each.
  `mode:=through` sends one `navigate_through_poses` goal; a waypoint counts as passed within 0.7 m.
- Every 2 s it re-reads the waypoints from GLIM; if one still ahead moved more than 0.3 m (loop
  closure), it re-sends the goal (through: with only the waypoints not yet passed).
- It waits up to 30 s for a pending waypoint. Ctrl+C cancels the Nav2 goal.
- Over SSH, run it inside `tmux`: a plain SSH session's processes are killed (SIGHUP) when the
  link drops. Parameters: [`src/kratos_nav/README.md`](src/kratos_nav/README.md).

Test-day procedure with checks and troubleshooting: [`docs/FIELD_TEST.md`](docs/FIELD_TEST.md).

## Configuration

### Heights: `lidar_z`

GLIM's `map`/`odom` origin is the LiDAR's **start** pose, so the ground is at `z = −lidar_z`.
Every obstacle height band is 0.2 m to 1.8 m above ground, i.e. `0.2 − lidar_z` to `1.8 − lidar_z`
(−0.40 / 1.20 at `lidar_z` 0.60). The value appears in four places; change them together:

| Where | What |
|---|---|
| `start.sh lidar_z:=` (→ `nav.launch.py`, `perception.launch.py`) | static TF; nvblox slice band (computed) |
| `src/kratos_nav/config/nav2_params.yaml` | `min_obstacle_height` / `max_obstacle_height`, both costmaps |
| `src/pcd2pgm/config/pcd2pgm_live.yaml` | `thre_z_min` / `thre_z_max` |

Symptom of a wrong value: the floor shows as obstacles, or low obstacles are missing.

### Camera mount

`cam_*` must be measured, better calibrated against the LiDAR. A 1° pitch error moves the ground
by 3.5 cm at 2 m; ground that rises into the band becomes an obstacle.

### Other settings

| File | Setting |
|---|---|
| `nav2_params.yaml` | footprint 0.74 × 0.74 m (**placeholder**), Smac Hybrid-A* (Dubins, min. turn radius 0.6 m), Regulated Pure Pursuit 0.4 m/s without rotate-in-place, local costmap 6×6 m (LiDAR + nvblox), global costmap `/map` + LiDAR |
| `behavior_trees/*_no_spin.xml` | Jazzy default trees without Spin (the rover can't turn in place) |
| `pcd2pgm_live.yaml` | fixed grid origin (−25, −25), 50×50 m, 0.05 m: size it to the whole arena; points outside are dropped. Radius filter 0.75 m / 2 neighbours (GLIM's map is voxelized at 0.5 m; tighter values erase walls) |
| `lidar_angle_filter/config/angle_filter.yaml` | masked sectors (full widths), `front_center_deg` if LiDAR +X isn't rover-forward |
| `glim/glim_config/config.json` | CPU modules (odometry CPU, sub-mapping passthrough, pose-graph global mapping). `config_global_mapping_pose_graph.json`: `min_travel_dist` 8 m (stock 50 m never closes loops in small areas) |
| `glim/glim_config/config_ros.json` | `base_frame_id: base_link`, topics, extension modules (`libwaypoint_manager.so`) |
| `kratos_perception/config/zed2i_glim.yaml` | ZED: HD720, 15 Hz published, **tracking and TF off** (GLIM owns the pose) |
| `kratos_perception/config/nvblox_glim.yaml` | nvblox: `odom`, 3 cm voxels, no color, 5 m integration, clears > 8 m from `base_link` |
| `kratos_perception/config/ess_zed2i.yaml` | ESS: contrast stretch, TTA (vertical-flip agreement), confidence 0.1. `tta: false` doubles the rate |

## Maps (GLIM sessions)

`stop.sh` files each session under `~/kratos_glim/maps/<date_time>/` (ignored by git): numbered
submaps, `graph.bin`, `values.bin`, `odom_*.txt` (trajectory before loop closure), `traj_*.txt`
(after), and the config used. Tools, in the container:

```bash
ros2 run glim_ros offline_viewer --map_path /workspaces/kratos_glim/maps/<date_time>   # needs a display
ros2 run glim_ros map_editor                                                           # needs a display
ros2 run glim_dump_export glim_dump_export /workspaces/kratos_glim/maps/<date_time> map.pcd /workspaces/kratos_glim/glim/glim_config
```

- `offline_viewer`: add missed loop closures (right-click a submap sphere → `Loop begin`, another
  → `Loop end`), plane bundle adjustment, re-optimize, `File → Save → Export Points` (PLY).
- `glim_dump_export`: dump → PCD without a GUI, using the poses as optimized live.
- A Nav2 map file from a PCD: `pcd2pgm_node` in file mode (`src/pcd2pgm/config/pcd2pgm.yaml`,
  set `pcd_file`), then `ros2 run nav2_map_server map_saver_cli -f <name>`.

## Laptop

`~/kratos_glim/ros_network.env` sets the container's ROS networking (restart the stack after editing):

| Variable | Value | Why |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | must equal the laptop's |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` | the laptop can see the rover |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | same DDS on both sides |
| `ROS_STATIC_PEERS` | (commented) | laptop IP, if the radio drops the multicast DDS uses for discovery |

On the laptop: same domain ID and RMW; Humble: `ROS_LOCALHOST_ONLY` unset; Jazzy:
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`. It needs `waypoint_interfaces` (this repo) to call the
waypoint services. View the rover with `rviz2 -d src/kratos_bringup/rviz/laptop.rviz`: only `/map`,
costmaps, TF, `/plan` and odometry, which are light over the radio. Point clouds, images and the
nvblox mesh are not (1280×720 color is tens of MB/s). "2D Goal Pose" sends a Nav2 goal.
Humble ↔ Jazzy is not officially supported by ROS; standard messages and these services work over
Fast DDS, but test before relying on it.

Check from the laptop: `ros2 node list` (shows `/glim_ros`, `/bt_navigator`, `/nvblox_node`, ...),
`ros2 service list | grep waypoint`.

Clocks: without PTP, the Livox driver stamps each packet with the Orin's clock when it arrives
(`GetEthPacketTimestamp`, `src/comm/pub_handler.cpp`), so all sensors share one clock.
`python3 tools/check_time_sync.py` shows every sensor's offset (LiDAR ~100 ms: a scan is stamped
at its start). `tools/setup_ptp.sh <iface>` makes the Orin a PTP master for the LiDAR; optional,
it only removes arrival jitter.

## Tests

In the container (`~/kratos_glim/docker/run_container.sh`), from `/workspaces/kratos_glim`:

```bash
colcon test --packages-select waypoint_manager lidar_angle_filter && colcon test-result --all
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh                  # fake GLIM + real Nav2 + mission
MISSION_ARGS="-p mode:=through" KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/mission_edge_test.sh         # late goal accept, Ctrl+C
PCD2PGM_SETUP=$PWD/install/setup.bash bash src/pcd2pgm/test/run_all.sh                     # pcd2pgm live mode
```

The shell tests use their own `ROS_DOMAIN_ID` (43–45) and `pkill` their node names: run them one at
a time, not during a live run. Details: [`src/kratos_nav/test/README.md`](src/kratos_nav/test/README.md),
[`src/pcd2pgm/test/README.md`](src/pcd2pgm/test/README.md).

## Known limits

- **Ramps:** nvblox (and the LiDAR layers) mark every surface 0.2–1.8 m above the *start* ground as
  an obstacle. A 10° ramp becomes a wall ~1.1 m up it (5°: 2.3 m); on a hill, ground can enter the band.
- **Ditches and drops are not detected** by any layer, and the planner allows unknown space
  (`allow_unknown: true`).
- **No relocalization:** waypoints and the map live only in the running GLIM session.
- **GLIM aborts on full LiDAR occlusion** (e.g. a hand over the sensor).
- **Fixed 50×50 m grid** around GLIM's start: size it to the arena.
- **ESS with TTA skips ~¼ of the camera's frames**; enough for the rover's speed.
- **Placeholders:** `lidar_z`, camera mount, footprint, grid size, turning radius.
- **Not tested on the moving rover yet.** Tested on the Orin: all of the above bring-up with both
  sensors (bench), and the hardware-free tests.

## Repository layout

| Path | Contents |
|---|---|
| `start.sh`, `stop.sh`, `logs.sh`, `ros_network.env` | run the stack from the host |
| `docker/` | image (`Dockerfile.glim`, `build_image.sh`), container (`run_container.sh`), workspace build (`build_ws.sh`) |
| `tools/` | `check_time_sync.py`, `zed_hid_rebind.sh`, `setup_ptp.sh` |
| `src/kratos_bringup/` | `kratos.launch.py` (whole stack), RViz layouts (`kratos.rviz`, `laptop.rviz`) |
| `src/kratos_perception/` | ZED 2i + ESS + nvblox launch and config |
| `src/kratos_nav/` | Nav2 launch, params, behavior trees, `waypoint_mission.py`, tests |
| `src/pcd2pgm/` | point cloud → OccupancyGrid; live mode on a fixed grid |
| `src/lidar_angle_filter/` | front/back sector mask |
| `src/livox_ros_driver2/` | Livox driver (MID-360 only) |
| `glim/glim_config/` | GLIM configuration (keep the directory together) |
| `glim/glim_ext_addon/` | `waypoint_manager`, `waypoint_interfaces`, `glim_dump_export` |
| `glim/glim_ros_fix/` | patch for GLIM 1.2.2's `/glim_ros/map` bug (built into the image) |
| `docs/FIELD_TEST.md` | test-day runbook |
| `WAYPOINT_AND_CONE_GUIDE.md` | design proposal: semantic landmarks and cones for IRC 2026 (not implemented) |
| `AGENTS.md` | orientation for AI agents and reviewers |
