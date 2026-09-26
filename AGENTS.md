# AGENTS.md

Guidance for AI coding agents (Codex, Claude Code, etc.) and new teammates
working in this repo.

## What this repo is

`auto_fastlio2` is Project Kratos's autonomy workspace (ROS 2 Humble, Ubuntu
22.04, Livox MID-360). The name is historical. **FAST-LIO2 is no longer used.**
The active stack is:

1. **GLIM** (apt `ros-humble-glim-ros` 1.2.2): SLAM and localization.
   - Config is in `glim/glim_config`.
   - The `/glim_ros/map` bug fix overlay is in `glim/glim_ros_fix`.
2. **waypoint_manager** (`glim/glim_ext_addon`): a GLIM extension. It stores
   waypoints relative to submaps, so they follow loop closure.
3. **pcd2pgm** (`src/pcd2pgm`), live mode: `/glim_ros/map` → 2D `/map`.
4. **kratos_nav** (`src/kratos_nav`):
   - Nav2 config
   - `waypoint_mission.py` (autonomous mission)
   - `rover_bridge.py` (`/cmd_vel` → wheel PWM)
5. Also used: `src/livox_ros_driver2` (driver) and `src/lidar_angle_filter`
   (masks the rover body out of the scan).

Legacy code, not used: `src/FAST_LIO`, and the FAST-LIO sections of the root
`README.md`.

**Out of scope:** the rover drive stack (`drive.py`, ESP32 firmware) is in a
separate repo (ARC_26) that this team doesn't own. Don't propose changes to it
here. Report drive-side problems as notes.

## Layout

| Path | Contents |
|---|---|
| `src/kratos_nav/` | Nav2 launch/params/BTs, mission and bridge scripts, hardware-free tests |
| `src/pcd2pgm/` | Fork of LihanChen2004/pcd2pgm plus live mode (`config/pcd2pgm_live.yaml`), standalone test suite in `test/` |
| `src/lidar_angle_filter/` | Front/back sector mask for the LiDAR (gtest) |
| `glim/glim_config/` | GLIM JSON config (`base_frame_id: base_link`, CPU modules) |
| `glim/glim_ext_addon/` | `waypoint_interfaces` (srvs), `waypoint_manager` (GLIM extension + gtest), `glim_dump_export` |
| `glim/glim_ros_fix/` | Patch for glim_ros 1.2.2 `rviz_viewer` (upstream koide3/glim_ros2#76), built as an overlay |
| `docs/` | Runbooks. Start at `docs/README.md` |

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select waypoint_interfaces waypoint_manager glim_dump_export pcd2pgm kratos_nav lidar_angle_filter \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon test --packages-select waypoint_manager lidar_angle_filter && colcon test-result --verbose
python3 src/kratos_nav/test/test_rover_bridge.py
python3 src/kratos_nav/test/test_lidar_ip.py
bash -n bringup.sh && ./bringup.sh --check   # pre-flight only, starts nothing
PCD2PGM_SETUP=$PWD/install/setup.bash bash src/pcd2pgm/test/run_all.sh
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/mission_edge_test.sh
```

- Building `livox_ros_driver2` needs the extra steps in the root `README.md`.
- The shell tests start ROS nodes on their own `ROS_DOMAIN_ID` (43, 44, 45).
  Run them one at a time.
- They `pkill` their own node names. Don't run them while a live test is
  running on the same machine.

## Invariants (breaking these fails silently in the field)

- **LiDAR height `lidar_z` is in three places and must match:**
  - `src/kratos_nav/launch/nav.launch.py` (arg `lidar_z`)
  - `src/kratos_nav/config/nav2_params.yaml` (`min/max_obstacle_height`, twice)
  - `src/pcd2pgm/config/pcd2pgm_live.yaml` (`thre_z_min/max`)

  The heights are relative to GLIM's `map` z=0, which is the LiDAR's start
  height, not the ground.
- **The TF chain is `map → odom → base_link → livox_frame`.**
  - GLIM publishes the first two links.
  - The static `base_link → livox_frame` comes from `nav.launch.py` (or a
    manual `static_transform_publisher`).
  - Without the static link, GLIM warns every frame and publishes no pose TF.
- **pcd2pgm live mode uses a fixed grid.**
  - Origin and size never change at runtime, because Nav2's static layer must
    not resize.
  - `/map` cells are only 0 or 100.
  - An empty filtered cloud keeps the previous `/map`. Never publish an
    all-free grid.
  - `/map` QoS is reliable + transient_local + depth 1.
- **The pcd2pgm radius filter must stay loose** (0.75 m / 2 neighbours).
  GLIM's map is voxelized at 0.5 m, and tighter values erase every wall.
- **The glim_ros overlay must be sourced last** in GLIM's terminal. Check with
  `ros2 pkg prefix glim_ros`.
- **waypoint_manager:**
  - Pending (not yet bound to a submap) waypoints are listed as
    `"<name> (pending)"`, are saved under `pending_waypoints`, and return
    `found=False` from `/get_waypoint`.
  - Callers that consume the list must strip the suffix (see
    `waypoint_mission.py`).
- **Only `rover_bridge.py` writes `/rover` during a mission.** `drive.py` is
  remapped to `/rover_joy`.
  - The bridge must keep publishing zeros on stale input and on shutdown.
- **ROS 2 Humble rclpy scripts:**
  - Use `SignalHandlerOptions.NO` plus explicit SIGINT/SIGTERM handlers, so
    cleanup (cancel goal, zero wheels) can still publish.
  - After `spin_until_future_complete` times out, poll `future.done()`.
    Don't rely on `add_done_callback`.
- **The VM is the runtime.** Its workspaces (`~/Kratos/*_ws`) hold copies of
  this code. Keep them in sync when you change anything here (the table is in
  `docs/LIVE_MISSION_TEST.md`).

## Review hints

- **Placeholders, not bugs (unless the reason is wrong):**
  - `lidar_z` 0.60
  - the 0.74 m square footprint
  - `track_width` 0.80
  - `max_wheel_speed` 1.0
  - the 50x50 m grid
- **Not verified on hardware:** `rover_bridge.py` and the full mission. The
  rest was verified on the VM with a real LiDAR (GLIM, waypoints, pcd2pgm) or
  with fakes (`fake_glim.py`).
- **Not tracked, never commit:** `build/`, `install/`, `log/`, `*.pcd`,
  `*.pgm`, `maps/`, GLIM dumps.
- `src/pcd2pgm/config/pcd2pgm.yaml` is upstream file mode with a teammate's
  path, and isn't used by the mission.
- **Writing docs:** keep them short and concrete, with commands that can be
  pasted. The VM paths in `docs/` are intentional.
