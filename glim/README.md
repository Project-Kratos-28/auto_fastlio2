# GLIM SLAM configuration for Livox MID-360

This directory contains the repository-local GLIM configuration and RViz layout. GLIM
itself is installed from the official apt repository and is not built by this ROS 2
workspace.

For first-time installation, driver building, and network setup, see the root
[`README.md`](../README.md).

## Contents

| Path | Purpose |
|---|---|
| `glim_config/` | GLIM configuration files; keep the complete directory together |
| `glim_ros.rviz` | RViz layout with the GLIM points, map, odometry, and TF displays |

## CPU and GPU selection

Set the following entries in `glim_config/config.json`:

| Setting | CPU | NVIDIA GPU |
|---|---|---|
| `config_odometry` | `config_odometry_cpu.json` | `config_odometry_gpu.json` |
| `config_sub_mapping` | `config_sub_mapping_passthrough.json` | `config_sub_mapping_gpu.json` |
| `config_global_mapping` | `config_global_mapping_pose_graph.json` | `config_global_mapping_gpu.json` |

The GPU GLIM apt package must match the locally installed CUDA toolkit. Use
`nvcc --version` to identify the toolkit version. No source rebuild is needed after
editing these JSON files because the commands below pass the configuration directory
directly to GLIM.

The GPU JSON files select CUDA computation. On an NVIDIA PRIME system, use the two
environment variables shown in the NVIDIA commands below to also force GLIM and RViz
window rendering onto NVIDIA OpenGL. CPU-only systems must use the unprefixed commands.

The current higher-detail GPU profile uses:

```text
config_preprocess.json:       random_downsample_target = 20000
config_sub_mapping_gpu.json:  submap_downsample_resolution = 0.05
config_sub_mapping_gpu.json:  submap_target_num_points = 100000
```

## Run live SLAM

Terminal 1 -- MID-360 driver, CPU/default launch:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Terminal 1 -- NVIDIA OpenGL for the raw-point RViz window:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Use `rviz_MID360_launch.py`; GLIM requires its `sensor_msgs/msg/PointCloud2` output.

Terminal 2 -- antenna angle filter:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lidar_angle_filter angle_filter.launch.py
```

This publishes `/livox/lidar_filtered` for GLIM and removes 30-degree-wide sectors
centered on the LiDAR +X and -X axes. Adjust `front_center_deg` in
`src/lidar_angle_filter/config/angle_filter.yaml` if LiDAR +X is not rover-forward.

Terminal 3 -- GLIM, CPU/default launch:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:="$(realpath "$REPO_ROOT/glim/glim_config")"
```

Terminal 3 -- GLIM with CUDA modules and NVIDIA OpenGL rendering:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:="$(realpath "$REPO_ROOT/glim/glim_config")"
```

Keep the rover completely stationary until GLIM prints
`initial IMU state estimation result`. Starting while moving can invalidate the initial
IMU bias, orientation, and velocity estimate. After initialization, move smoothly and
revisit mapped areas to support loop closure.

Terminal 4 -- preconfigured CPU/default visualization:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
rviz2 -d "$REPO_ROOT/glim/glim_ros.rviz"
```

Terminal 4 -- preconfigured RViz forced onto NVIDIA OpenGL:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
rviz2 -d "$REPO_ROOT/glim/glim_ros.rviz"
```

`/glim_ros/points` is the current registered scan. `/glim_ros/map` is the accumulated
map and updates approximately every 10 seconds.

## Validate the session

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/lidar_filtered
ros2 topic hz /livox/imu
ros2 topic hz /glim_ros/odom
ros2 topic echo /glim_ros/odom --once --field pose.pose
ros2 run tf2_ros tf2_echo map livox_frame
```

Expected nominal rates are approximately 10 Hz for both LiDAR topics, 200 Hz for
`/livox/imu`, and 8-10 Hz for `/glim_ros/odom`.

## Save the map

Press `Ctrl+C` once in the GLIM terminal and wait for the `saved` message. Preserve the
temporary dump immediately:

```bash
cd /path/to/auto_fastlio2
REPO_ROOT="$(git rev-parse --show-toplevel)"
MAP_PATH="$REPO_ROOT/maps/glim_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$REPO_ROOT/maps"
mv /tmp/dump "$MAP_PATH"
echo "Saved map: $MAP_PATH"
```

The dump contains the factor graph, numbered submaps, session configuration, odometry
trajectories, and globally optimized trajectories.

## Open and export a saved map

### Scriptable export (no GUI, recommended for scripting/pipelines)

The installed GLIM (1.2.2) `offline_viewer` has no command-line export flag — that
was added in a later GLIM version than what the apt PPA ships. `glim_dump_export`
(in `glim_ext_addon/`) is a small standalone tool that loads a dump the same way
`offline_viewer` does internally and writes straight to PCD, skipping the PLY step
entirely. Build it alongside `waypoint_manager` (see
[`glim_ext_addon/README.md`](glim_ext_addon/README.md)):

```bash
source /opt/ros/humble/setup.bash
source ~/glim_ext_ws/install/setup.bash
ros2 run glim_dump_export glim_dump_export /path/to/saved/glim_dump /path/to/map.pcd "$(realpath glim_config)"
```

This uses the dump's poses exactly as already optimized live during the run (same as
GUI export with `enable_optimization=false`) — it does not add new loop closures.
Use the GUI viewer below if you need to manually close a missed loop, run Bundle
Adjustment, or crop points before exporting.

### Manual export (GUI, for editing before export)


CPU/default viewer:

```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros offline_viewer --map_path /absolute/path/to/saved/glim_dump
```

CUDA-enabled map with its viewer forced onto NVIDIA OpenGL:

```bash
source /opt/ros/humble/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 run glim_ros offline_viewer --map_path /absolute/path/to/saved/glim_dump
```

Use `File -> Save -> Export Points` to export PLY. Convert it to PCD if required:

```bash
pcl_ply2pcd /path/to/map.ply /path/to/map.pcd
```

For manual point removal:

```bash
ros2 run glim_ros map_editor
```

Force the map editor onto NVIDIA OpenGL when an NVIDIA GPU is available:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 run glim_ros map_editor
```

Verify the renderer and monitor CUDA processes with:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
glxinfo -B | grep "OpenGL renderer"

watch -n 1 nvidia-smi
```

## Waypoints

`extension_modules` in `glim_config/config_ros.json` already loads
`libwaypoint_manager.so` — a GLIM extension module that tags waypoints relative to the
current submap's own origin, so a tagged point automatically rides along with every
loop-closure correction instead of going stale. Requires a one-time separate build; see
[`glim_ext_addon/README.md`](glim_ext_addon/README.md) for the build steps and the full
service list (`/add_waypoint`, `/get_waypoint`, `/save_waypoints`, `/list_waypoints`).

Single-session only for now — see that README's "Known limitations" for why, and see
[`WAYPOINT_AND_CONE_GUIDE.md`](../WAYPOINT_AND_CONE_GUIDE.md) for how this is meant to
fit into the broader semantic-landmark / cone system.
