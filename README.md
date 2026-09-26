# Livox MID-360 GLIM SLAM + Nav2 Workspace

ROS 2 Humble workspace for Project Kratos rover autonomy with a Livox MID-360.
It covers real-time LiDAR-inertial SLAM with GLIM, submap-relative waypoints,
a live 2D occupancy grid, and Nav2 navigation to the tagged waypoints.
(The repo name is historical. FAST-LIO2 is still in `src/FAST_LIO` but is not
part of the active pipeline.)

> **Start here for the autonomous mission:** [`docs/README.md`](docs/README.md).
> It covers the mission, the runbooks, and where every piece lives.
> **AI assistants / code reviewers:** read [`AGENTS.md`](AGENTS.md) first.

The SLAM pipeline (sections 1-5 below) is:

```text
                                 /livox/lidar
Livox MID-360 -> livox_ros_driver2 -> angular filter -> /livox/lidar_filtered -> GLIM
                         \--------> /livox/imu -------------------------------> GLIM
                                                                              |
                                                              odometry + 3D map
```

GLIM performs continuous pose estimation while it builds and optimizes the map. The
autonomous mission builds on that live session:

```text
GLIM (+ waypoint_manager) --/glim_ros/map--> pcd2pgm --/map--> Nav2 (kratos_nav) --/cmd_vel-->
  |  TF map->odom->base_link                                   ^
  +-- /get_waypoint <-- waypoint_mission.py --NavigateToPose---+
```

Map and tag waypoints by hand, close the loop, then let Nav2 drive to each waypoint in
the same GLIM session. See section 6 and [`docs/LIVE_MISSION_TEST.md`](docs/LIVE_MISSION_TEST.md).

This repository does **not** provide localization against a previously saved map
(GLIM 1.2.2 has no such mode). Waypoints are only valid within the session that
tagged them. Loading a saved GLIM dump in the offline viewer is for visualization,
editing, and export; it does not start live localization.

## 1. Getting started and workspace setup

### 1.1 Supported platform

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Livox MID-360 with its integrated IMU
- Ethernet connection to the LiDAR
- Optional NVIDIA GPU and a GLIM-supported CUDA toolkit

Commands below determine the repository location dynamically. They do not depend on a
particular username, home directory, Ethernet interface name, or clone location.

### 1.2 Install ROS 2 Humble and build tools

If ROS 2 Humble is not already installed, configure the official ROS 2 apt repository.
These commands follow the maintained
[ROS 2 Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html):

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' | awk -F'"' '{print $4}')"
ROS_UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${ROS_UBUNTU_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake \
  git \
  libpcl-dev \
  libeigen3-dev \
  mesa-utils \
  pcl-tools
```

Initialize `rosdep` once per computer:

```bash
sudo rosdep init
rosdep update
```

If `rosdep` was initialized previously, the first command may report that its sources
file already exists; continue with `rosdep update`.

### 1.3 Install Livox-SDK2

`livox_ros_driver2` requires Livox-SDK2 to be installed system-wide:

```bash
DEPENDENCY_DIR="$(mktemp -d)"
git clone https://github.com/Livox-SDK/Livox-SDK2.git "$DEPENDENCY_DIR/Livox-SDK2"
cmake -S "$DEPENDENCY_DIR/Livox-SDK2" -B "$DEPENDENCY_DIR/Livox-SDK2/build"
cmake --build "$DEPENDENCY_DIR/Livox-SDK2/build" --parallel
sudo cmake --install "$DEPENDENCY_DIR/Livox-SDK2/build"
sudo ldconfig
```

### 1.4 Install GLIM

Add the official GLIM Ubuntu 22.04 package repository:

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" \
  | gpg --dearmor \
  | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null

echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" \
  | sudo tee /etc/apt/sources.list.d/koide3_ppa.list >/dev/null

sudo apt update
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev
```

Choose exactly one GLIM installation.

CPU-only:

```bash
sudo apt install -y libgtsam-points-dev ros-humble-glim-ros
sudo ldconfig
```

NVIDIA GPU:

1. Install a GLIM-supported CUDA toolkit using the
   [NVIDIA CUDA installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/).
2. Confirm the installed toolkit version:

   ```bash
   nvcc --version
   ```

3. Install GLIM packages matching that CUDA version. For example, for CUDA 13.1:

   ```bash
   sudo apt install -y \
     libgtsam-points-cuda13.1-dev \
     ros-humble-glim-ros-cuda13.1
   sudo ldconfig
   ```

Replace `cuda13.1` in both package names with the available suffix matching the local
toolkit, such as `cuda12.2` or `cuda12.6`. See the
[official GLIM installation page](https://koide3.github.io/glim/installation.html) for
the currently published combinations.

Verify the installation:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg executables glim_ros
```

The output should include `glim_rosnode`, `offline_viewer`, and `map_editor`.

### 1.5 Select CPU or GPU configuration

Set these entries under `global` in `glim/glim_config/config.json`.

| Setting | CPU | NVIDIA GPU |
|---|---|---|
| `config_odometry` | `config_odometry_cpu.json` | `config_odometry_gpu.json` |
| `config_sub_mapping` | `config_sub_mapping_passthrough.json` | `config_sub_mapping_gpu.json` |
| `config_global_mapping` | `config_global_mapping_pose_graph.json` | `config_global_mapping_gpu.json` |

No source rebuild is required after editing these repository-local GLIM JSON files. The
launch command below passes their directory directly to GLIM.

On systems using NVIDIA PRIME in `on-demand` mode, selecting the GPU JSON files enables
CUDA computation but does not necessarily make the GLIM or RViz window use NVIDIA
OpenGL. Prefix GUI commands with both variables below to force NVIDIA rendering:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
glxinfo -B | grep "OpenGL renderer"
```

The result should name the NVIDIA GPU rather than `llvmpipe`. These variables are only
for NVIDIA systems. CPU-only systems must use the unprefixed commands.

### 1.6 Clone and build the ROS packages

```bash
git clone https://github.com/Project-Kratos-28/auto_fastlio2.git
cd auto_fastlio2
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select livox_ros_driver2 lidar_angle_filter \
    waypoint_interfaces waypoint_manager glim_dump_export pcd2pgm kratos_nav \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

GLIM itself is installed system-wide and is not built here. The
`-DROS_EDITION`/`-DDISTRO_ROS` flags are required by `livox_ros_driver2` on the first
configure.

**Also build the GLIM 1.2.2 bug-fix overlay** if pcd2pgm/Nav2 will consume
`/glim_ros/map`. Without it, `/map` can get phantom walls. Follow
[`glim/glim_ros_fix/README.md`](glim/glim_ros_fix/README.md).

### 1.7 Configure the MID-360 network

The checked-in driver configuration expects:

| Device | IPv4 address |
|---|---|
| Computer Ethernet adapter | `192.168.1.10/24` |
| MID-360 | `192.168.1.162` |

Configure the wired adapter with the desktop network settings. No particular network
connection profile or interface name is required. Confirm the resulting address and
sensor connectivity:

```bash
ip -brief address
ping -c 3 192.168.1.162
```

The MID-360's own address differs per unit (192.168.1.1xx, from its serial).
`bringup.sh` and `ros2 launch kratos_nav livox_driver.launch.py` find it
automatically (set `LIVOX_LIDAR_IP` to force one). Only the host address fields
(`*_ip` in `host_net_info`) in `src/livox_ros_driver2/config/MID360_config.json`
must be this computer's address; `bringup.sh` checks that. The stock
`rviz_MID360_launch.py` still uses the sensor `ip` written in that file. If a
computer uses a different address, update all host address fields before
building the driver again.

## 2. Create a map with GLIM

Use four terminals. In every terminal, change to the cloned repository first; the
commands do not assume where it was cloned.

### 2.1 Start the MID-360 driver

Terminal 1, CPU/default launch:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Terminal 1, NVIDIA launch with the driver's raw-point RViz forced onto the GPU:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

The Livox driver does not perform CUDA processing; the NVIDIA prefix accelerates the
RViz window started by this launch file.

Use `rviz_MID360_launch.py`, not `msg_MID360_launch.py`. GLIM consumes
`sensor_msgs/msg/PointCloud2`; the `msg_` launch publishes Livox `CustomMsg` data.

Before starting GLIM, confirm both streams:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

Expected rates are approximately 10 Hz for LiDAR frames and 200 Hz for IMU data.

### 2.2 Start the antenna angle filter

Terminal 2:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lidar_angle_filter angle_filter.launch.py
```

The filter reads `/livox/lidar` and publishes `/livox/lidar_filtered`, which is the
topic configured as GLIM's input. It removes 30-degree-wide sectors centered on the
LiDAR +X axis (front) and -X axis (back): -15 to +15 degrees and 165 to 180 / -180 to
-165 degrees. All fields and per-point timestamps are preserved.

The angle-filter settings are in `src/lidar_angle_filter/config/angle_filter.yaml`:

| Parameter | Default | Meaning |
|---|---:|---|
| `input_topic` | `/livox/lidar` | Raw `PointCloud2` input from the Livox driver |
| `output_topic` | `/livox/lidar_filtered` | Filtered cloud consumed by GLIM |
| `front_center_deg` | `0.0` | Rover-forward direction in the LiDAR XY plane; 0° is +X and positive rotation is toward +Y |
| `front_sector_deg` | `30.0` | Full width of the excluded front sector |
| `back_sector_deg` | `30.0` | Full width of the excluded rear sector, centered 180° from the front |

The sector values are full widths, not half-angles. To exclude 30 degrees on each side
of an axis (a 60-degree-wide sector), set the corresponding value to `60.0`. Set a
sector to `0.0` to disable it. If LiDAR +X does not point toward the rover's front,
change `front_center_deg` to the rover-forward azimuth in the LiDAR frame.

This is an all-range azimuth mask: valid environmental returns in those directions are
also removed. It prevents antenna returns from entering new GLIM maps, but it does not
alter maps that were saved before the filter was enabled.

Confirm the filtered stream before starting GLIM:

```bash
ros2 topic hz /livox/lidar_filtered
ros2 topic info /livox/lidar_filtered --verbose
```

The topic rate should remain approximately 10 Hz. After GLIM starts, the verbose topic
information should list `glim_rosnode` as a subscriber.

### 2.3 Start GLIM SLAM

Stop the rover and keep it completely motionless before running this command.

`glim/glim_config/config_ros.json` uses `base_frame_id: base_link`. GLIM needs the
static transform `base_link -> livox_frame` to publish its pose TF. Without it, GLIM
warns `Failed to lookup transform` every frame and Nav2 has no pose. Start it first:

- For the full mission, `ros2 launch kratos_nav nav.launch.py` provides it.
- For SLAM only, run this in its own terminal first:

  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run tf2_ros static_transform_publisher --z 0.60 --frame-id base_link --child-frame-id livox_frame
  ```

  0.60 m is the placeholder LiDAR height above the ground. Keep it equal to
  `lidar_z` in `src/kratos_nav/launch/nav.launch.py`.

The config also loads `libwaypoint_manager.so`, so `source install/setup.bash` in this
terminal. If you built the `glim_ros_fix` overlay, source its `local_setup.bash` last.

Terminal 3, CPU/default launch:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:="$(realpath "$REPO_ROOT/glim/glim_config")"
```

Terminal 3, NVIDIA launch with CUDA configuration and forced NVIDIA OpenGL:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:="$(realpath "$REPO_ROOT/glim/glim_config")"
```

This GPU command requires the three GPU JSON entries from section 1.5. The JSON files
enable CUDA odometry and mapping; the environment variables accelerate GLIM's standard
viewer with NVIDIA OpenGL.

Keep the rover still until GLIM prints `initial IMU state estimation result`, normally
after two to five seconds. Starting while the rover is moving can produce incorrect
IMU bias, orientation, and velocity estimates. Move only after initialization has
completed.

The one-time messages about large point timestamps and Livox `FLOAT64` nanoseconds are
expected when automatic MID-360 timestamp detection is enabled.

### 2.4 Visualize the live map

Terminal 4, CPU/default visualization:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
rviz2 -d "$REPO_ROOT/glim/glim_ros.rviz"
```

Terminal 4, RViz forced onto NVIDIA OpenGL:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
rviz2 -d "$REPO_ROOT/glim/glim_ros.rviz"
```

The supplied RViz layout already contains the required GLIM displays:

- `/glim_ros/points`: current registered LiDAR scan
- `/glim_ros/map`: accumulated optimized map; updated approximately every 10 seconds
- `/glim_ros/odom`: current LiDAR-inertial pose

The driver launch also opens its own RViz window for the raw point cloud. Use the GLIM
RViz window or GLIM's standard viewer to inspect the accumulated map.

### 2.5 Record the area

After IMU initialization:

1. Drive slowly and smoothly through the area.
2. Avoid abrupt acceleration, impacts, wheel vibration, and movement of the LiDAR mount.
3. Keep nearby surfaces in view and overlap adjacent passes.
4. Revisit previously mapped areas and return near the starting position to provide
   useful loop-closure opportunities.
5. Do not disconnect Ethernet or interrupt the IMU stream while mapping.

The current GPU profile retains more detail than the original defaults:

| File | Parameter | Value |
|---|---|---:|
| `config_preprocess.json` | `random_downsample_target` | `20000` |
| `config_sub_mapping_gpu.json` | `submap_downsample_resolution` | `0.05 m` |
| `config_sub_mapping_gpu.json` | `submap_target_num_points` | `100000` |

This profile uses more GPU memory, system memory, and storage. Reducing the resolution
below 0.05 m generally increases noise and processing cost substantially.

## 3. Verify live SLAM and localization

GLIM localization is the pose estimate produced during the active SLAM session. Check
the odometry rate:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /glim_ros/odom
```

It should normally be close to the LiDAR frame rate. Inspect one pose:

```bash
ros2 topic echo /glim_ros/odom --once --field pose.pose
```

For a physical motion check:

1. Leave the rover stationary and record the pose above.
2. Move forward by approximately 1-2 m and rotate 30-45 degrees.
3. Stop the rover and record the pose again.
4. Confirm that the position and orientation changed consistently with the motion.

Inspect the complete GLIM transform chain:

```bash
ros2 run tf2_ros tf2_echo map base_link     # published by GLIM
ros2 run tf2_ros tf2_echo map livox_frame   # via the static base_link -> livox_frame TF
```

If the driver frame was changed from `livox_frame`, substitute its configured frame ID.

Useful live publishers can be confirmed without modifying RViz:

```bash
ros2 topic info /livox/lidar_filtered --verbose
ros2 topic info /glim_ros/points --verbose
ros2 topic info /glim_ros/map --verbose
```

## 4. Stop and save a map

In Terminal 3, press `Ctrl+C` once. Wait for GLIM to print `saved` before closing the
terminal or stopping the driver. GLIM writes the completed graph, submaps,
configuration, and trajectories to `/tmp/dump`.

`/tmp/dump` is temporary, is replaced by a later GLIM run, and may be removed during a
reboot. Move it immediately to a permanent directory:

```bash
cd /path/to/auto_fastlio2
REPO_ROOT="$(git rev-parse --show-toplevel)"
MAP_PATH="$REPO_ROOT/maps/glim_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$REPO_ROOT/maps"
mv /tmp/dump "$MAP_PATH"
echo "Saved map: $MAP_PATH"
```

The `maps/` directory is ignored by Git because GLIM dumps can be large. Important dump
contents include:

- numbered submap directories such as `000000/`
- `graph.bin` and `values.bin`
- `odom_imu.txt` and `odom_lidar.txt`: trajectories before global correction
- `traj_imu.txt` and `traj_lidar.txt`: optimized trajectories after global correction
- the exact configuration used for the session under `config/`

## 5. Open, edit, and export a saved map

Open a saved dump directly on a CPU-only system:

```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros offline_viewer --map_path /absolute/path/to/saved/glim_dump
```

Open it with CUDA computation and NVIDIA OpenGL rendering:

```bash
source /opt/ros/humble/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 run glim_ros offline_viewer --map_path /absolute/path/to/saved/glim_dump
```

An offline viewer uses the configuration saved inside the dump. A map recorded with
the GPU configuration loads CUDA mapping modules; the NVIDIA environment variables
force its window onto the discrete GPU.

Alternatively, launch `ros2 run glim_ros offline_viewer`, then select
`File -> Open Map` and choose the dump directory.

The offline viewer can optimize explicit constraints:

- Loop closure: right-click one submap sphere and select `Loop begin`; select another
  sphere and choose `Loop end`; align the clouds and create the factor.
- Plane adjustment: right-click a point on a flat surface, choose
  `Bundle Adjustment (Plane)`, set the selection radius, and create the factor.

Remove unwanted map points on a CPU-only system with:

```bash
ros2 run glim_ros map_editor
```

On an NVIDIA system, force the editor onto NVIDIA OpenGL:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 run glim_ros map_editor
```

While a CUDA-enabled GLIM process or offline viewer is running, confirm NVIDIA usage
with:

```bash
watch -n 1 nvidia-smi
```

Export the map from `File -> Save -> Export Points`. GLIM exports PLY. Convert it to PCD
when another component requires PCD:

```bash
pcl_ply2pcd /path/to/map.ply /path/to/map.pcd
```

## 6. Live 2D map and autonomous navigation

These are the pieces added on top of live GLIM for the competition mission. The full
terminal-by-terminal bring-up, checks and troubleshooting are in
[`docs/LIVE_MISSION_TEST.md`](docs/LIVE_MISSION_TEST.md).

| Piece | Package / path | What it does |
|---|---|---|
| Waypoints | `glim/glim_ext_addon/waypoint_manager` | GLIM extension: `/add_waypoint`, `/get_waypoint`, `/list_waypoints`, `/save_waypoints`. Poses ride along with loop closure |
| Live 2D map | `src/pcd2pgm` (live mode) | `/glim_ros/map` -> filters -> fixed 50x50 m, 0.05 m `/map` (`OccupancyGrid`, transient_local) |
| Navigation | `src/kratos_nav` | `nav.launch.py`: Nav2 without map_server/AMCL plus the static `base_link -> livox_frame` TF. `waypoint_mission.py`: drives to each GLIM waypoint in order |
| Wheel interface | `src/kratos_nav/scripts/rover_bridge.py` | `/cmd_vel` -> the rover's `/rover` PWM topic, with joystick passthrough. **Untested on hardware** |
| GLIM bug fix | `glim/glim_ros_fix` | Patch + overlay for the GLIM 1.2.2 `/glim_ros/map` corruption |
| Bring-up | `bringup.sh` (repo root) | Starts and supervises the whole stack in one terminal (below) |
| LiDAR driver launch | `src/kratos_nav/launch/livox_driver.launch.py` | Finds the MID-360's IP (it differs per unit), then starts `livox_ros_driver2` |

**One command (what the GUI button runs):** from the repo root,

```bash
./bringup.sh              # --help for options: --lidar-z, --no-rviz, --check, ...
```

It first checks the setup: packages built, patched GLIM overlay, `lidar_z` consistent
in all three files, this computer has 192.168.1.10, no leftover stack processes. Then
it finds the MID-360 and starts driver -> angle filter -> Nav2 -> GLIM -> pcd2pgm ->
`rover_bridge` (MANUAL) -> RViz, each only after the previous one is verified. While
running it prints a status line every 15 s and an ALERT when something breaks, and
restarts crashed stateless nodes (never GLIM, which holds the waypoints). Ctrl+C saves
the waypoints to the log folder and stops everything in reverse order. Logs, plus
`state` and `health` files for the GUI, are in `~/kratos_logs/latest/`.
`./bringup.sh --check` runs only the checks.

The manual equivalent (each line in its own terminal, after `source install/setup.bash`):

```bash
ros2 launch kratos_nav livox_driver.launch.py      # finds the MID-360 IP
ros2 launch lidar_angle_filter angle_filter.launch.py
ros2 launch kratos_nav nav.launch.py                 # BEFORE GLIM (static TF)
# GLIM (section 2.3; source the glim_ros_fix overlay last)
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  --params-file "$(ros2 pkg prefix pcd2pgm)/share/pcd2pgm/config/pcd2pgm_live.yaml"
# ...drive and tag:  ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
```

**Placeholders to measure on the rover before trusting any of this:**

- `lidar_z` (0.60 m). It appears in three files: `nav.launch.py`, `nav2_params.yaml`
  and `pcd2pgm_live.yaml`.
- The robot footprint in `nav2_params.yaml`.
- `track_width` / `max_wheel_speed` in `rover_bridge.py`.

pcd2pgm has **no command-line converter**. To get a Nav2 map file from a saved
session, export the dump to PCD (section 5), run `pcd2pgm_node` in file mode
(`config/pcd2pgm.yaml`, set `pcd_file`), and save `/map` with
`ros2 run nav2_map_server map_saver_cli -f <name>`. See
[`src/pcd2pgm/README.md`](src/pcd2pgm/README.md).

## 7. Repository layout

| Path | Purpose |
|---|---|
| `glim/glim_config/` | MID-360 GLIM CPU and GPU configuration files |
| `glim/glim_ros.rviz` | Preconfigured live GLIM RViz layout |
| `glim/README.md` | Compact GLIM command reference and configuration notes |
| `glim/glim_ext_addon/` | `waypoint_manager` GLIM extension, `waypoint_interfaces`, `glim_dump_export` |
| `glim/glim_ros_fix/` | Patch + overlay build for the GLIM 1.2.2 `/glim_ros/map` bug |
| `src/pcd2pgm/` | Point cloud -> `OccupancyGrid` node; live mode follows `/glim_ros/map` on a fixed grid |
| `bringup.sh` | One-terminal bring-up of the live mission stack (section 6) |
| `src/kratos_nav/` | Nav2 config/launch, LiDAR driver launch, `waypoint_mission.py`, `rover_bridge.py`, bring-up monitor, hardware-free tests |
| `docs/` | Mission overview, live-test runbooks, design notes |
| `AGENTS.md` | Orientation for AI assistants and reviewers (`CLAUDE.md` points to it) |
| `src/FAST_LIO/` | FAST-LIO2 (earlier approach; not used by the GLIM pipeline) |
| `src/lidar_angle_filter/` | Front/rear antenna-sector PointCloud2 filter |
| `src/livox_ros_driver2/` | Livox ROS 2 driver source and MID-360 network configuration |
| `maps/` | Local GLIM dump storage; generated at runtime and ignored by Git |
