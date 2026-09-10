# Livox MID-360 GLIM SLAM Workspace

ROS 2 Humble workspace for real-time LiDAR-inertial odometry and 3D simultaneous
localization and mapping (SLAM) with a Livox MID-360 and GLIM.

The active pipeline is:

```text
Livox MID-360 -> livox_ros_driver2 -> PointCloud2 + IMU -> GLIM -> odometry + 3D map
```

GLIM performs continuous pose estimation while it builds and optimizes the map. This
repository does not provide autonomous path planning, waypoint following, or
localization against a previously saved map. Loading a saved GLIM dump in the offline
viewer is for visualization, editing, and export; it does not start live localization.

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
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake \
  git \
  libpcl-dev \
  libeigen3-dev \
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

### 1.6 Clone and build the Livox driver

```bash
git clone https://github.com/Project-Kratos-28/auto_fastlio2.git
cd auto_fastlio2
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

source install/setup.bash
```

GLIM is installed system-wide and is not built by `colcon` in this workspace.

### 1.7 Configure the MID-360 network

The checked-in driver configuration expects:

| Device | IPv4 address |
|---|---|
| Computer Ethernet adapter | `192.168.1.50/24` |
| MID-360 | `192.168.1.125` |

Configure the wired adapter with the desktop network settings. No particular network
connection profile or interface name is required. Confirm the resulting address and
sensor connectivity:

```bash
ip -brief address
ping -c 3 192.168.1.125
```

If a computer or sensor uses different addresses, update all host address fields and
the sensor `ip` field in `src/livox_ros_driver2/config/MID360_config.json` before
building the driver again.

## 2. Create a map with GLIM

Use three terminals. In every terminal, change to the cloned repository first; the
commands do not assume where it was cloned.

### 2.1 Start the MID-360 driver

Terminal 1:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Use `rviz_MID360_launch.py`, not `msg_MID360_launch.py`. GLIM consumes
`sensor_msgs/msg/PointCloud2`; the `msg_` launch publishes Livox `CustomMsg` data.

Before starting GLIM, confirm both streams:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

Expected rates are approximately 10 Hz for LiDAR frames and 200 Hz for IMU data.

### 2.2 Start GLIM SLAM

Stop the rover and keep it completely motionless before running this command.

Terminal 2:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"

ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:="$(realpath "$REPO_ROOT/glim/glim_config")"
```

Keep the rover still until GLIM prints `initial IMU state estimation result`, normally
after two to five seconds. Starting while the rover is moving can produce incorrect
IMU bias, orientation, and velocity estimates. Move only after initialization has
completed.

The one-time messages about large point timestamps and Livox `FLOAT64` nanoseconds are
expected when automatic MID-360 timestamp detection is enabled.

### 2.3 Visualize the live map

Terminal 3:

```bash
cd /path/to/auto_fastlio2
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
rviz2 -d "$REPO_ROOT/glim/glim_ros.rviz"
```

The supplied RViz layout already contains the required GLIM displays:

- `/glim_ros/points`: current registered LiDAR scan
- `/glim_ros/map`: accumulated optimized map; updated approximately every 10 seconds
- `/glim_ros/odom`: current LiDAR-inertial pose

The driver launch also opens its own RViz window for the raw point cloud. Use the GLIM
RViz window or GLIM's standard viewer to inspect the accumulated map.

### 2.4 Record the area

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
ros2 run tf2_ros tf2_echo map livox_frame
```

If the driver frame was changed from `livox_frame`, substitute its configured frame ID.

Useful live publishers can be confirmed without modifying RViz:

```bash
ros2 topic info /glim_ros/points --verbose
ros2 topic info /glim_ros/map --verbose
```

## 4. Stop and save a map

In Terminal 2, press `Ctrl+C` once. Wait for GLIM to print `saved` before closing the
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

Open a saved dump directly:

```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros offline_viewer --map_path /absolute/path/to/saved/glim_dump
```

Alternatively, launch `ros2 run glim_ros offline_viewer`, then select
`File -> Open Map` and choose the dump directory.

The offline viewer can optimize explicit constraints:

- Loop closure: right-click one submap sphere and select `Loop begin`; select another
  sphere and choose `Loop end`; align the clouds and create the factor.
- Plane adjustment: right-click a point on a flat surface, choose
  `Bundle Adjustment (Plane)`, set the selection radius, and create the factor.

Remove unwanted map points with:

```bash
ros2 run glim_ros map_editor
```

Export the map from `File -> Save -> Export Points`. GLIM exports PLY. Convert it to PCD
when another component requires PCD:

```bash
pcl_ply2pcd /path/to/map.ply /path/to/map.pcd
```

## 6. Repository layout

| Path | Purpose |
|---|---|
| `glim/glim_config/` | MID-360 GLIM CPU and GPU configuration files |
| `glim/glim_ros.rviz` | Preconfigured live GLIM RViz layout |
| `glim/README.md` | Compact GLIM command reference and configuration notes |
| `src/livox_ros_driver2/` | Livox ROS 2 driver source and MID-360 network configuration |
| `maps/` | Local GLIM dump storage; generated at runtime and ignored by Git |
