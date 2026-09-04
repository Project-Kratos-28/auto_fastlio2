# Livox Mid-360 LiDAR SLAM, Map Conversion, and Relocalization Workspace

A ROS 2 Humble workspace for the Livox Mid-360 LiDAR featuring:
* **High-Frequency LiDAR-Inertial Odometry (LIO)** and dense 3D point cloud mapping with PCD export.
* **3D-to-2D Occupancy Grid Conversion (`pcd2pgm`)** to generate standard `map.pgm` and `map.yaml` maps for ROS 2 Nav2 navigation.
* **Real-Time 3D Relocalization** against pre-built global maps using two-stage coarse-to-fine Iterative Closest Point (ICP).

---

## 1. Getting Started and Workspace Setup

This section outlines system requirements, external dependencies, network configuration, and building instructions for setting up the workspace on a clean system.

### 1.1 System Requirements

* **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS Distribution**: ROS 2 Humble Hawksbill (Desktop or Base)
* **LiDAR Sensor**: Livox Mid-360 with integrated 6-axis IMU

### 1.2 Prerequisites and Dependencies

Install core build tools, Point Cloud Library (PCL), Eigen, and standard ROS 2 Humble dependencies:

```bash
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    libpcl-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-message-filters \
    ros-humble-nav2-map-server \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    pcl-tools
```

#### Install Livox-SDK2
The Livox ROS 2 driver requires `Livox-SDK2` installed system-wide:

```bash
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### Install Sophus
The `fastlio2` odometry package requires the `Sophus` Lie group library:

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 1.3 Network Configuration (Livox Mid-360)

The Livox Mid-360 communicates over Ethernet via UDP. By default, it sends data to host IP `192.168.1.50` and listens at static IP `192.168.1.125`.

#### Configure Host Network Interface
Set a static IP on the Ethernet interface connected to the LiDAR:

* **IP Address**: `192.168.1.50`
* **Subnet Mask**: `255.255.255.0` (`/24`)
* **Gateway**: `192.168.1.1`

Using NetworkManager CLI (`nmcli`):
```bash
# Identify your Ethernet interface name (e.g., eth0, enp3s0)
ip link

# Configure static IP
sudo nmcli connection modify <INTERFACE_NAME> ipv4.addresses 192.168.1.50/24 ipv4.method manual
sudo nmcli connection up <INTERFACE_NAME>
```

#### Verify Sensor Connectivity
```bash
ping 192.168.1.125
```
If your LiDAR has a custom IP address or broadcast code, update `src/livox_ros_driver2/config/MID360_config.json`.

### 1.4 Cloning and Building the Workspace

Clone the repository and build all packages using `colcon`:

```bash
# Clone the repository
git clone https://github.com/Project-Kratos-28/auto_fastlio2.git ~/ros2_livox_ws
cd ~/ros2_livox_ws

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the entire workspace
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

# Source workspace overlay
source install/setup.bash
```

To reload workspace environment variables automatically in new terminal sessions:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_livox_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Complete Operation Workflows

The workspace supports an end-to-end pipeline:
1. **3D Mapping**: Scan the environment and export a dense `.pcd` point cloud.
2. **2D Map Conversion (`pcd2pgm`)**: Slice and rasterize the 3D map into a 2D occupancy grid (`map.pgm` + `map.yaml`) for Nav2.
3. **3D Relocalization**: Ingest live scans and relocalize against the global map with ICP (`map -> lidar` TF).
4. **Autonomous Navigation**: Deploy the map with Nav2 for path planning and obstacle avoidance.

---

### 2.1 Generating and Saving a 3D PCD Map

#### Step 1: Launch the Livox LiDAR Driver
Terminal 1:
```bash
cd ~/ros2_livox_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

#### Step 2: Launch FAST-LIO Mapping and Visualizer
Terminal 2:
```bash
cd ~/ros2_livox_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```
Move the sensor smoothly through the target area. RViz will display real-time odometry and registered point cloud accumulation.

#### Step 3: Save the Global Map
Map points are accumulated during the run. You can save the map in two ways:

* **Automatic Save on Shutdown**:
  Terminate the mapping process in Terminal 2 with `Ctrl + C`. The node automatically saves the full-resolution point cloud to:
  ```text
  src/FAST_LIO/PCD/scans.pcd
  ```
* **Manual Save Trigger via Service**:
  While mapping is still active, run in another terminal:
  ```bash
  ros2 service call /map_save std_srvs/srv/Trigger "{}"
  ```

#### Step 4: Verify the Saved PCD Map
Inspect the generated point cloud using `pcl_viewer`:
```bash
pcl_viewer src/FAST_LIO/PCD/scans.pcd
```
*(Tip: Press keys `1`, `2`, `3`, `4`, or `5` inside `pcl_viewer` to switch color rendering modes: Random, X, Y, Z, or Intensity).*

---

### 2.2 Converting 3D PCD to 2D Occupancy Grid (`.pgm` + `.yaml`) for Nav2

ROS 2 Nav2 requires a 2D grid map where pixels represent free space (white, `254`) and obstacles (black, `0`). The `pcd2pgm` package extracts vertical obstacles from `scans.pcd` while filtering out the floor and ceiling.

#### Step 1: Configure Conversion Parameters
Open `src/pcd2pgm/config/pcd2pgm.yaml` to configure thresholds:
```yaml
pcd2pgm:
  ros__parameters:
    # Path to input PCD file
    pcd_file: "/home/eepy/ros2_livox_ws/src/FAST_LIO/PCD/scans.pcd"
    
    # 2D Grid map resolution in meters per pixel (0.05 = 5cm/pixel)
    map_resolution: 0.05
    
    # Output OccupancyGrid topic name
    map_topic_name: "map"
    
    # Rigid coordinate transform [x, y, z, roll, pitch, yaw] from odom to lidar
    odom_to_lidar_odom: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Z-axis passthrough filter thresholds:
    # IMPORTANT: flag_pass_through: false keeps points INSIDE [thre_z_min, thre_z_max]
    flag_pass_through: false
    thre_z_min: 0.2     # Height above ground where obstacles start (0.15m - 0.2m excludes floor)
    thre_z_max: 1.8     # Max height of obstacles (cuts off ceiling and overhead pipes)
    
    # Radius Outlier Removal (cleans LiDAR dust and floating noise)
    thre_radius: 0.5
    thres_point_count: 5
```

> [!TIP]
> **Why `thre_z_min` should be 0.15–0.20m instead of 0.0m:**  
> If `thre_z_min` is set to `0.0`, all ground points hit by the LiDAR will be treated as obstacles, turning the entire room black and blocking path planning. Setting `thre_z_min: 0.2` cuts out the floor while preserving full vertical walls, doors, table legs, and columns.

#### Step 2: Run the `pcd2pgm` Converter
Terminal 1:
```bash
cd ~/ros2_livox_ws
source install/setup.bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```
This node filters `scans.pcd`, publishes the 2D grid to `/map`, and opens RViz2 for inspection.

#### Step 3: Save the 2D Map Using Nav2 Map Server
Terminal 2:
```bash
cd ~/ros2_livox_ws
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_livox_ws/map
```
This will generate:
* `map.pgm`: 2D occupancy grid image.
* `map.yaml`: Map metadata (resolution, origin, occupancy thresholds) required by Nav2.

*(Note: `map.pgm` and `map.yaml` are ignored by git in this repository to keep commits lightweight).*

---

### 2.3 Relocalizing on the Pre-Built Map

Once `scans.pcd` is available, run the real-time relocalization pipeline to track robot position against the global map.

#### Step 1: Launch the LiDAR Driver
Terminal 1:
```bash
cd ~/ros2_livox_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

#### Step 2: Launch the Localizer and Odometry Stack
Terminal 2:
```bash
cd ~/ros2_livox_ws
source install/setup.bash
ros2 launch localizer localizer_launch.py
```
This launch file starts:
* `lio_node` (`fastlio2`): High-frequency LiDAR-inertial state estimation.
* `localizer_node` (`localizer`): Coarse-to-fine ICP matching engine.
* `rviz2`: Visualizer preconfigured with localizer views and map frames.

#### Step 3: Trigger Relocalization Service
Provide the path to the map and a rough initial pose estimate `(x, y, z, roll, pitch, yaw)`:

```bash
cd ~/ros2_livox_ws
source install/setup.bash

ros2 service call /localizer/relocalize interface/srv/Relocalize "{
  pcd_path: '/home/eepy/ros2_livox_ws/src/FAST_LIO/PCD/scans.pcd',
  x: 0.0,
  y: 0.0,
  z: 0.0,
  roll: 0.0,
  pitch: 0.0,
  yaw: 0.0
}"
```
*Expected service response:*
```text
response:
interface.srv.Relocalize_Response(success=True, message='relocalize success')
```

#### Step 4: Verify Relocalization Convergence
Query the relocalization check service:
```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```
*Expected response when converged:*
```text
response:
interface.srv.IsValid_Response(valid=True)
```

#### Step 5: Verify the Global TF Transform
Confirm the global coordinate transform is active and being broadcasted:
```bash
ros2 run tf2_ros tf2_echo map lidar
```

---

### 2.4 Nav2 Navigation with the Generated Map

To test the generated 2D map with Nav2:

1. Launch Nav2 bringup with your generated map:
   ```bash
   ros2 launch nav2_bringup bringup_launch.py map:=/home/eepy/ros2_livox_ws/map.yaml use_sim_time:=False
   ```
2. Open RViz2 configured with the Nav2 display plugin.
3. Use **"2D Pose Estimate"** to set the initial pose and **"Nav2 Goal"** to command navigation targets.

---

## 3. Technical Details and Architecture

### 3.1 Workspace Package Overview

| Package Directory | ROS 2 Package Name | Description |
| :--- | :--- | :--- |
| `src/livox_ros_driver2` | `livox_ros_driver2` | Hardware communication driver for Livox Mid-360. Streams raw LiDAR packets and IMU data. |
| `src/FAST_LIO` | `fast_lio` | HKU-MARS FAST-LIO direct LiDAR-inertial odometry and mapping node with PCD auto-save support. |
| `src/FASTLIO2_ROS2/fastlio2` | `fastlio2` | Modular C++ ROS 2 reimplementation of FAST-LIO2, providing odometry and body point clouds. |
| `src/FASTLIO2_ROS2/localizer` | `localizer` | Two-stage (rough + refine) ICP relocalization against a static PCD map; broadcasts `map -> lidar` TF. |
| `src/FASTLIO2_ROS2/interface` | `interface` | Service definitions for relocalization queries and map operations (`Relocalize`, `IsValid`, `SaveMaps`). |
| `src/pcd2pgm` | `pcd2pgm` | Converts 3D `.pcd` maps into 2D Occupancy Grid (`.pgm` + `.yaml`) for Nav2 costmaps. |

### 3.2 Coordinate Frames and TF Architecture

The workspace adheres to ROS standard coordinate conventions (REP-105):

```text
[map] (Global Map Frame from PCD)
  │
  └── (Published by localizer_node via ICP alignment)
  │
[lidar] (Local Odometry Frame from fastlio2)
  │
  └── (Published by fastlio2 / lio_node)
  │
[body] (Sensor IMU / Body Frame)
```

* **`map`**: Fixed global coordinate system defined by the pre-recorded `scans.pcd`.
* **`lidar`**: Odometry frame relative to the initial boot position of the sensor.
* **`body`**: Moving sensor frame centered at the Livox Mid-360 IMU origin.

### 3.3 Key ROS 2 Topics

#### Sensor Driver (`livox_ros_driver2`)
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Raw Livox LiDAR point packets with point-level timestamps |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Mid-360 internal IMU readings at 200 Hz |

#### FAST-LIO Mapping (`fast_lio`)
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/Odometry` | `nav_msgs/msg/Odometry` | Real-time estimated LiDAR pose and velocity |
| `/path` | `nav_msgs/msg/Path` | Sensor trajectory path |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Undistorted points registered in world frame (`camera_init`) |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | Undistorted scan points in sensor body frame |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | Incremental global map from ikd-Tree |

#### Map Conversion (`pcd2pgm`)
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D occupancy grid published for visualization and saving |
| `/pcd_cloud` | `sensor_msgs/msg/PointCloud2` | Filtered obstacle point cloud in `map` frame |

#### Localization Stack (`fastlio2` + `localizer`)
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | Odometry published by `fastlio2` (consumed by localizer) |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | Body-frame point cloud (consumed by localizer) |
| `/localizer/map_cloud` | `sensor_msgs/msg/PointCloud2` | Downsampled prior map point cloud published in `map` frame |
| `/tf` | `tf2_msgs/msg/TFMessage` | Broadcasts the rigid transformation from `map` to `lidar` |

### 3.4 Key ROS 2 Services

| Service Name | Service Type | Package | Description |
| :--- | :--- | :--- | :--- |
| `/map_save` | `std_srvs/srv/Trigger` | `fast_lio` | Manually triggers PCD map save during mapping |
| `/localizer/relocalize` | `interface/srv/Relocalize` | `localizer` | Loads the specified PCD map and sets initial pose guess |
| `/localizer/relocalize_check` | `interface/srv/IsValid` | `localizer` | Returns whether ICP has successfully aligned to the map |

#### Service Payload Reference (`/localizer/relocalize`):
```text
string pcd_path    # Absolute path to .pcd map file
float32 x          # Initial estimate X (meters)
float32 y          # Initial estimate Y (meters)
float32 z          # Initial estimate Z (meters)
float32 roll       # Initial estimate Roll (radians)
float32 pitch      # Initial estimate Pitch (radians)
float32 yaw        # Initial estimate Yaw (radians)
---
bool success       # True if PCD was loaded and initial guess was accepted
string message     # Status message
```

### 3.5 Configuration Files Guide

* **`src/livox_ros_driver2/config/MID360_config.json`**:
  Defines host IP, LiDAR IP, UDP ports, and lidar type.
* **`src/FAST_LIO/config/mid360.yaml`**:
  LiDAR-to-IMU extrinsics (`extrinsic_T`, `extrinsic_R`), blind field filtering (`blind_front_deg`, `blind_back_deg`), and map downsample sizes (`filter_size_surf`, `filter_size_map`).
* **`src/FASTLIO2_ROS2/fastlio2/config/lio.yaml`**:
  Parameters for the `fastlio2` odometry node including IMU noise terms, search distances, and extrinsics.
* **`src/FASTLIO2_ROS2/localizer/config/localizer.yaml`**:
  ICP resolution and thresholds for relocalization:
  * `rough_scan_resolution` / `rough_map_resolution`: Voxel leaf sizes for coarse alignment (default `0.25m`).
  * `rough_score_thresh`: Max allowable ICP distance score for rough pass (default `0.2`).
  * `refine_scan_resolution` / `refine_map_resolution`: Voxel leaf sizes for fine alignment (default `0.10m`).
  * `refine_score_thresh`: Max allowable ICP distance score for final alignment (default `0.1`).
  * `update_hz`: Rate at which localizer executes ICP correction cycles (default `1.0 Hz`).
* **`src/pcd2pgm/config/pcd2pgm.yaml`**:
  Occupancy grid conversion settings: resolution (`0.05m`), height boundaries (`thre_z_min`, `thre_z_max`), radius outlier filter (`thre_radius`, `thres_point_count`), and odom-to-lidar transforms.
