# Livox Mid-360 + FAST-LIO2 (ROS 2 Humble)

This workspace contains the ROS 2 driver and SLAM pipeline for **Livox LiDARs** (specifically tuned for **Livox Mid-360** with built-in IMU) using **FAST-LIO2** for high-frequency direct LiDAR-Inertial Odometry and real-time mapping.

---

## ⚡ Quick Start: 2-Step Launch

Open two separate terminals and run the following commands:

### **Terminal 1: Start Livox LiDAR Driver**
Publishes point cloud and IMU data from the Livox Mid-360:
```bash
cd ~/ros2_livox_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### **Terminal 2: Start FAST-LIO SLAM & Viewer**
Runs the FAST-LIO odometry & mapping node and automatically opens RViz2 with the configured display:
```bash
cd ~/ros2_livox_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

---

## 🖥️ Alternative: Standalone Driver + Raw Viewer (Without SLAM)

If you only want to visualize the raw point cloud from the LiDAR in RViz2 without running FAST-LIO:

```bash
cd ~/ros2_livox_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
*(This launch file starts `livox_ros_driver2_node` in `PointCloud2` mode and automatically opens RViz2 configured with `display_point_cloud_ROS2.rviz`.)*

---

## 🌐 Network Configuration (One-Time Setup)

Livox Mid-360 communicates with the host computer via Ethernet over UDP.

### 1. Host Network Settings
Configure your Ethernet interface with a static IP address in the `192.168.1.X` subnet:
* **Host IP**: `192.168.1.50`
* **Subnet Mask**: `255.255.255.0` (`/24`)
* **Gateway**: `192.168.1.1`

Example command via NetworkManager CLI:
```bash
sudo nmcli connection modify <interface_name> ipv4.addresses 192.168.1.50/24 ipv4.method manual
sudo nmcli connection up <interface_name>
```

### 2. Verify Connectivity
Verify you can ping the LiDAR (default LiDAR IP is usually `192.168.1.1XX` where XX is derived from the broadcast code, configured in this workspace as `192.168.1.125`):
```bash
ping 192.168.1.125
```

### 3. LiDAR IP Configuration File
Network settings for LiDAR and Host are located at:
[`src/livox_ros_driver2/config/MID360_config.json`](file:///home/eepy/ros2_livox_ws/src/livox_ros_driver2/config/MID360_config.json)

---

## 🔨 Building the Workspace

If you make modifications or rebuild the packages:

```bash
cd ~/ros2_livox_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

Or using the Livox build script:
```bash
cd ~/ros2_livox_ws/src/livox_ros_driver2
./build.sh humble
```

---

## 🗺️ PCD Map Saving & Viewing

### 1. Auto-Save on Shutdown
When running FAST-LIO (`mapping.launch.py`), stopping the node with `Ctrl + C` will automatically save the accumulated point cloud map if `pcd_save_en` is enabled:
* Saved output: `src/FAST_LIO/PCD/scans.pcd`

### 2. Manual Trigger via ROS 2 Service
You can trigger a map save at any time while the node is running:
```bash
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

### 3. View Saved PCD Map
View the generated PCD file using `pcl_viewer`:
```bash
pcl_viewer src/FAST_LIO/PCD/scans.pcd
```
*(Press keys `1`, `2`, `3`, `4`, or `5` in pcl_viewer to switch between color visualization modes: Random, X, Y, Z, Intensity).*

---

## 📊 Key ROS 2 Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Raw Livox LiDAR point cloud packets with accurate timestamps |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Mid-360 integrated IMU data (200 Hz) |
| `/Odometry` | `nav_msgs/msg/Odometry` | Real-time estimated LiDAR pose & odometry |
| `/path` | `nav_msgs/msg/Path` | Estimated trajectory path of the sensor |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Undistorted & registered point cloud in global frame (`camera_init`) |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | Undistorted scan points in IMU body frame |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | Incremental global map from ikd-Tree |

---

## ⚙️ Configuration Files

* **FAST-LIO Mid-360 Parameters**: [`src/FAST_LIO/config/mid360.yaml`](file:///home/eepy/ros2_livox_ws/src/FAST_LIO/config/mid360.yaml)
  * Azimuth angle filtering (e.g. `blind_front_deg`, `blind_back_deg`)
  * Extrinsic parameters between LiDAR and IMU (`extrinsic_T`, `extrinsic_R`)
  * Filter resolutions and map sizing (`filter_size_surf`, `filter_size_map`)
* **Livox Driver Mid-360 Network**: [`src/livox_ros_driver2/config/MID360_config.json`](file:///home/eepy/ros2_livox_ws/src/livox_ros_driver2/config/MID360_config.json)
* **RViz2 Configurations**:
  * FAST-LIO SLAM Display: [`src/FAST_LIO/rviz/fastlio.rviz`](file:///home/eepy/ros2_livox_ws/src/FAST_LIO/rviz/fastlio.rviz)
  * Raw Livox Point Cloud Display: [`src/livox_ros_driver2/config/display_point_cloud_ROS2.rviz`](file:///home/eepy/ros2_livox_ws/src/livox_ros_driver2/config/display_point_cloud_ROS2.rviz)
