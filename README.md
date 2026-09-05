# LiDAR Mapping, Localization and Waypoint Markers

## Overview

This project extends the FAST-LIO2 ROS 2 mapping and localization pipeline with **named waypoint tagging and visualization**.

The system allows a user to:

1. Create a LiDAR map using FAST-LIO2.
2. Record named waypoints during mapping.
3. Save the waypoints alongside the generated PCD map.
4. Reload the map for localization.
5. Automatically load the corresponding waypoint file.
6. Visualize saved waypoints as named markers in RViz.

The main idea is that a `.pcd` file stores point-cloud geometry, while waypoint metadata is stored separately in a YAML sidecar file.

---

# System Architecture

```text
                     MAPPING
                     =======

                Livox Mid-360
                      │
                      ▼
             livox_ros_driver2
                      │
                      ▼
                 FAST-LIO2
                      │
          ┌───────────┴────────────┐
          │                        │
          ▼                        ▼
     Point Cloud Map          Current Pose
          │                        │
          │                        ▼
          │                 /add_waypoint
          │                        │
          │                        ▼
          │              Named Waypoints
          │                        │
          └───────────┬────────────┘
                      │
                      ▼
                  /map_save
                      │
          ┌───────────┴────────────┐
          ▼                        ▼
     map_name.pcd       map_name_waypoints.yaml


                   LOCALIZATION
                   ============

        map_name.pcd + map_name_waypoints.yaml
                      │
                      ▼
           /localizer/relocalize
                      │
                      ▼
                Localizer Node
                      │
          ┌───────────┴──────────────┐
          ▼                          ▼
     ICP Localization          Load Waypoints
          │                          │
          ▼                          ▼
       map → lidar          /localizer/waypoints
                                      │
                                      ▼
                             MarkerArray in RViz
                                      │
                         ┌────────────┴────────────┐
                         ▼                         ▼
                    Green Sphere              Waypoint Name
```

---

# Workspace Structure

The workspace used in this project is:

```text
~/lidar_mapping_ws
```

Important packages:

```text
lidar_mapping_ws/
├── src/
│   ├── FAST_LIO/
│   │   ├── src/
│   │   │   └── laserMapping.cpp
│   │   ├── PCD/
│   │   ├── launch/
│   │   └── config/
│   │
│   ├── FASTLIO2_ROS2/
│   │   ├── interface/
│   │   │   └── srv/
│   │   │       └── AddWaypoint.srv
│   │   │
│   │   └── localizer/
│   │       └── src/
│   │           └── localizer_node.cpp
│   │
│   └── livox_ros_driver2/
│
├── build/
├── install/
└── log/
```

---

# Waypoint Storage Design

A PCD file stores LiDAR point-cloud data such as geometry and intensity.

Named waypoints are therefore **not embedded inside the PCD**.

Instead, each map has a corresponding YAML sidecar file.

Example:

```text
PCD/
├── kratos_room.pcd
└── kratos_room_waypoints.yaml
```

When the localizer receives:

```text
kratos_room.pcd
```

it automatically searches for:

```text
kratos_room_waypoints.yaml
```

The waypoint YAML must be located in the **same directory as the corresponding PCD**.

---

# Waypoint Service

A custom ROS 2 service was added:

```text
interface/srv/AddWaypoint.srv
```

Service definition:

```text
string name
---
bool success
string message
```

The FAST-LIO mapping node exposes:

```text
/add_waypoint
```

Calling this service records the current FAST-LIO pose using the supplied name.

Example:

```bash
ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'charging_dock'}"
```

Another example:

```bash
ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'waypoint_2'}"
```

The recorded pose contains:

* Waypoint name
* X coordinate
* Y coordinate
* Z coordinate
* Yaw

All coordinates are stored in the same map coordinate system used by the saved FAST-LIO map.

---

# Example Waypoint YAML

For a map called:

```text
kratos_room.pcd
```

the waypoint file is:

```text
kratos_room_waypoints.yaml
```

Example:

```yaml
waypoints:
  - name: "charging_dock"
    x: 1.234
    y: -0.567
    z: 0.012
    yaw: 1.5708

  - name: "waypoint_2"
    x: 3.456
    y: 2.100
    z: 0.045
    yaw: 0.0
```

---

# Mapping Workflow

## Terminal 1 — Livox Mid-360 Driver

Start the Livox ROS driver:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Keep this terminal running.

---

## Terminal 2 — FAST-LIO2 Mapping

Open a second terminal:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

FAST-LIO2 will now perform live LiDAR odometry and mapping.

Keep this terminal running.

---

## Terminal 3 — Add Waypoints

Open another terminal whenever you want to add named locations.

First source the workspace:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

Check that the waypoint service exists:

```bash
ros2 service list | grep waypoint
```

Expected:

```text
/add_waypoint
```

### Add a waypoint

Move the robot, drone, or LiDAR platform to the desired location and call:

```bash
ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'charging_dock'}"
```

For another location:

```bash
ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'waypoint_2'}"
```

For example:

```bash
ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'entrance'}"

ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'table_1'}"

ros2 service call /add_waypoint interface/srv/AddWaypoint "{name: 'charging_dock'}"
```

Each call stores the **current FAST-LIO pose** in memory.

---

# Saving the Map and Waypoints

When mapping is complete, open a terminal and source the workspace:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then save:

```bash
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

The modified FAST-LIO mapping node saves:

```text
map_name.pcd
```

and:

```text
map_name_waypoints.yaml
```

For example:

```text
PCD/
├── kratos_room.pcd
└── kratos_room_waypoints.yaml
```

Verify the files:

```bash
ls -lh ~/lidar_mapping_ws/src/FAST_LIO/PCD/
```

Inspect the waypoints:

```bash
cat ~/lidar_mapping_ws/src/FAST_LIO/PCD/kratos_room_waypoints.yaml
```

---

# Localization Workflow

Localization requires the following three nodes:

1. Livox driver
2. FAST-LIO2
3. Localizer

---

## Terminal 1 — Start Livox Driver

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Keep this terminal running.

---

## Terminal 2 — Start FAST-LIO2

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Keep this terminal running.

FAST-LIO provides the live point cloud and odometry required by the localizer.

---

## Terminal 3 — Start the Localizer

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch localizer localizer_launch.py
```

The localizer provides:

```text
/localizer/relocalize
/localizer/relocalize_check
/localizer/waypoints
```

---

## Terminal 4 — Relocalize Against a Saved Map

Source the workspace:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

Call the relocalization service.

Example:

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize "{
  pcd_path: '/home/pranjal/lidar_mapping_ws/src/FAST_LIO/PCD/kratos_room.pcd',
  x: 0.0,
  y: 0.0,
  z: 0.0,
  roll: 0.0,
  pitch: 0.0,
  yaw: 0.0
}"
```

Replace the PCD path with the map being used.

The localizer will:

1. Load the PCD map.
2. Generate the matching YAML filename automatically.
3. Load all saved waypoints.
4. Perform ICP-based localization.
5. Publish the waypoint markers.

For example:

```text
kratos_room.pcd
        ↓
kratos_room_waypoints.yaml
```

The localizer terminal should print something similar to:

```text
Loaded 3 waypoints from /home/pranjal/lidar_mapping_ws/src/FAST_LIO/PCD/kratos_room_waypoints.yaml
```

If the YAML file is missing:

```text
Waypoint file not found: ...
```

will be printed instead.

---

# RViz Waypoint Visualization

Waypoints are published as:

```text
/localizer/waypoints
```

The message type is:

```text
visualization_msgs/msg/MarkerArray
```

Each waypoint produces:

* One `SPHERE` marker at the saved position.
* One `TEXT_VIEW_FACING` marker above the sphere containing the waypoint name.

Current visualization parameters:

### Waypoint sphere

```text
Type: SPHERE
Scale: 0.30 × 0.30 × 0.30 m
Position: x, y, z
```

### Waypoint label

```text
Type: TEXT_VIEW_FACING
Position: x, y, z + 0.35 m
Text size: 0.25 m
Text: waypoint name
```

---

## Adding Waypoints to RViz

Start RViz using the project's normal RViz workflow.

Set the **Fixed Frame** to:

```text
map
```

Then add a display:

```text
Add
  ↓
MarkerArray
```

Set the MarkerArray topic to:

```text
/localizer/waypoints
```

After successful relocalization and waypoint loading, the saved waypoints should appear in the same coordinate frame as the map.

---

# Verify the Waypoint Topic

Check whether the publisher exists:

```bash
ros2 topic list | grep waypoint
```

Expected:

```text
/localizer/waypoints
```

Check the message type:

```bash
ros2 topic info /localizer/waypoints
```

Inspect one published MarkerArray:

```bash
ros2 topic echo /localizer/waypoints --once
```

If waypoints have been loaded, the output should contain marker entries with waypoint positions and names.

---

# Build Instructions

After modifying the custom interface:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select interface --allow-overriding interface
```

Build FAST-LIO:

```bash
colcon build --symlink-install --packages-select fast_lio
```

Build the localizer:

```bash
colcon build --symlink-install --packages-select localizer --allow-overriding localizer
```

Then source the workspace:

```bash
source install/setup.bash
```

Verify the custom service:

```bash
ros2 interface show interface/srv/AddWaypoint
```

Expected:

```text
string name
---
bool success
string message
```

Verify FAST-LIO:

```bash
ros2 pkg executables fast_lio
```

Expected:

```text
fast_lio fastlio_mapping
```

Verify the localizer:

```bash
ros2 pkg executables localizer
```

Expected:

```text
localizer localizer_node
```

---

# Implementation Details

## FAST-LIO Changes

The following file was modified:

```text
src/FAST_LIO/src/laserMapping.cpp
```

### Added custom service

```cpp
#include "interface/srv/add_waypoint.hpp"
```

A waypoint structure stores:

```cpp
struct Waypoint
{
    std::string name;
    double x;
    double y;
    double z;
    double yaw;
};
```

The mapping node stores captured waypoints in:

```cpp
std::vector<Waypoint> waypoints_;
```

The service:

```text
/add_waypoint
```

captures the current FAST-LIO state:

```text
state_point.pos
state_point.rot
```

The yaw is extracted from the current rotation and stored with the XYZ position.

When:

```text
/map_save
```

is called, the existing PCD save operation is followed by waypoint YAML generation.

---

# Localizer Changes

The following file was modified:

```text
src/FASTLIO2_ROS2/localizer/src/localizer_node.cpp
```

The localizer:

1. Receives the PCD path through `/localizer/relocalize`.
2. Derives the YAML filename using the PCD filename.
3. Loads the YAML using `yaml-cpp`.
4. Stores waypoints in `m_waypoints`.
5. Publishes them as `visualization_msgs/msg/MarkerArray`.

The waypoint YAML filename is generated using:

```text
parent_directory / (pcd_filename_without_extension + "_waypoints.yaml")
```

Therefore:

```text
/path/to/map.pcd
```

becomes:

```text
/path/to/map_waypoints.yaml
```

Waypoint markers are published in:

```text
m_config.map_frame
```

This is normally:

```text
map
```

Because both the saved PCD and saved waypoint coordinates use the same map frame, no additional coordinate conversion is required after relocalization.

---

# Important Notes

## 1. Keep Matching Map and YAML Names

This is required:

```text
office.pcd
office_waypoints.yaml
```

This will not be loaded automatically:

```text
office.pcd
waypoints.yaml
```

unless the code is modified.

---

## 2. Keep Both Files Together

The PCD and YAML must remain in the same directory.

Correct:

```text
PCD/
├── office.pcd
└── office_waypoints.yaml
```

Incorrect:

```text
PCD/
└── office.pcd

waypoints/
└── office_waypoints.yaml
```

---

## 3. Waypoints Are Captured During Mapping

The `/add_waypoint` service captures the pose estimated by FAST-LIO at the time the service is called.

Therefore, waypoint accuracy depends on:

* FAST-LIO odometry quality.
* LiDAR scan quality.
* Mapping quality.
* Drift accumulated before the waypoint is recorded.

---

## 4. Waypoints Are Currently Visualization Metadata

The current implementation publishes waypoint positions for RViz and makes the marker data available through:

```text
/localizer/waypoints
```

A future navigation or mission node can subscribe to this topic or the system can be extended with a dedicated service for querying:

```text
get_waypoint("charging_dock")
```

and returning:

```text
x
y
z
yaw
```

---

# Troubleshooting

## `/add_waypoint` Does Not Exist

Check:

```bash
ros2 service list | grep waypoint
```

If nothing appears, verify that:

1. The `interface` package was built.
2. `fast_lio` was rebuilt after adding the service dependency.
3. The workspace was sourced.
4. FAST-LIO is currently running.

Rebuild:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select interface --allow-overriding interface

source install/setup.bash

colcon build --symlink-install --packages-select fast_lio

source install/setup.bash
```

---

## `/localizer/waypoints` Does Not Exist

The localizer must be running.

Start it:

```bash
cd ~/lidar_mapping_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch localizer localizer_launch.py
```

Then check:

```bash
ros2 topic list | grep waypoint
```

---

## Waypoint Topic Exists but RViz Shows Nothing

Check the localizer terminal after calling `/localizer/relocalize`.

Expected:

```text
Loaded X waypoints from ...
```

Check that the matching YAML exists:

```bash
ls -lh ~/lidar_mapping_ws/src/FAST_LIO/PCD/
```

Check the topic directly:

```bash
ros2 topic echo /localizer/waypoints --once
```

Also verify that RViz uses:

```text
Fixed Frame: map
```

and the MarkerArray display subscribes to:

```text
/localizer/waypoints
```

---

## YAML File Not Found

If the localizer receives:

```text
/path/room.pcd
```

it searches for:

```text
/path/room_waypoints.yaml
```

Check both the filename and location.

---

# Future Improvements

Possible improvements for future contributors:

### Waypoint Query Service

Add a service such as:

```text
/get_waypoint
```

Request:

```text
string name
```

Response:

```text
bool success
string message
float64 x
float64 y
float64 z
float64 yaw
```

This would allow mission-planning and navigation nodes to directly request a waypoint by name.

---

### Persistent Waypoint Database

Replace the YAML sidecar with a more structured map database containing:

* Map metadata
* Waypoints
* Creation timestamp
* Coordinate frame
* Robot/platform configuration

---

### Waypoint Orientation Visualization

Add arrows to visualize the saved yaw direction instead of only displaying spheres.

---

### Interactive RViz Editing

Use RViz interactive markers to:

* Move waypoints.
* Rename waypoints.
* Delete waypoints.
* Add waypoints manually after mapping.

---

### Waypoint Validation

Add validation for:

* Duplicate names.
* Empty names.
* Invalid YAML fields.
* Waypoints outside expected map bounds.

---

### Navigation Integration

Connect saved waypoints directly to:

* ROS 2 Nav2.
* Drone mission planning.
* Autonomous docking.
* Inspection routes.
* Custom mission scripts.

---

# Quick Localization Reference

### Terminal 1

```bash
cd ~/lidar_mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Terminal 2

```bash
cd ~/lidar_mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### Terminal 3

```bash
cd ~/lidar_mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch localizer localizer_launch.py
```

### Terminal 4

```bash
cd ~/lidar_mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /localizer/relocalize interface/srv/Relocalize "{
  pcd_path: '/home/pranjal/lidar_mapping_ws/src/FAST_LIO/PCD/kratos_room.pcd',
  x: 0.0,
  y: 0.0,
  z: 0.0,
  roll: 0.0,
  pitch: 0.0,
  yaw: 0.0
}"
```

### Terminal 5 — Optional Waypoint Check

```bash
cd ~/lidar_mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /localizer/waypoints --once
```

---

# Current Status

The current implementation supports:

* [x] FAST-LIO2 LiDAR mapping.
* [x] Livox Mid-360 input.
* [x] Named waypoint capture during mapping.
* [x] YAML waypoint sidecar generation.
* [x] Automatic waypoint loading during relocalization.
* [x] Waypoint publication as `MarkerArray`.
* [x] RViz visualization with spheres and text labels.
* [ ] Waypoint query service.
* [ ] Navigation/mission integration.
* [ ] Interactive waypoint editing.
* [ ] Waypoint validation and duplicate handling.
