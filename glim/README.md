# GLIM SLAM — MID-360

Config only. GLIM installs from apt — nothing to build, no `colcon build`.

| Path | |
|---|---|
| `glim_config/` | 15 JSON configs for our MID-360 (keep all of them together) |
| `glim_ros.rviz` | RViz layout — the apt package does not ship this |

---

## Install

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
```

```bash
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
```

```bash
sudo apt update && sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev libgtsam-points-dev ros-humble-glim-ros && sudo ldconfig
```

Check:

```bash
ros2 pkg executables glim_ros
```

---

## CPU vs GPU

The configs here are **CPU-only** by default. On an NVIDIA machine, install the CUDA
build instead (match your CUDA version — `cuda12.2`, `cuda12.6`, `cuda13.1`):

```bash
sudo apt install -y libgtsam-points-cuda12.2-dev ros-humble-glim-ros-cuda12.2
```

...and switch these three lines in `glim_config/config.json`:

| | CPU | GPU |
|---|---|---|
| `config_odometry` | `config_odometry_cpu.json` | `config_odometry_gpu.json` |
| `config_sub_mapping` | `config_sub_mapping_passthrough.json` | `config_sub_mapping_gpu.json` |
| `config_global_mapping` | `config_global_mapping_pose_graph.json` | `config_global_mapping_gpu.json` |

---

## Run

**Terminal 1 — driver.** Use `rviz_`, **not** `msg_` — `msg_` publishes Livox CustomMsg,
which GLIM cannot read, and you get an empty screen with no error:

```bash
cd ~/Kratos/auto_fastlio2 && source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

**Terminal 2 — GLIM:**

```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath <repo>/glim_config)
```

**Terminal 3 — RViz:**

```bash
rviz2 -d <repo>/glim_ros.rviz
```

Hold still ~2 s at startup for IMU init, then move smoothly. Return to where you started
so loop closure has something to close.

Check it is running:

```bash
ros2 topic hz /glim_ros/odom          # ~8-10 Hz
```

In RViz, `/glim_ros/points` is the live scan; `/glim_ros/map` is the accumulated map and
updates only every 10 s.

---

## Saving

`Ctrl+C` terminal 2. GLIM saves automatically to `/tmp/dump` — no service call.

`/tmp/dump` is **overwritten by the next run and deleted on reboot**, so move it now:

```bash
mkdir -p ~/Kratos/maps && mv /tmp/dump ~/Kratos/maps/$(date +%Y%m%d_%H%M%S)
```

Trajectories inside, TUM format: `odom_*.txt` before loop closure, `traj_*.txt` after.

---

## Editing

```bash
ros2 run glim_ros offline_viewer
```

`File → Open Map` → select a dump directory.

**Add a loop closure:** right-click a submap sphere → `Loop begin`, right-click another →
`Loop end`, drag to roughly align red and green, `Align`, then `Create Factor`.

**Flatten a drifted surface:** right-click a point on the plane → `Bundle Adjustment
(Plane)`, size the sphere to cover it, `Create Factor`.

Point removal: `ros2 run glim_ros map_editor`.

---

## Exporting

`File → Save → Export Points` — writes **PLY**, not PCD.

```bash
sudo apt install pcl-tools
pcl_ply2pcd map.ply map.pcd
```
