# GLIM Test Runbook — Map + Localization + Save (MID-360, Ubuntu VM)

> **This tests GLIM alone (mapping, TF, loop closure, saving) — no Nav2, no
> waypoints.** For the full waypoint-tagging + Nav2 mission, use
> `LIVE_MISSION_TEST.md`.

Last verified: 2026-09-10. VM `advaith@192.168.64.6` (Ubuntu 22.04 aarch64, UTM on M1).

## What this test actually covers

| Term | What you're testing here |
|---|---|
| **Mapping** | GLIM builds a factor-graph map live from `/livox/lidar` + `/livox/imu` |
| **Localization** | GLIM's *live* pose estimate while mapping (`/glim_ros/odom`, `map→odom→base_link` TF, plus the static `base_link→livox_frame`). **NOT** relocalization into a previously-saved map — GLIM 1.2.2 has no such mode. |
| **Saving** | Dump the session to disk, reopen it offline, export a `.pcd` |

If you need "boot the robot somewhere in a known map and find yourself" — that is a
separate tool (hdl_localization / RTAB-Map / the FAST-LIO2 localizer). Not this.

---

## Part 0 — Network bring-up (do this every boot / replug)

The MID-360 link is **not persistent**. Both commands below vanish on reboot.

### 0a. On the Mac
```bash
ifconfig en11 | grep status        # must say: status: active   (else check the cable/dongle)
ifconfig -a | grep -B1 vmenet      # confirm the bridge is bridge100 (rarely different)
sudo ifconfig bridge100 addm en11
```

### 0b. In the VM
```bash
sudo ip addr add 192.168.1.10/24 dev enp0s1
ip -br addr show enp0s1             # should now list 192.168.1.10/24 alongside 192.168.64.6/24
```

### 0c. Verify LiDAR is reachable (VM)
```bash
ip neigh | grep 192.168.1.162      # a resolved MAC (lladdr ...) = success. "ping" does NOT work, don't use it.
```

---

## Part 1 — Start the Livox driver (VM, Terminal 1)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

- **Must be `rviz_MID360_launch.py`, never `msg_MID360_launch.py`.** `msg_` publishes Livox
  CustomMsg (`xfer_format=1`); GLIM cannot read it. `rviz_` publishes `PointCloud2`
  (`xfer_format=0`), which GLIM needs.
- This opens its own small RViz showing the raw cloud. Ignore it or close it — it will
  never show the GLIM map.

### Check data is flowing (VM, Terminal 2)
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /livox/lidar    # expect ~10 Hz
ros2 topic hz /livox/imu      # expect ~200 Hz
```
Ctrl+C once both look right.

---

## Part 1b — `base_link → livox_frame` TF (VM, its own terminal, before GLIM)

GLIM's config uses `base_frame_id: base_link` (the setting Nav2 needs). Without this
transform GLIM warns `Failed to lookup transform from livox_frame to base_link` every
frame and publishes no `odom → base_link`. The TF chain is
`map → odom → base_link → livox_frame`.

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher --z 0.60 --frame-id base_link --child-frame-id livox_frame
```

(0.60 = placeholder LiDAR height; same value as `lidar_z` in `nav.launch.py`.)

---

## Part 2 — Start GLIM (VM, Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/glim_ros_fix_ws/install/local_setup.bash  # patched GLIM rviz_viewer (1.2.2 /glim_ros/map bug), must be last
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/Kratos/glim_config)
```

A GLIM 3D viewer window opens automatically (this is your main view, not RViz).

### Healthy startup looks like
- log: `initial IMU state estimation result` with a plausible `imu_bias`
- **no** `TF_SELF_TRANSFORM` spam
- **no** `unknown initialization mode` / segfault (that means the wrong config dir — must be
  `~/Kratos/glim_config`, which was built from `/opt/ros/humble/share/glim/config`)
- points start accumulating in the viewer as you move the sensor

---

## Part 3 — (Optional) RViz second view (VM, Terminal 3)

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/Kratos/glim_ros2/rviz/glim_ros.rviz
```
Use **this** rviz config only. Fixed frame = `map`.

---

## Part 4 — Verify localization (VM, Terminal 4)

```bash
source /opt/ros/humble/setup.bash

ros2 topic hz /glim_ros/odom            # ~8-10 Hz = pose tracking is alive
ros2 topic echo /glim_ros/odom --once   # sane position/orientation numbers

ros2 run tf2_ros tf2_echo map livox_frame   # transform updates as you move; no errors
ros2 run tf2_tools view_frames             # writes frames.pdf: chain must be map -> odom -> base_link -> livox_frame
```

Topics GLIM publishes:
| Topic | Use |
|---|---|
| `/glim_ros/odom` | smooth pose, for control |
| `/glim_ros/odom_corrected` | pose after loop-closure jumps applied |
| `/glim_ros/aligned_points` | live scan in map frame (~11 Hz) |
| `/glim_ros/map` | accumulated map — only publishes every ~10 s **and only while something subscribes**. Give `ros2 topic echo` up to 30 s before deciding it's broken. |

---

## Part 5 — Do the mapping run

1. Start from a spot you can return to. Note it.
2. Walk the sensor slowly (< 0.5 m/s), keep it fairly level, avoid fast yaw.
3. Cover the area. **Return to the start** and overlap the first ~5 m of your path —
   this gives GLIM a loop to close.
4. Watch the viewer: the map should stay crisp; walls shouldn't "double up".
5. When done, keep everything running — go to Part 6.

---

## Part 6 — Save the map

### Method A — GLIM viewer menu (while it's still running)
In the GLIM 3D viewer window: top-left menu → **Save** →
- **Save map** → point cloud → choose `~/Kratos/maps/<name>.ply`
- **Save session / dump** if offered → choose a folder (for later editing)

### Method B — Ctrl+C dump (reliable fallback)
In Terminal 2 (the `glim_rosnode` one) press **Ctrl+C once** and wait for it to exit.
It writes the full session to `/tmp/dump`.

`/tmp/dump` is **overwritten on every run and wiped on reboot** — move it immediately:
```bash
mkdir -p ~/Kratos/maps
mv /tmp/dump ~/Kratos/maps/$(date +%Y%m%d_%H%M%S)
ls ~/Kratos/maps/                      # confirm the timestamped folder is there
```

The dump folder contains: submap subdirectories, the factor graph (`graph.bin`), and TUM
trajectories — `odom_*.txt` (before loop closure), `traj_*.txt` (after).

---

## Part 7 — Reopen offline, refine, export PCD

```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros offline_viewer
```
Then in the window: **File → Open** → select `~/Kratos/maps/<timestamp>/`.

Useful actions:
- **Find loop / Optimize** — add loop closures the online run missed, re-optimize the graph
- **Bundle Adjustment (plane)** — flatten walls/floor
- **Crop** — delete stray points
- **File → Save → Export points** → writes a **`.ply`**

### Convert PLY → PCD
`pcl_ply2pcd` is **not installed yet**:
```bash
sudo apt install -y pcl-tools
```
Then:
```bash
pcl_ply2pcd ~/Kratos/maps/<timestamp>/points.ply ~/Kratos/maps/<timestamp>/map.pcd
```

---

## Troubleshooting

| Symptom | Cause → Fix |
|---|---|
| Nothing in viewer, `ros2 topic hz /livox/lidar` silent | Network not up → redo **Part 0** (dongle→bridge100, `.50` IP) |
| `unknown initialization mode ROBUST` then segfault | Wrong config dir → must be `~/Kratos/glim_config` (built from `/opt/ros/humble/share/glim/config`, not the git clone) |
| `TF_SELF_TRANSFORM` error flood | `publish_imu2lidar` not `false` in `config_ros.json` (it is set correctly in `~/Kratos/glim_config`) |
| Driver runs but GLIM sees no points | Launched `msg_MID360_launch.py` instead of `rviz_MID360_launch.py` |
| `/glim_ros/map` echo times out | Normal — it publishes every ~10 s and only when subscribed. Wait 30 s. |
| Map "doubles up" / walls smear | Moved too fast, or IMU extrinsic off. Slow down; if persistent, re-check `T_lidar_imu` in `config_sensors.json`. |
| RViz shows raw cloud only, no map | Opened the driver's RViz, not `~/Kratos/glim_ros2/rviz/glim_ros.rviz` |

---

## One-glance command sequence (once network is up)

```bash
# T1 - driver
source /opt/ros/humble/setup.bash && source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# T1b - static TF (before GLIM)
ros2 run tf2_ros static_transform_publisher --z 0.60 --frame-id base_link --child-frame-id livox_frame

# T2 - GLIM
source /opt/ros/humble/setup.bash
source ~/Kratos/glim_ros_fix_ws/install/local_setup.bash  # patched GLIM rviz_viewer (1.2.2 /glim_ros/map bug), must be last
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/Kratos/glim_config)

# T3 - rviz (optional)
source /opt/ros/humble/setup.bash && rviz2 -d ~/Kratos/glim_ros2/rviz/glim_ros.rviz

# T4 - checks
source /opt/ros/humble/setup.bash
ros2 topic hz /glim_ros/odom
ros2 run tf2_ros tf2_echo map livox_frame

# save: Ctrl+C on T2, then:
mkdir -p ~/Kratos/maps && mv /tmp/dump ~/Kratos/maps/$(date +%Y%m%d_%H%M%S)

# refine + export
ros2 run glim_ros offline_viewer     # File>Open the timestamped dir, then File>Save>Export points (.ply)
```

---

## Part 8 — Loop-closure test (added 2026-09-10)

### Config change already made on the VM

`~/Kratos/glim_config/config_global_mapping_pose_graph.json`:
`min_travel_dist` **50.0 → 8.0**. Backup at `*.json.bak`.

Stock 50 m means online loop closure never fires in a room-sized space — the two
submaps have to be ≥50 m apart *along the path* before GLIM even considers them a
loop candidate. 8 m makes it testable indoors. Revert with:
```bash
cp ~/Kratos/glim_config/config_global_mapping_pose_graph.json.bak ~/Kratos/glim_config/config_global_mapping_pose_graph.json
```

### How the current config behaves (so numbers make sense)

- keyframe every 0.1 m of travel, 50 keyframes per submap → **~1 submap per 5 m walked**
- `submap_voxel_resolution: 0.5` → coarse saved map (that's why the dump looked sparse)
- loop candidate needs: ≥8 m of travel between the two submaps **AND** they end up within
  5 m of each other in space **AND** VGICP aligns them with ≥50% inliers

### The walk

1. Start at point **A**. Face something with geometry (corner, clutter — not a blank wall).
2. Walk a loop with a **perimeter > 10 m** (a ~4×4 m square is enough). Go slow, < 0.3 m/s,
   low yaw rate.
3. Come back to **A** and keep going 2–3 m past it, **retracing your original heading** —
   the sensor must re-see the start area from a similar angle.
4. Do 2 laps if the space allows. Then stop and save.

### Did online loop closure fire? Watch for any of:

- **Visual snap** in the GLIM viewer — doubled walls suddenly merge into one when you
  re-reach A
- `glim_rosnode` terminal prints a loop / factor line around that moment
- `ros2 topic echo /glim_ros/odom_corrected --once` jumps while `/glim_ros/odom` stays
  smooth (run both before and after the revisit and compare)
- in RViz, the map cloud shifts as a block

If nothing snaps: you didn't travel 8 m before returning, the revisit viewpoint was too
different, or the overlap area was featureless. Loop closure **cannot** fix a smeared
single pass (that's an extrinsic/real-time problem) — only accumulated drift.

### Offline loop closure (`offline_viewer`)

```bash
source /opt/ros/humble/setup.bash
ros2 run glim_ros offline_viewer          # File > Open Map > ~/Kratos/maps/<timestamp>/
```

- The red spheres are submap handles. Lines between them = existing factors
  (consecutive = odometry).
- **Automatic:** menu → **Find loop candidates** (or "Detect loops") → then **Optimize**.
- **Manual:** click submap A sphere, ctrl/shift-click submap B sphere (the two that should
  overlap), right-click → **Create loop factor** → **Optimize**.

**Confirmed working when:**
- a **new edge appears between two non-consecutive** red spheres
- the map **tightens** on Optimize — walls/features that were doubled collapse together
- `traj_lidar.txt` in the dump now differs from `odom_lidar.txt` (pre-loop). Quick diff:
  ```bash
  paste <(tail -1 ~/Kratos/maps/<ts>/odom_lidar.txt) <(tail -1 ~/Kratos/maps/<ts>/traj_lidar.txt)
  ```
  final pose should differ if optimization moved things.

Then **File → Save → Export points** → `.ply`, and `pcl_ply2pcd` to `.pcd`.
