# Waypoint Manager — Live Test (solo run)

> **This tests the waypoint manager in isolation (no Nav2).** For the full
> tag-then-navigate mission, use `LIVE_MISSION_TEST.md` instead — it supersedes
> this doc for anything beyond checking loop-closure snapping in isolation.
> RViz config path and service package name below were current as of
> 2026-09-11; the service is now under `waypoint_interfaces` (from `glim_ext_ws`; fixed inline 2026-09-15).
> Since 2026-09-15 `/list_waypoints` also shows not-yet-bound tags as `name (pending)`,
> and save/autosave keep them under `pending_waypoints`.

Everything is already built and wired in. Nothing is currently running on the VM
(checked and killed before writing this). Follow top to bottom.

## Part 0 — Network (do this first, every session)

**On the Mac:**
```bash
ifconfig en11 | grep status        # must say "active"
sudo ifconfig bridge100 addm en11
```
(This was dropped since last time — needs re-attaching.)

**On the VM** (only if the check below fails):
```bash
ip -br addr show enp0s1            # look for 192.168.1.10/24 in the list
# if missing:
sudo ip addr add 192.168.1.10/24 dev enp0s1
```

**Verify the LiDAR is reachable (VM):**
```bash
ip neigh | grep 192.168.1.162      # a resolved MAC = good. Don't use ping.
```

---

## Part 1 — Driver (VM, Terminal 1)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
Must be `rviz_MID360_launch.py`, not `msg_MID360_launch.py`. Ignore the small RViz
window this opens — it only shows the raw cloud, not the map.

**Sanity check (VM, throwaway terminal):**
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /livox/lidar     # ~10 Hz
ros2 topic hz /livox/imu       # ~200 Hz
```
Ctrl+C once both look right, then move on.

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

## Part 2 — GLIM + waypoint manager (VM, Terminal 2)

**All three of these `source` lines are required** — the third one is new, it's
what makes the waypoint manager available:

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/auto_fastlio2/install/setup.bash
source ~/Kratos/glim_ext_ws/install/setup.bash
source ~/Kratos/glim_ros_fix_ws/install/local_setup.bash  # patched GLIM rviz_viewer (1.2.2 /glim_ros/map bug), must be last
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/Kratos/glim_config)
```

A GLIM 3D viewer window opens automatically.

**Healthy startup log includes:**
```
[glim] load libwaypoint_manager.so
[waypoint_manager] initializing waypoint manager
[waypoint_manager] waypoint manager ready: ~/add_waypoint ~/get_waypoint ~/save_waypoints ~/list_waypoints ~/waypoints
[odom] initial IMU state estimation result
```
No `TF_SELF_TRANSFORM` spam, no segfault, no `unknown initialization mode`.

---

## Part 3 — RViz (VM, Terminal 3)

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/Kratos/glim_ros2/rviz/glim_ros.rviz
```

This config does **not** have the waypoint markers added yet — add them once, manually:
1. Bottom-left **Add** button → **By topic** tab
2. Find `/glim_ros/waypoints` → **MarkerArray** → Add
3. Fixed Frame (top of Displays panel) should already be `map` — leave it

You should see nothing yet (no waypoints tagged). That's correct.

---

## Part 4 — Tag waypoints (VM, Terminal 4 — commands)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/glim_ext_ws/install/setup.bash   # waypoint_interfaces lives here
```

Move the sensor a bit first (you need at least one odometry frame; walk ~1-2 m so
the tag lands in a finished submap). Then, at any point you want to mark:

```bash
ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: 'wp1'}"
```
Expect:
```
response:
interface.srv.AddWaypoint_Response(success=True, message="waypoint 'wp1' tagged in submap 0")
```
If you get `success=True, message="waypoint 'wp1' queued until its submap is finalized"`, that is fine:
the tag binds automatically once GLIM closes the current submap (keep moving). The GLIM log shows
`queued waypoint ...` and later `tagged waypoint 'wp1' submap_id=N`. `success=False, message='no odometry yet - move the sensor first'`
means GLIM has not produced a single frame yet.

Repeat with different names (`wp2`, `wp3`, …) as you explore. A green sphere +
text label should appear in RViz on `/glim_ros/waypoints` within ~1 second of
each successful call.

---

## Part 5 — The walk (this is the actual test)

1. Start at a point you can identify again. Tag it (`wp1`).
2. Walk a loop with **perimeter > 10 m** (a 4×4 m square is enough), slow,
   < 0.3 m/s, low yaw rate. Tag 1-2 more waypoints along the way.
3. Return to your start point, retracing your original heading, and continue
   2-3 m past it — this is what triggers a loop closure.
4. **Watch the RViz markers while you close the loop.** This is the actual
   pass/fail moment:
   - ✅ **Working correctly**: the waypoint markers **snap/jump slightly** in
     RViz at the moment the loop closes (walls in the GLIM viewer should also
     visibly tighten/merge at the same moment). The marker is following the
     corrected map, not sitting frozen at a stale coordinate.
   - ❌ **Not working**: nothing moves even though the map visibly corrected
     itself in the GLIM viewer window. (Shouldn't happen — but if it does,
     stop and we'll debug the callback wiring together next session.)
5. Do a second lap if you have room — more loop closures, more chances to see
   it snap.

---

## Part 6 — Save (optional, same as before)

In the `glim_rosnode` terminal (Terminal 2), Ctrl+C once, wait for exit, then:
```bash
mkdir -p ~/Kratos/maps
mv /tmp/dump ~/Kratos/maps/$(date +%Y%m%d_%H%M%S)
```

---

## Quick troubleshooting

| Symptom | Fix |
|---|---|
| `service /add_waypoint not available` | You forgot to source `~/Kratos/glim_ext_ws/install/setup.bash` in Terminal 2 before launching GLIM |
| `ros2 service call ... waypoint_interfaces/srv/AddWaypoint` → "invalid service type" | Forgot to `source ~/Kratos/glim_ext_ws/install/setup.bash` in the terminal you're calling the service from |
| No `libwaypoint_manager.so` line in the GLIM startup log | Config didn't load right — must launch with `config_path:=$(realpath ~/Kratos/glim_config)` (not `glim_deliverable`, which no longer exists) |
| `success=False, no odometry yet` forever | GLIM gets no points/IMU. Check `ros2 topic hz /livox/lidar_filtered` and the GLIM log |
| Waypoint stays `queued` (no marker in RViz) | Its submap is not finished. Keep moving; submaps close every ~50 keyframes |
| Nothing in RViz Displays list for `/glim_ros/waypoints` | Topic only appears after the node is running — launch order matters: GLIM (Part 2) before adding the display (Part 3) |
| Markers appear but never move on loop closure | Note it and stop — real bug to debug together, don't try to fix mid-test |

---

## What NOT to worry about this test

- The earlier LiDAR-occlusion crash (covering the sensor) — known issue, fix deferred, just don't deliberately cover it this run
- Map density/sparsity — not what we're testing today
- GNSS — not part of this design at all
