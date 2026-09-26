# pcd2pgm Live-Mode — Real GLIM Test (solo run)

> **This tests pcd2pgm in isolation (no Nav2).** For the full tag-then-navigate
> mission (which also exercises `/map` inside Nav2's costmaps), use
> `LIVE_MISSION_TEST.md`.

## What you're actually testing

```
LiDAR driver -> GLIM (builds map + pose) -> /glim_ros/map topic
                                                     |
                                     pcd2pgm live node subscribes
                                                     |
                                filters + rasterizes into a FIXED-size grid
                                                     |
                                          publishes /map  (OccupancyGrid)
```

Everything upstream of pcd2pgm (driver, GLIM, waypoints) is already proven. What's
never been checked: does pcd2pgm's live mode produce a sane occupancy grid from
*real*, noisy point cloud data, and does the grid stay a FIXED size/origin every
time it updates (the whole reason we rebuilt it this way instead of the original
file-based version).

Three concrete pass/fail things to watch for, explained in Part 5.

---

## Part 0 — Network (every boot/replug)

**On the Mac:**
```bash
ifconfig en11 | grep status        # must say "active"
sudo ifconfig bridge100 addm en11
```

**On the VM:**
```bash
ip -br addr show enp0s1            # check for 192.168.1.10/24
# if missing:
sudo ip addr add 192.168.1.10/24 dev enp0s1
```

**Verify LiDAR reachable (VM):**
```bash
ip neigh | grep 192.168.1.162      # resolved MAC = good. Don't ping.
```

---

## Part 1 — Driver (VM, Terminal 1)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
Must be `rviz_MID360_launch.py`. Ignore the small RViz window it opens.

**Sanity check (VM, throwaway terminal):**
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /livox/lidar     # ~10 Hz
ros2 topic hz /livox/imu       # ~200 Hz
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

## Part 2 — GLIM (VM, Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/auto_fastlio2/install/setup.bash
source ~/Kratos/glim_ros_fix_ws/install/local_setup.bash  # patched GLIM rviz_viewer (1.2.2 /glim_ros/map bug), must be last
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/Kratos/glim_config)
```
GLIM's 3D viewer opens. Healthy startup: `initial IMU state estimation result`,
no `TF_SELF_TRANSFORM` spam, no segfault.

---

## Part 3 — pcd2pgm live node (VM, Terminal 3)

```bash
source /opt/ros/humble/setup.bash
source ~/Kratos/pcd2pgm_live_ws/install/setup.bash
ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file ~/Kratos/pcd2pgm_live_ws/src/pcd2pgm/config/pcd2pgm_live.yaml
```

**What this line is doing:** starting the modified pcd2pgm node, telling it (via
the yaml) to subscribe to `/glim_ros/map` instead of reading a file, and to
always rasterize into a grid fixed at origin `(-25,-25)`, size `50m x 50m`,
`0.05m` resolution — regardless of what shape the incoming cloud is.

You should NOT see any output yet — it's just sitting there waiting for GLIM's
first `/glim_ros/map` publish, which only happens once something subscribes
(that's this node) and then every ~10s after.

---

## Part 4 — RViz (VM, Terminal 4)

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/Kratos/glim_ros2/rviz/glim_ros.rviz
```

The `/map` topic (the OccupancyGrid pcd2pgm publishes) is NOT in this config yet
— add it manually:
1. Bottom-left **Add** -> **By topic** tab
2. Find `/map` -> **Map** -> Add
3. Fixed Frame should already be `map` — leave it

You now have three things worth watching side by side: the GLIM 3D viewer
(raw accumulated map), RViz's `/glim_ros/aligned_points` if you added it before
(live scan), and this new `/map` display (pcd2pgm's occupancy grid).

---

## Part 5 — The actual test (walk + watch)

1. Start GLIM from a fixed point, walk slowly (< 0.5 m/s), cover a real chunk
   of the room — enough that GLIM's `/glim_ros/map` has real structure in it
   (walls, furniture) by the time you check.
2. Wait for the first `/map` update in RViz (up to ~10-15s after starting —
   GLIM needs at least one submap before it has anything to publish, then
   pcd2pgm needs to receive and process it).

**Check these three things, in order:**

**(a) Does the grid look like your room?**
Black/dark cells should roughly trace your walls and big obstacles. If it's
just noise or empty, the Z-band filter in the yaml is probably wrong for your
sensor's actual mounting height. **Updated 2026-09-14:** the values are now
`thre_z_min: -0.40` / `thre_z_max: 1.20`, measured relative to the GROUND
(GLIM's z=0 is the LiDAR's start height, not the floor) — see
`LIVE_MISSION_TEST.md` section 0 for the formula and how to re-measure them.
Tell me the numbers and we'll fix it together — don't hand-tune mid-test.

**(b) Does the grid size/origin stay IDENTICAL across updates?**
In Terminal 3 (or a 5th terminal), run:
```bash
ros2 topic echo /map --field info --once
```
Note `width`, `height`, `origin`. Wait for the next update (~10s, watch RViz
for the grid to visibly refresh) and echo again. **These numbers must be
byte-identical every time** — `width: 1000 height: 1000`, same origin. This is
the entire point of the fixed-bounds redesign (vs. the original file-based
pcd2pgm, which recomputed size from each cloud's bounding box). If it ever
changes size, something is wrong — stop and flag it, don't try to fix it live.

**(c) Do points ever fall outside the fixed box?**
Watch Terminal 3's log output. If you see a warning about dropped points, it
means part of GLIM's map fell outside the `50m x 50m` box centered near
`(-25,-25)` to `(25,25)`. That's expected/fine if you're testing in a small
room near the origin — but at competition time this box must be sized to
cover the WHOLE arena, or points near the edges get silently discarded.

3. Walk further, into a new area you hadn't covered yet at first check.
   Confirm `/map` picks up the new area on its next ~10s update, still at
   the same fixed size (re-check with the `ros2 topic echo` command).

---

## Part 6 — Shut down

Ctrl+C each terminal, in any order. Nothing here writes anything to disk —
this test doesn't touch `~/Kratos/maps/`. If you also want to save the GLIM
session, that's the same Part 6 as the mapping runbook (Ctrl+C GLIM,
`mv /tmp/dump ~/Kratos/maps/$(date +%Y%m%d_%H%M%S)`).

---

## Troubleshooting

| Symptom | Cause -> Fix |
|---|---|
| `/map` never appears in RViz topic list | pcd2pgm node not running, or `source ~/Kratos/pcd2pgm_live_ws/install/setup.bash` forgotten in Terminal 3 |
| `/map` appears but grid is always empty | GLIM hasn't produced its first submap yet (need more motion) — check GLIM's own 3D viewer has visible points first |
| Grid looks like scattered noise, no wall shapes | `thre_z_min`/`thre_z_max` in the yaml don't match your sensor's real mount height — note the numbers, don't edit mid-test |
| `width`/`height`/`origin` changes between two echoes | Bug — this should be impossible by design. Stop, note exact values from both echoes, we debug together |
| Log spam about dropped points | Some of GLIM's map fell outside the fixed 50x50m box — expected in a small test room, needs real sizing later |
| Node exits immediately / param error | Typo'd the `--params-file` path, or `pcd2pgm_live.yaml` moved — path is `~/Kratos/pcd2pgm_live_ws/src/pcd2pgm/config/pcd2pgm_live.yaml` |
