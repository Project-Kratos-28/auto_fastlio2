# Live test: tag waypoints with GLIM, then run them with Nav2

This is the one doc for the live test.

> **Where the code lives.** Source of truth is this repo (`auto_fastlio2`). On the team VM
> (`advaith@192.168.64.6`) the test still runs from these workspaces, which hold the
> same code. Paths in the runbooks below use the VM layout.
>
> | Repo path | VM runtime workspace |
> |---|---|
> | `src/kratos_nav` | `~/Kratos/kratos_nav_ws/src/kratos_nav` |
> | `src/pcd2pgm` | `~/Kratos/pcd2pgm_live_ws/src/pcd2pgm` |
> | `glim/glim_ext_addon/*` | `~/Kratos/glim_ext_ws` (waypoint_manager is built inside the `glim_ext` clone there) |
> | `glim/glim_config` | `~/Kratos/glim_config` |
> | `glim/glim_ros_fix` | `~/Kratos/glim_ros_fix_ws` |
> | `src/livox_ros_driver2`, `src/lidar_angle_filter` | `~/Kratos/auto_fastlio2` (this repo, built in place) |
>
> On another machine, build everything from the repo root (see the root `README.md`,
> section 1.6). Then replace each VM `source .../install/setup.bash` with the repo's
> `install/setup.bash`, and each VM config path with the repo path.

## What you're testing

```
drive manually ──► GLIM maps + you tag wp1, wp2, ...      (GLIM never stops)
                     │
                     ├─ /glim_ros/map ─► pcd2pgm ─► /map ─► Nav2 global costmap
                     ├─ TF map→odom→base_link ─────────────► Nav2 knows where the rover is
                     └─ /get_waypoint ◄── waypoint_mission.py ──► Nav2 navigate_to_pose
```

- `waypoint_mission.py` asks GLIM where each waypoint is every 2 s.
- If a loop closure moves a waypoint by more than 0.3 m, it re-sends the goal.
- That's why it asks GLIM live instead of reading a saved file.

The stack ends at Nav2's `/cmd_vel`; whatever drives the wheels subscribes to it.

## Already verified on the VM (no hardware)

- **Unit tests (re-checked 2026-09-15):** waypoint_manager 13/13, lidar_angle_filter 8/8.
- **End-to-end** (`src/kratos_nav/test/e2e_test.sh`): a fake GLIM ran with the real Nav2 and the real `waypoint_mission.py`. The fake publishes a real-size 1000×1000 `/map` every 10 s with a wall in the way, plus dynamic map→odom TF, `/glim_ros/odom` and the waypoint services.
  - The planner detoured around the wall to wp1, with no collision.
  - wp1 moved 0.5 m (fake loop closure) → "re-sending goal" → reached wp1.
  - wp2 was "not found" for its first 3 queries (pending) → retried → reached → "mission complete", exit code 0. Passed twice in a row.
- **pcd2pgm on real data:** the 2026-09-14 GLIM dump (577k points) produced a 1000×1000 `/map` with 14,995 occupied cells. Each update took **105–125 ms**, and only 4 points fell outside the ±25 m grid.
- **GLIM 1.2.2 bug found and patched:** apt GLIM's rviz_viewer could fill `/glim_ros/map` with garbage "sheets" after idle optimizations, which would show up as phantom walls in `/map`. The fix is an overlay, so T4 **must** source `glim_ros_fix_ws` (see below).
- **pcd2pgm empty-cloud fix:** an empty or fully filtered cloud used to publish an all-free `/map` and erase every obstacle. It now keeps the previous map (tested on real data and in the pcd2pgm suite, all scenarios pass).
- **Mission safety (2026-09-16):** if Nav2 accepts a goal after the 5 s send timeout, the mission now cancels it; Ctrl+C cancels the active goal (`src/kratos_nav/test/mission_edge_test.sh`, both pass). The e2e test re-passed after this change.
- **Bug found earlier by the e2e test:** Nav2 silently loaded its stock behavior trees, which contain Spin, and bringup aborted. The launch file can only replace YAML keys that already exist, so both BT keys are now in `nav2_params.yaml`.

---

## 0. Before you go out

- [ ] **Measure `lidar_z`**: MID-360 centre height above the ground, in metres. The placeholder is 0.60. If it's different, change **both** of these files, using min = 0.2 − lidar_z and max = 1.8 − lidar_z:
  - `src/kratos_nav/config/nav2_params.yaml` (VM: `~/Kratos/kratos_nav_ws/src/kratos_nav/config/`) → `min_obstacle_height` / `max_obstacle_height` (both costmaps)
  - `pcd2pgm_live_ws/src/pcd2pgm/config/pcd2pgm_live.yaml` → `thre_z_min` / `thre_z_max`
  - then launch with `lidar_z:=<value>` (step T3)

  Example: lidar_z 0.55 → min -0.35, max 1.25.

- [ ] **Footprint.** Placeholder is 0.74 × 0.74 m. Fix `footprint` in `nav2_params.yaml` (2 places) if the rover is bigger.
- [ ] **VM packages built.** If you edited a `.py`, `.yaml` or `.xml` in kratos_nav, you don't need to rebuild: it's a symlink install. You only need to rebuild if you add a new file:
  ```bash
  cd ~/Kratos/kratos_nav_ws && source /opt/ros/humble/setup.bash && source ~/Kratos/glim_ext_ws/install/setup.bash && colcon build --symlink-install
  ```
- [ ] **Hardware connected.** MID-360 plugged in, then (neither step survives a reboot):
  - Mac: `ifconfig en11 | grep status` must say active, then `sudo ifconfig bridge100 addm en11`
  - VM: `sudo ip addr add 192.168.1.50/24 dev enp0s1` (skip if `ip -br addr show enp0s1` already lists it)
  - Check on the VM with `ip neigh | grep 192.168.1.125`: a resolved MAC means it works. **`ping` fails even when it works**, so don't use it.

## 1. Start everything (on the VM, one terminal each, IN THIS ORDER)

Every terminal first runs:
```bash
source /opt/ros/humble/setup.bash
```

**T1: LiDAR driver**
```bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

**T2: angle filter**
```bash
source ~/Kratos/auto_fastlio2/install/setup.bash
ros2 launch lidar_angle_filter angle_filter.launch.py
```

**T3: Nav2 + the base_link→livox_frame TF** (must be BEFORE GLIM)
```bash
source ~/Kratos/glim_ext_ws/install/setup.bash
source ~/Kratos/kratos_nav_ws/install/setup.bash
ros2 launch kratos_nav nav.launch.py lidar_z:=0.60
```
- Until GLIM is up it prints "waiting for transform map → base_link". That's normal.
- Wait for `Managed nodes are active`.
- If you see `Aborting bringup` instead, see Troubleshooting.

**T4: GLIM.** Keep the rover **still** for the first ~3 s while the IMU initializes.
```bash
source ~/Kratos/auto_fastlio2/install/setup.bash
source ~/Kratos/glim_ext_ws/install/setup.bash
source ~/Kratos/glim_ros_fix_ws/install/local_setup.bash   # patched rviz_viewer: MUST be the last source
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/Kratos/glim_config)
```
- Check the patch is the one being used: `ros2 pkg prefix glim_ros` must print `.../glim_ros_fix_ws/install/glim_ros`. If it prints `/opt/ros/humble`, `/glim_ros/map` can pick up phantom walls.

**T5: pcd2pgm live** (GLIM map → `/map`)
```bash
source ~/Kratos/pcd2pgm_live_ws/install/setup.bash
ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file ~/Kratos/pcd2pgm_live_ws/src/pcd2pgm/config/pcd2pgm_live.yaml
```

**T6: RViz**
```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
- Fixed Frame = `map`.
- Add a MarkerArray display on `/glim_ros/waypoints` to see your tags.

**T7: your commands.** Source humble, `glim_ext_ws` and `kratos_nav_ws` here; T7 is used for everything below.

## 2. Health checks (stop at the first failure)

| # | Command (T7) | Pass looks like |
|---|---|---|
| 1 | `ros2 run tf2_ros tf2_echo base_link livox_frame` | Translation `[0.000, 0.000, 0.600]` (your lidar_z) |
| 2 | `ros2 run tf2_ros tf2_echo map base_link` | Changes when the rover moves. z ≈ **-lidar_z** (base_link is on the ground) |
| 3 | `ros2 topic echo /map --field info --once` | width 1000, height 1000, resolution 0.05, origin -25/-25 |
| 4 | T5 log | `Live map update took N ms` every ~10 s, and N stays well under 10000 |
| 5 | `ros2 lifecycle get /bt_navigator` | `active [3]` |
| 6 | RViz global costmap | Walls where the GLIM map has walls; **the floor is NOT black** |
| 7 | RViz local costmap | 6×6 m square that follows the rover, no permanent blob on the rover itself |

If the floor is all obstacles (6), `lidar_z` is wrong. See section 0.

## 3. Part A: map + tag waypoints

1. Drive manually. Stop where you want a waypoint, **facing the direction you'll want to arrive in**. The planner uses that heading.
2. Tag it:
   ```bash
   ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
   ```
   Pass: `success=True`.
3. Repeat for wp2, wp3…
4. **After the LAST tag, keep driving a few metres.** A waypoint stays "pending" until GLIM finishes its submap. While pending, `/get_waypoint` says `found=False` (the mission waits up to 30 s for it) and loop closure does not move it.
5. Check they're all there:
   ```bash
   ros2 service call /list_waypoints waypoint_interfaces/srv/ListWaypoints "{}"
   ros2 service call /get_waypoint waypoint_interfaces/srv/GetWaypoint "{name: wp1}"
   ```
   Pass: all names are listed **without** `(pending)`, and `found=True` with sensible x/y.
6. Optional backup:
   ```bash
   ros2 service call /save_waypoints waypoint_interfaces/srv/SaveWaypoints "{path: /home/advaith/Kratos/maps/waypoints.yaml}"
   ```

**Do NOT restart GLIM from here on.** Waypoints live in GLIM's memory and are not reloaded after a restart.

## 4. Part B: mission, rover stationary (proves the whole chain)

Keep the wheels disconnected from `/cmd_vel`, so the rover won't move by itself.

```bash
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
```

Also watch the output:
```bash
ros2 topic echo /cmd_vel
```

Pass:
- T7 prints `-> 'wp1' at (x, y)` with the same x/y that `get_waypoint` gave you.
- RViz shows a green path from the rover to wp1 that follows curves, not sharp corners.
- `/cmd_vel` shows `linear.x > 0`, and never `linear.x = 0` with `angular.z ≠ 0` held for long (that would be a spin in place).
- After ~15 s Nav2 gives up because the rover made no progress. **That's expected** since the rover didn't move. The mission prints `failed 'wp1' (status 6)` then `mission finished with failures`.

**Ctrl+C** in T7 cancels the goal at any time.

## 5. Part C: shadow mode (full mission, you are the motors)

Run the same mission command as Part B. This time drive with the joystick **along the green path** at a walking pace.

Pass:
- `reached 'wp1'` when you get within 0.5 m, then it moves on to wp2.
- If GLIM closes a loop you'll see `'wp1' moved X m (loop closure) - re-sending goal`, and the path jumps. That's correct.
- `mission complete` at the end.

If you drive too slowly (< 0.5 m in 15 s) it aborts. Just rerun it.

Real autonomy is the same mission with the wheels listening to `/cmd_vel`.


## 6. Things to write down during the test

- `lidar_z` measured, and whether the floor was clean in the costmap
- the `Live map update took N ms` value at the start and end of the run
- any waypoint that jumped (loop closure) and by how much
- any planner failure, and where (tight spot? goal near a wall?)
- the turning radius on full lock (the placeholder is 0.6 m in `nav2_params.yaml`, `minimum_turning_radius`)

---

## Troubleshooting

| Symptom | Cause → fix |
|---|---|
| T3: `Aborting bringup` | Read the ERROR above it. `spin not available` means the BT keys are missing from `nav2_params.yaml`, so check `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` are there |
| T4 GLIM: `Failed to lookup transform ... livox_frame ... base_link` | T3 wasn't running before GLIM. Keep T3 up, restart GLIM (before tagging!) |
| `Timed out waiting for transform from base_link to map` | GLIM not initialized yet / not receiving points. Check `ros2 topic hz /livox/lidar_filtered` |
| `add_waypoint` → service not available | GLIM not running, or waypoint_manager didn't load. Both workspaces must be sourced in T4. Check `ros2 service list \| grep waypoint` |
| Mission: `'wp3' not resolvable yet (pending submap?) - waiting`, then `not found` after 30 s | Its submap isn't finished. Drive a few more metres and rerun |
| Mission: `waypoint 'x' not found in GLIM` | Typo, or GLIM was restarted (waypoints gone) |
| Mission: `no TF livox_frame -> base_link` | T3 (nav.launch.py) isn't running |
| Mission: `Nav2 navigate_to_pose not available` | T3 not active. Check `ros2 lifecycle get /bt_navigator` |
| Global costmap empty but `/map` exists | `ros2 topic info /map -v`: both sides must be RELIABLE + TRANSIENT_LOCAL |
| Floor shows as obstacles | Height band wrong. Fix `lidar_z` in both files (section 0) |
| `failed to create plan` | The tagged heading is impossible in a tight spot, or the goal is inside inflation. Re-tag in an open spot, facing the travel direction |
| T5: `empty after PassThrough/RadiusOutlier - keeping the previous /map` | pcd2pgm got a cloud with nothing in the height band. It deliberately keeps the old `/map` instead of wiping it (fixed 2026-09-15). Once is fine. If it repeats every 10 s, `lidar_z` / `thre_z_*` are wrong (section 0) |
| Straight "sheets" or walls in `/map` / RViz where nothing exists | GLIM was started without the overlay. Check `ros2 pkg prefix glim_ros` in a T4-sourced shell; restart GLIM with `glim_ros_fix_ws` sourced last (before tagging!) |
| `Live map update took` keeps growing to seconds | The map is getting big. Note the number, and tell the team: the outlier filter runs on the whole map |
| Costmap warnings about dropped LiDAR messages / TF extrapolation | GLIM is lagging. Note it; raise `transform_tolerance` in both costmaps from 0.3 |

## Stop everything

Ctrl+C in T7 → T5 → T4 → T3 → T2 → T1. Before killing GLIM, save waypoints if you want them (step 3.6).
