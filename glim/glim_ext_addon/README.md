# Waypoint manager + headless dump exporter (GLIM extras)

Tags waypoints relative to the current submap's own origin instead of a frozen world
coordinate, so a tagged point automatically rides along with every pose-graph
loop-closure correction instead of going stale. See the root
[`WAYPOINT_AND_CONE_GUIDE.md`](../../WAYPOINT_AND_CONE_GUIDE.md) for the broader
semantic-landmark design this is meant to sit underneath — this module is the
submap-relative pose-correction piece that guide's section 7.2 flags as not yet
implemented ("a later production implementation should associate each landmark
observation with the relevant GLIM keyframe/submap and re-optimize landmark positions
after loop closure").

Depends only on `glim` and our own `waypoint_interfaces` package — no dependency on the
now-removed `FASTLIO2_ROS2/interface` package.

This folder also has `glim_dump_export`, a small standalone CLI that loads a saved
GLIM dump and writes straight to PCD — no GUI, no PLY intermediate. The installed
GLIM (1.2.2) `offline_viewer` doesn't have a command-line export flag (that was added
in a later GLIM version than the apt PPA ships), so this exists to make dump export
scriptable. Unlike `waypoint_manager`, it isn't a GLIM extension module and doesn't
need to go inside the `glim_ext` clone at all — it only needs `glim` itself.

## Build (one-time, separate from the main workspace)

`glim_ext` (koide3's GLIM extension-module repo) is not part of this workspace and is
not vendored here in full — clone it fresh, then drop the two `waypoint_*` packages in:

```bash
git clone https://github.com/koide3/glim_ext.git ~/glim_ext
cp -r waypoint_manager waypoint_interfaces ~/glim_ext/modules/mapping/
```

Add `waypoint_interfaces` as a dependency and wire the new module into
`~/glim_ext/CMakeLists.txt` (top-level, aggregated build — modules are not independent
colcon packages in this repo):

```cmake
# near the other option(...) lines:
option(ENABLE_WAYPOINT_MANAGER "Enable submap-relative waypoint manager module" ON)

# near the other `<depend>` lines in glim_ext/package.xml, add:
#   <depend>waypoint_interfaces</depend>

# near the other if(ENABLE_...) add_subdirectory(...) blocks:
if(ENABLE_WAYPOINT_MANAGER)
  add_subdirectory(modules/mapping/waypoint_manager)
  list(APPEND glim_ext_LIBRARIES waypoint_manager)
endif()
```

Build (needs `waypoint_interfaces` built first, then `glim_ext` — a separate colcon
workspace from the main one, `glim` and this main workspace's install just need to be
sourced first):

```bash
mkdir -p ~/glim_ext_ws/src
ln -s ~/glim_ext ~/glim_ext_ws/src/glim_ext
cp -r waypoint_interfaces glim_dump_export ~/glim_ext_ws/src/

source /opt/ros/humble/setup.bash
source /path/to/auto_fastlio2/install/setup.bash   # only needed if you also use lidar_angle_filter etc.
cd ~/glim_ext_ws
colcon build --symlink-install --packages-select waypoint_interfaces glim_ext glim_dump_export
```

## Run

Add `libwaypoint_manager.so` to `extension_modules` in `glim_config/config_ros.json`
(already done in this repo's `glim_config/`). Source the new workspace alongside the
others before launching GLIM:

```bash
source /opt/ros/humble/setup.bash
source /path/to/auto_fastlio2/install/setup.bash
source ~/glim_ext_ws/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath /path/to/glim_config)
```

## Services / topics

| Interface | Type | Purpose |
|---|---|---|
| `/add_waypoint` | `waypoint_interfaces/srv/AddWaypoint` | Tag the current pose as a named waypoint |
| `/get_waypoint` | `waypoint_interfaces/srv/GetWaypoint` | Resolve a named waypoint to its current `map`-frame pose, computed fresh from the submap's live-corrected position every call |
| `/list_waypoints` | `waypoint_interfaces/srv/ListWaypoints` | Names of everything tagged so far |
| `/save_waypoints` | `waypoint_interfaces/srv/SaveWaypoints` | Write all tagged waypoints to a YAML file |
| `~/waypoints` | `visualization_msgs/MarkerArray` | Live-updated RViz markers (green sphere + text label), republished every 1s and whenever the pose graph re-optimizes |

Also autosaves to `/tmp/waypoints_autosave.yaml` every 10s while any waypoints exist, so
a crash doesn't lose an entire session's tags.

## Exporting a saved dump to PCD

```bash
source /opt/ros/humble/setup.bash
source ~/glim_ext_ws/install/setup.bash
ros2 run glim_dump_export glim_dump_export /path/to/saved/glim_dump /path/to/map.pcd "$(realpath /path/to/glim_config)"
```

Uses the dump's poses exactly as already optimized during the live run
(`enable_optimization=false`, same as GUI export) — does not add new loop closures.
Use `offline_viewer` instead if you need to manually close a missed loop or edit the
map before exporting.

## Known limitations

- **Single-session only.** Waypoints are stored `(name, submap_id, local_pose)` —
  submap-relative, not a resolved world coordinate — but there is no *load* path yet.
  Restarting GLIM starts a new map at a new origin; a saved YAML from a previous session
  cannot currently be reloaded into a new one. Loading requires solving submap
  correspondence between the old and new sessions (a relocalization problem), not yet
  implemented.
- `/add_waypoint` returns `success=false` until GLIM has processed at least one
  odometry frame and formed at least one submap (i.e., after a small amount of motion).
