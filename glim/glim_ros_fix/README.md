# glim_ros 1.2.2 `/glim_ros/map` fix (overlay)

**Why it's needed:** the apt package `ros-humble-glim-ros` 1.2.2 has a bug in
`rviz_viewer` that corrupts `/glim_ros/map` during a normal run.

1. The pose graph re-optimizes on an idle timer, and every time it fires
   `on_update_submaps`.
2. The viewer appends the latest submap again on each of those calls.
3. Its submap list then grows longer than the pose list, so poses are looked
   up out of range.
4. The published cloud fills with straight "sheets" of garbage points.
5. pcd2pgm turns them into **phantom walls** in `/map`, and Nav2 plans
   around walls that do not exist.

Upstream fixed this in koide3/glim_ros2#76 (after 1.2.2).
`rviz_viewer_submap_guard.patch` is that fix backported: one `if` that
appends only when a new submap actually exists. It is needed whenever
something consumes `/glim_ros/map` (pcd2pgm, and therefore Nav2). It does
not matter for mapping itself.

## Build (once per machine, about 5 min)

The overlay is a separate workspace, so the upstream clone never lands inside this
repo:

```bash
mkdir -p ~/glim_ros_fix_ws/src && cd ~/glim_ros_fix_ws/src
git clone --branch v1.2.2 https://github.com/koide3/glim_ros2.git
cd glim_ros2
git apply /path/to/auto_fastlio2/glim/glim_ros_fix/rviz_viewer_submap_guard.patch
cd ~/glim_ros_fix_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

It builds against the apt GLIM (`libglim` headers from the koide3 PPA, the same
ones `ros-humble-glim-ros` uses). Only `glim_ros` is rebuilt.

On the team VM this already exists at `~/Kratos/glim_ros_fix_ws`, as a git
worktree of `~/Kratos/glim_ros2` at `v1.2.2` (`fc72f46`) with the patch applied.

## Use

Source the overlay **last**, after ROS and this workspace, in the terminal that
starts GLIM:

```bash
source /opt/ros/humble/setup.bash
source /path/to/auto_fastlio2/install/setup.bash       # waypoint_manager
source ~/glim_ros_fix_ws/install/local_setup.bash     # MUST be last
ros2 pkg prefix glim_ros    # must print .../glim_ros_fix_ws/install/glim_ros, NOT /opt/ros/humble
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath /path/to/auto_fastlio2/glim/glim_config)
```

To be sure the patched library is the one loaded, run this while GLIM is running:
`grep -E 'librviz_viewer|libglim_ros' /proc/$(pgrep -f glim_rosnode)/maps`
Both libraries must come from `glim_ros_fix_ws`.

## When to drop this

Once the apt GLIM is newer than 1.2.2 and contains #76, delete this folder and the
overlay `source` lines in `docs/`.
