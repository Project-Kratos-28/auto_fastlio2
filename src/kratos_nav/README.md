# kratos_nav

Nav2 for the Kratos rover. It uses GLIM for localization and pcd2pgm's live
`/map`, and runs the autonomous phase of the mission.

| File | What it does |
|---|---|
| `launch/nav.launch.py` | Nav2 `navigation_launch.py` (**no** map_server or AMCL) plus the static TF `base_link -> livox_frame` (args `lidar_x`, `lidar_z`, default 0.60) |
| `config/nav2_params.yaml` | Global costmap static layer on `/map` (from pcd2pgm), 6x6 m rolling local costmap, Smac Hybrid (DUBIN, radius 0.6), RPP without rotate-in-place, `odom_topic: /glim_ros/odom` |
| `behavior_trees/*_no_spin.xml` | Default BTs with the Spin recovery removed (the rover can't turn in place) |
| `scripts/waypoint_mission.py` | Drives to GLIM waypoints in order with `navigate_to_pose`. Re-queries `/get_waypoint` every 2 s and re-sends the goal if the waypoint moved more than 0.3 m (loop closure) |
| `scripts/rover_bridge.py` | The only writer of `/rover` (ESP32 wheel PWM). MANUAL passes the joystick through (`/rover_joy`); AUTO converts `/cmd_vel` to open-loop diff drive |
| `test/` | Hardware-free tests. See `test/README.md` |

## Run

The full bring-up order, with health checks, is in `docs/LIVE_MISSION_TEST.md`.
In short:

```bash
source /opt/ros/humble/setup.bash
source <repo>/install/setup.bash
ros2 launch kratos_nav nav.launch.py            # before GLIM (provides base_link -> livox_frame)
# ... GLIM + pcd2pgm running, waypoints tagged ...
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
```

With no `waypoints` param, the mission uses `/list_waypoints`. That is every
bound waypoint in tag order, then any still-pending ones. The `" (pending)"`
suffix is stripped from the names.

### waypoint_mission.py parameters

| Parameter | Default | Meaning |
|---|---|---|
| `waypoints` | `['']` (= use the list) | Names, in order |
| `refresh_period` | 2.0 s | How often to re-ask GLIM where the current waypoint is |
| `replan_threshold` | 0.3 m | Re-send the goal if the waypoint moved more than this |
| `wait_for_waypoint_sec` | 30.0 s | How long to wait for a pending (unbound) waypoint |
| `stop_on_failure` | true | Abort the mission when one waypoint fails |
| `base_frame`, `sensor_frame` | `base_link`, `livox_frame` | GLIM returns the LiDAR pose; it is converted to `base_link` with the static TF |

Ctrl+C cancels the active Nav2 goal. A goal Nav2 accepts after the 5 s send
timeout is cancelled too.

### rover_bridge.py

```bash
ros2 run drive_controls drive.py --ros-args -r /rover:=/rover_joy   # drive team's node, remapped
ros2 run kratos_nav rover_bridge.py --ros-args -p track_width:=<m> -p max_wheel_speed:=<m/s>
ros2 service call /rover_bridge/set_auto std_srvs/srv/SetBool "{data: true}"
```

- Moving a stick past `override_deadzone` drops back to MANUAL.
- It publishes at `rate` (50 Hz). Input older than `input_timeout` (0.5 s)
  becomes zeros, and zeros are sent on shutdown.
- The rover firmware holds its last command if `/rover` stops (for example
  after `kill -9` or a network drop). Keep a hand on the power switch.

## Placeholders (measure before trusting)

| Value | Where | Placeholder |
|---|---|---|
| LiDAR height `lidar_z` | `nav.launch.py`, `nav2_params.yaml` (`min/max_obstacle_height`), `src/pcd2pgm/config/pcd2pgm_live.yaml` (`thre_z_min/max`) | 0.60 m. **Change all three together** |
| Footprint | `nav2_params.yaml` (global and local costmap) | 0.74 x 0.74 m square |
| `track_width`, `max_wheel_speed` | `rover_bridge.py` params | 0.80 m, 1.0 m/s. Not measured, and PWM is not linear in speed |

## Tests

```bash
python3 src/kratos_nav/test/test_rover_bridge.py                                   # pure Python
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh          # fake GLIM + real Nav2, ~2-4 min
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/mission_edge_test.sh # late-accept + Ctrl+C
```

Without `KRATOS_SETUP`, the shell tests source the VM workspaces
(`~/Kratos/glim_ext_ws`, `~/Kratos/kratos_nav_ws`). They use `ROS_DOMAIN_ID`
43 and 45, so run one at a time.
