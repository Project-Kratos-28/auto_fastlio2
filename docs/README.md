# Kratos autonomy docs

Runbooks for the GLIM + pcd2pgm + Nav2 mission stack. Start with
**`LIVE_MISSION_TEST.md`**; it is the one to follow on test day.
To start the whole stack in one terminal: **`./bringup.sh`** at the repo root
(`--help` for options, `--check` for the pre-flight checks only).

| Doc | Use it for | Status |
|---|---|---|
| [`LIVE_MISSION_TEST.md`](LIVE_MISSION_TEST.md) | Full mission: map + tag, loop closure, Nav2 drives to each waypoint (stationary, shadow, real autonomy) | **Current.** Wins on any disagreement |
| [`GLIM_TEST_RUNBOOK.md`](GLIM_TEST_RUNBOOK.md) | GLIM alone: network, driver, mapping, save/export, loop-closure test | Current |
| [`WAYPOINT_LIVE_TEST.md`](WAYPOINT_LIVE_TEST.md) | GLIM + waypoint_manager: tag, walk, check waypoints follow loop closure | Current |
| [`PCD2PGM_LIVE_TEST.md`](PCD2PGM_LIVE_TEST.md) | GLIM + pcd2pgm live: check `/map` grows and has no phantom walls | Current |
| [`NAV2_BRINGUP.md`](NAV2_BRINGUP.md) | Design background for `kratos_nav` | Partly superseded (see its banner) |

Package docs: `src/pcd2pgm/README.md`, `src/kratos_nav/README.md`,
`glim/README.md`, `glim/glim_ext_addon/README.md`, `glim/glim_ros_fix/README.md`.
Guidance for LLM agents: `AGENTS.md` at the repo root.

## The mission in one picture

```
Manual phase    RC-drive, GLIM maps, /add_waypoint at each flag
Return          drive back near the start -> GLIM loop closure fixes drift (waypoints move with it)
Auto phase      waypoint_mission.py -> Nav2 navigate_to_pose, one waypoint at a time

/livox/lidar -> lidar_angle_filter -> GLIM (+waypoint_manager) -> /glim_ros/map (every ~10 s)
                                        |                            |
                                        | TF map->odom->base_link    v
                                        |                       pcd2pgm (live) -> /map (fixed 50x50 m grid)
                                        v                                           |
                 waypoint_mission.py <- /get_waypoint              Nav2 (static layer reads /map)
                                                                        |
                                                   /cmd_vel -> rover_bridge.py -> /rover -> ESP32
```

## Where things live

The source of truth is this repo. The team VM (`advaith@192.168.64.6`) still
runs the stack from separate workspaces with the same code. The mapping table is
at the top of `LIVE_MISSION_TEST.md`. **If you change code in one place, copy it
to the other.**

## Known gaps (as of 2026-09-16)

- **Nothing has been tested on the real rover yet.** Everything below
  "Already verified on the VM" in `LIVE_MISSION_TEST.md` used fakes or
  recorded data.
- **Placeholders:**
  - LiDAR height `lidar_z` = 0.60 (in 3 files, see `src/kratos_nav/README.md`)
  - footprint
  - `rover_bridge` `track_width` and `max_wheel_speed`
- **The rover firmware (ARC_26 repo) holds its last wheel command** if `/rover`
  stops. The bridge sends zeros on stale input and on clean shutdown, but not if
  it is killed hard. The firmware is outside this repo.
- **Fixed map grid:** 50x50 m around GLIM's start point. Points outside it are
  dropped with a WARN. Resize `fixed_*` in `pcd2pgm_live.yaml` to fit the arena.
- **No relocalization:** GLIM must run for the whole mission. A GLIM restart
  starts a new map and loses the waypoints unless they were saved (and saved
  waypoints are not re-anchored).
- **apt GLIM 1.2.2 corrupts `/glim_ros/map`** without the overlay in
  `glim/glim_ros_fix`.
- **GLIM hard-aborts on full LiDAR occlusion** (for example a hand over the
  sensor). No config setting prevents it.
