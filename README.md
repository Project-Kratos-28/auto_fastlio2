# Kratos rover autonomy (branch `jazzy-nvblox`)

LiDAR SLAM, waypoints and Nav2 for the Kratos rover, on a Jetson AGX Orin (JetPack 7.2,
Ubuntu 24.04, ROS 2 Jazzy), with a ZED 2i adding near-field obstacles. The stack ends at
Nav2's `/cmd_vel`; whatever drives the wheels subscribes to it and is not part of this repo.
(The repo name `auto_fastlio2` is historical; FAST-LIO is not used.)

Pipeline: Livox MID-360 → GLIM (SLAM, waypoints) → pcd2pgm `/map` → Nav2 → `/cmd_vel`, plus
ZED 2i → ESS depth → nvblox → Nav2 local costmap. Everything runs in one Docker container
(`kratos_glim`), started by `start.sh`.

## Hardware and network

| Device | Connection | Address / notes |
|---|---|---|
| Livox MID-360 | Orin Ethernet `end0` (100 Mbit/s link, ~24 Mbit/s used) | LiDAR `192.168.1.162`, Orin `192.168.1.50/24` |
| ZED 2i | USB 3 | needs the udev rule below |
| Laptop | Ubiquiti link | same `ROS_DOMAIN_ID`; see [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md#laptop--remote-gui) |

The LiDAR's IP is in `src/livox_ros_driver2/config/MID360_config.json` (`lidar_configs[0].ip`).
A MID-360's factory address is `192.168.1.1XX`, XX = the last two digits of its serial number.
The driver tells the LiDAR where to send data (`host_net_info`, `192.168.1.50`) at startup.

## Setup (once)

```bash
# 1. ZED udev rule (host): lets the SDK reach the camera's sensors
sudo cp ~/kratos_nvblox/docker/99-slabs.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger

# 2. Images: the kratos nvblox image (Isaac ROS 4.6, nvblox, Nav2, ZED SDK 5.4.1), then this one on top
~/kratos_nvblox/docker/build_image.sh
~/kratos_glim/docker/build_image.sh

# 3. Build the workspace (in the container; start.sh also does this if it was never built)
~/kratos_glim/docker/run_container.sh docker/build_ws.sh
```

`docker/Dockerfile.glim` adds GLIM 1.2.2 (CPU, koide3 PPA), Livox-SDK2, Nav2 and the patched
`glim_ros` overlay ([`glim/glim_ros_fix`](glim/glim_ros_fix/README.md)) to the kratos image.
ESS (TensorRT engines, node, Python venv) comes from `~/kratos_nvblox/ess`, mounted into the
container; the ZED SDK's optimized models and calibration from `~/kratos_nvblox/zed`.

## Run

```bash
~/kratos_glim/start.sh                      # everything, ESS depth
~/kratos_glim/start.sh depth:=zed           # ZED SDK NEURAL depth instead of ESS
~/kratos_glim/start.sh depth:=none          # LiDAR only (no camera, no nvblox)
~/kratos_glim/start.sh lidar_z:=0.62 cam_x:=0.35 cam_z:=0.45 cam_pitch:=0.30
~/kratos_glim/start.sh --no-follow ...      # start and return to the prompt
~/kratos_glim/logs.sh                       # follow the log again; logs.sh 'glim|ess' filters
~/kratos_glim/stop.sh                       # stop; files GLIM's session; frees the ZED and GPU
```

**Keep the rover still for the first ~10 s**: GLIM estimates the IMU state at start
(`initial IMU state estimation result` in the log). Moving then gives wrong IMU bias and orientation.

- The stack runs **detached** in the container. Ctrl+C in `start.sh`/`logs.sh`, closing the
  terminal or losing SSH only stops showing the log. Only `stop.sh` stops the stack.
- `start.sh` checks the LiDAR answers, and (unless `depth:=none`) that the ZED is plugged in and
  no other container runs it (only one process can open the camera). It also re-binds the ZED's
  HID interface to `usbhid` if a previous ZED SDK run left it detached (`tools/zed_hid_rebind.sh`):
  otherwise the container gets no `/dev/hidraw` node for the camera's sensors.
- `stop.sh` sends Ctrl+C to the launch (GLIM writes its session to `/tmp/dump`), moves the dump to
  `~/kratos_glim/maps/<date_time>`, stops the container and re-binds the ZED HID interface.
- Logs: `~/kratos_glim/log/bringup/<date_time>.log` (`latest.log` links to the newest).
- **GUIs start only with a local X display** (the Orin's desktop). Over SSH, RViz is not started
  and GLIM runs without its OpenGL viewer: GLIM crashes at start when the viewer cannot open a
  display. Force with `gui:=true|false`.

### Launch arguments (`src/kratos_bringup/launch/kratos.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `depth` | `ess` | `ess`, `zed` (SDK NEURAL) or `none` |
| `gui` | `auto` | `auto`: RViz + GLIM viewer only with a local display |
| `lidar_z` | `0.60` | MID-360 height above ground (m). **Placeholder** |
| `lidar_x` | `0.0` | MID-360 forward of `base_link` (m) |
| `cam_x`, `cam_y`, `cam_z` | `0.30`, `0.0`, `0.45` | ZED (`zed_camera_link`) position from `base_link` (m). **Placeholder** |
| `cam_pitch`, `cam_yaw` | `0.30`, `0.0` | ZED down-tilt and yaw (rad). **Placeholder** |

`base_link` is on the ground, under the rover's turning centre.

## More documentation

| Doc | Contents |
|---|---|
| [`docs/FIELD_TEST.md`](docs/FIELD_TEST.md) | test-day runbook: checks and troubleshooting |
| [`docs/MISSIONS.md`](docs/MISSIONS.md) | tagging waypoints, running missions, saved GLIM sessions |
| [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) | `lidar_z`, camera mount, Nav2/GLIM/nvblox settings, laptop networking, clocks |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | data flow, what starts, measured performance, known limits, repo layout |
| [`docs/TESTING.md`](docs/TESTING.md) | hardware-free tests |
| [`AGENTS.md`](AGENTS.md) | invariants for AI agents and reviewers |
