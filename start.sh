#!/bin/bash
# Host: start the whole Kratos stack (MID-360, GLIM, pcd2pgm, Nav2 + nvblox, ZED 2i + ESS).
# Made for SSH from the laptop:
#   ~/kratos_glim/start.sh                    # defaults (ESS depth)
#   ~/kratos_glim/start.sh depth:=zed         # any kratos.launch.py argument, e.g. lidar_z:=0.62
#   ~/kratos_glim/start.sh --no-follow ...    # start and return (don't show the log)
#   ~/kratos_glim/logs.sh                     # show the log again
#   ~/kratos_glim/stop.sh                     # stop it (GLIM's session goes to ~/kratos_glim/maps/)
#
# The stack runs DETACHED inside the kratos_glim container: closing this terminal, Ctrl+C, or a
# dropped SSH/Wi-Fi link does not stop the rover. Ctrl+C here only stops showing the log.
# GUIs (RViz, GLIM's viewer) start only with a local X display (the Orin's own desktop); over SSH
# the stack runs headless. View it from the laptop with src/kratos_bringup/rviz/laptop.rviz.
# KEEP THE ROVER STILL for the first ~10 s (GLIM estimates the IMU state at start).
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
NAME=kratos_glim
FOLLOW=1
if [ "${1:-}" = "--no-follow" ]; then FOLLOW=0; shift; fi
ARGS="$*"

running() {
    docker ps --format '{{.Names}}' | grep -qx "$NAME" \
        && docker exec "$NAME" pgrep -f '[r]os2 launch kratos_bringup' >/dev/null
}
if running; then
    echo "The stack is already running (./stop.sh stops it). Showing its log."
    [ "$FOLLOW" = 1 ] && exec "$REPO/logs.sh"
    exit 0
fi

# Preflight: sensors reachable, nobody else holding the ZED.
LIDAR_IP=$(grep -oE '"ip" *: *"[0-9.]+"' "$REPO/src/livox_ros_driver2/config/MID360_config.json" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+')
ping -c 1 -W 1 "$LIDAR_IP" >/dev/null 2>&1 || echo "WARNING: MID-360 not answering at $LIDAR_IP (the Orin's LiDAR port must be 192.168.1.50/24)"
if [[ " $ARGS " != *" depth:=none "* ]]; then
    lsusb | grep -qi '2b03:f880' || echo "WARNING: ZED 2i not found on USB 3 (depth:=none runs LiDAR only)"
    for c in $(docker ps --format '{{.Names}}' | grep -vx "$NAME"); do
        if docker top "$c" -eo args 2>/dev/null | grep -qE 'zed_wrapper|zed_node|component_container.*zed'; then
            echo "ERROR: container '$c' is running the ZED; stop it first (only one process can open the camera),"
            echo "       or start LiDAR only: $0 depth:=none"
            exit 1
        fi
    done
    # The ZED's HID interface must be bound to usbhid BEFORE the container starts (it copies /dev then).
    "$REPO/tools/zed_hid_rebind.sh"
fi

"$REPO/docker/run_container.sh" true   # start the container if needed
if [ ! -f "$REPO/install/kratos_bringup/share/kratos_bringup/launch/kratos.launch.py" ]; then
    "$REPO/docker/run_container.sh" docker/build_ws.sh
fi

# Launch detached. A local X display (":0", ":1") is passed on for the GUIs; SSH sessions have none
# (or a forwarded "localhost:10.0", which kratos.launch.py treats as headless).
mkdir -p "$REPO/log/bringup"
LOG="log/bringup/$(date +%Y%m%d_%H%M%S).log"
ln -sfn "$(basename "$LOG")" "$REPO/log/bringup/latest.log"
date +%s > "$REPO/log/bringup/started_at"
docker exec -d -u "$(id -u):$(id -g)" ${DISPLAY:+-e DISPLAY="$DISPLAY"} -w /workspaces/kratos_glim "$NAME" \
    bash -lc "exec ros2 launch kratos_bringup kratos.launch.py $ARGS > $LOG 2>&1"
echo "started (log: ~/kratos_glim/$LOG). Keep the rover still ~10 s for GLIM's IMU initialization."
sleep 2
running || { echo "ERROR: the launch exited right away:"; tail -20 "$REPO/$LOG"; exit 1; }
[ "$FOLLOW" = 1 ] && exec "$REPO/logs.sh"
exit 0
