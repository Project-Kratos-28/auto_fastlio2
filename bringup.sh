#!/usr/bin/env bash
# bringup.sh — Kratos rover full stack: GLIM (LiDAR SLAM) + nvblox/ZED 2i (depth obstacles).
#
# Start order (each stage waits for real data before continuing):
#   1. livox_ros_driver2       /livox/lidar (10 Hz), /livox/imu (200 Hz)
#   2. lidar_angle_filter      /livox/lidar_filtered  (body mask)
#   3. static TF x2            base_link->livox_frame, base_link->zed_camera_link
#   4. Nav2 (7 nodes)          controller, smoother, planner, behavior, bt_navigator,
#                              waypoint_follower, velocity_smoother + lifecycle_manager
#   5. GLIM                    TF map->odom->base_link, /glim_ros/map
#                              *** KEEP ROVER STILL for ~10 s (IMU init) ***
#   6. pcd2pgm                 /glim_ros/map -> /map  (Nav2 global costmap)
#   7a. zed_wrapper            ZED 2i RGB + grey pair (+ depth if depth:=zed)
#   7b. ess_stereo             /ess/depth, /ess/camera_info  (if depth:=ess)
#   7c. nvblox                 /nvblox_node/static_map_slice -> Nav2 local costmap
#   8.  RViz (ONE window)      LiDAR cloud + GLIM map + ZED color + ESS depth +
#                              nvblox mesh + costmaps + Nav2 plan
#
# NOT started: waypoint_mission.py, rover/drive bridge (run separately).
# No Docker, no ROS 1 bridge — bare-metal ROS 2 Jazzy.
#
# Usage:
#   ./bringup.sh                   # ESS depth (default)
#   ./bringup.sh --depth zed       # ZED SDK NEURAL depth instead of ESS
#   ./bringup.sh --depth none      # LiDAR + GLIM only (no camera, no nvblox)
#   ./bringup.sh --lidar-z 0.62    # LiDAR height above ground (m)
#   ./bringup.sh --no-rviz         # headless
#   ./bringup.sh --check           # pre-flight checks only, start nothing
#   ./bringup.sh --help
#
# Stop: Ctrl+C  ->  GLIM session is saved, then everything stops in reverse order.

set +m
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
HOST_IP=192.168.1.50
LIDAR_PORTS='56000|56101|56201|56301|56401|56501'

# ── tuneable defaults ──────────────────────────────────────────────────────────
DEPTH=ess
LIDAR_Z=0.60
LIDAR_X=0.0
CAM_X=0.30
CAM_Y=0.0
CAM_Z=0.45
CAM_PITCH=0.30
CAM_YAW=0.0
RVIZ=auto
CHECK_ONLY=0
STATUS_EVERY=15
ESS_ROOT="${KRATOS_ESS_ROOT:-$HOME/kratos_nvblox/ess}"

usage() {
  cat <<EOF
Usage: $0 [options]
  --depth      ess|zed|none  depth source for nvblox (default: ess)
  --lidar-z M                LiDAR height above ground in metres (default: 0.60)
  --lidar-x M                LiDAR forward offset from base_link (default: 0.0)
  --cam-x/y/z M              ZED 2i position on rover (defaults: 0.30 / 0.0 / 0.45)
  --cam-pitch R              ZED 2i downward tilt in radians (default: 0.30)
  --cam-yaw R                ZED 2i yaw offset (default: 0.0)
  --no-rviz                  do not start RViz
  --status-every S           seconds between health status lines (default: 15)
  --check                    pre-flight checks only, start nothing
  --help
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --depth)        DEPTH="$2";       shift 2 ;;
    --lidar-z)      LIDAR_Z="$2";     shift 2 ;;
    --lidar-x)      LIDAR_X="$2";     shift 2 ;;
    --cam-x)        CAM_X="$2";       shift 2 ;;
    --cam-y)        CAM_Y="$2";       shift 2 ;;
    --cam-z)        CAM_Z="$2";       shift 2 ;;
    --cam-pitch)    CAM_PITCH="$2";   shift 2 ;;
    --cam-yaw)      CAM_YAW="$2";     shift 2 ;;
    --no-rviz)      RVIZ=no;          shift   ;;
    --status-every) STATUS_EVERY="$2";shift 2 ;;
    --check)        CHECK_ONLY=1;     shift   ;;
    -h|--help)      usage; exit 0 ;;
    *) echo "unknown option: $1"; usage; exit 2 ;;
  esac
done
case "$DEPTH" in ess|zed|none) ;; *) echo "--depth must be ess|zed|none"; exit 2 ;; esac

# ── logging / colour ───────────────────────────────────────────────────────────
LOG_ROOT="${KRATOS_LOG_DIR:-$HOME/kratos_logs}"
LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR" && ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"
if [ -t 1 ]; then R=$'\e[31;1m'; G=$'\e[32m'; Y=$'\e[33;1m'; B=$'\e[1m'; D=$'\e[0m'; else R=; G=; Y=; B=; D=; fi

_out()  { printf '%s\n' "$1" 2>/dev/null; printf '%s %s\n' "$(date +%H:%M:%S)" "$2" >>"$LOG_DIR/bringup.log"; }
info()  { _out "[bringup] $*"               "[bringup] $*"; }
ok()    { _out "${G}[bringup] OK  $*${D}"   "[bringup] OK  $*"; }
warn()  { _out "${Y}[bringup] WARN  $*${D}" "[bringup] WARN  $*"; }
alert() { _out "${R}!! ALERT $(date +%H:%M:%S) $*${D}" "!! ALERT $*"; }
step()  { _out "" ""; _out "${B}===== $* =====${D}" "===== $* ====="; }
set_state() { echo "$1" >"$LOG_DIR/state"; }

FAILED=0
fail() { _out "${R}[bringup] FAILED: $*${D}" "FAILED: $*"; FAILED=1; set_state failed; exit 1; }
ge()   { awk -v a="$1" -v b="$2" 'BEGIN{exit !((a+0)>=(b+0))}'; }

hv() {
  local f="$LOG_DIR/health" m
  m=$(stat -c %Y "$f" 2>/dev/null) || return 0
  [ $(($(date +%s)-m)) -le 5 ] && sed -n "s/^$1=//p" "$f"
}

# ── process management ─────────────────────────────────────────────────────────
declare -A PID STARTED RESTARTS LOGOFF
ORDER=()

if env --default-signal=INT true 2>/dev/null; then
  SIGRESET=(env --default-signal=INT)
else
  SIGRESET=(python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])')
fi

start_proc() {
  local name="$1"; shift
  LOGOFF[$name]=$(stat -c %s "$LOG_DIR/$name.log" 2>/dev/null || echo 0)
  echo "----- $(date +%H:%M:%S) start: $*" >>"$LOG_DIR/$name.log"
  setsid "${SIGRESET[@]}" "$@" >>"$LOG_DIR/$name.log" 2>&1 </dev/null &
  PID[$name]=$!; STARTED[$name]=$SECONDS
  [[ " ${ORDER[*]} " == *" $name "* ]] || ORDER+=("$name")
}

alive()       { [ -n "${PID[$1]:-}" ] && kill -0 "${PID[$1]}"    2>/dev/null; }
group_alive() { [ -n "${PID[$1]:-}" ] && kill -0 -- "-${PID[$1]}" 2>/dev/null; }
log_since()   { tail -c +$((${LOGOFF[$1]:-0}+1)) "$LOG_DIR/$1.log" 2>/dev/null; }

stop_proc() {
  local name="$1" t="${2:-10}" i
  group_alive "$name" || return 0
  kill -INT -- "-${PID[$name]}" 2>/dev/null
  for ((i=0;i<t*5;i++)); do group_alive "$name" || break; [ -n "${FORCE:-}" ] && break; sleep 0.2; done
  if group_alive "$name"; then
    kill -TERM -- "-${PID[$name]}" 2>/dev/null; sleep 2
    group_alive "$name" && kill -KILL -- "-${PID[$name]}" 2>/dev/null
  fi
  wait "${PID[$name]}" 2>/dev/null
}

show_log() { _out "--- last lines of $1.log ---" "--- $1.log ---"; tail -n "${2:-15}" "$LOG_DIR/$1.log" 2>/dev/null | sed 's/^/      /'; }

# wait_for DESC TIMEOUT PROC CHECK_FN [HINT] [FATAL_RE]  ->  0=ok 1=timeout 2=died 3=fatal
wait_for() {
  local desc="$1" timeout="$2" proc="$3" check="$4" hint="${5:-}" fatal="${6:-}" t0=$SECONDS
  while ! "$check"; do
    if [ -n "$proc" ] && ! alive "$proc"; then
      alert "$proc exited while waiting for $desc"; show_log "$proc" 25; return 2
    fi
    if [ -n "$fatal" ] && log_since "$proc" | grep -qE "$fatal"; then return 3; fi
    local el=$((SECONDS-t0))
    [ "$el" -ge "$timeout" ] && return 1
    if [ "$el" -gt 0 ] && [ $((el%10)) -eq 0 ]; then info "... waiting for $desc (${el}s/${timeout}s) $hint"; fi
    sleep 1
  done
}

rx_rate() {
  local f="/sys/class/net/$HOST_IF/statistics/rx_bytes" a b
  a=$(cat "$f" 2>/dev/null||echo 0); sleep 1; b=$(cat "$f" 2>/dev/null||echo 0)
  echo $((b-a))
}

lidar_diag() {
  local r; r=$(rx_rate)
  info "network diag: ${r} B/s on $HOST_IF (streaming MID-360 ~3 MB/s)"
  info "network diag: $(ip neigh show "$LIDAR_IP" 2>/dev/null|head -1)"
}

# ── shutdown ───────────────────────────────────────────────────────────────────
CLEANED=0
cleanup() {
  [ "$CLEANED" = 1 ] && return; CLEANED=1
  trap 'FORCE=1; _out "${R}second stop: killing now${D}" "force kill"' INT TERM HUP
  [ "$FAILED" = 1 ] || set_state stopping
  touch "$LOG_DIR/.shutdown"

  if [ "${#ORDER[@]}" -gt 0 ]; then
    step "Stopping in reverse order"
    if group_alive glim; then
      info "sending SIGINT to GLIM (it writes /tmp/dump on clean exit)"
      kill -INT -- "-${PID[glim]}" 2>/dev/null
      local i; for ((i=0;i<60;i++)); do group_alive glim || break; sleep 1; done
    fi
    local i name
    for ((i=${#ORDER[@]}-1;i>=0;i--)); do
      name="${ORDER[$i]}"
      [ "$name" = monitor ] && continue
      group_alive "$name" && { info "stopping $name"; stop_proc "$name" 10; }
    done
    stop_proc monitor 3
  fi

  if [ -d /tmp/dump ]; then
    mkdir -p "$LOG_DIR/glim_session"
    mv /tmp/dump/* "$LOG_DIR/glim_session/" 2>/dev/null && ok "GLIM session saved to $LOG_DIR/glim_session/"
  fi

  if [ "$FAILED" = 1 ]; then
    info "logs: $LOG_DIR"
    if [ -t 0 ] && [ -t 1 ] && [ -z "${FORCE:-}" ] && [ "$CHECK_ONLY" -eq 0 ] && [ "${KRATOS_NO_PROMPT:-0}" -eq 0 ]; then
      printf '%s' "${R}Start-up failed. Press Enter to close.${D}"; read -r _
    fi
  else
    set_state stopped; info "stopped. Logs: $LOG_DIR"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

exec 9>"/tmp/kratos_bringup_$(id -u).lock"
flock -n 9 || { echo "${R}Another bringup.sh is already running. Stop it first.${D}"; CLEANED=1; exit 1; }
set_state starting

# ════════════════════════════════════ STEP 1: SOFTWARE CHECKS ════════════════
step "1/8  Software checks"
info "repo: $REPO | depth: $DEPTH | lidar_z: $LIDAR_Z | logs: $LOG_DIR"

[ -f /opt/ros/jazzy/setup.bash ] || fail "ROS 2 Jazzy not found at /opt/ros/jazzy"
source /opt/ros/jazzy/setup.bash
[ -f "$REPO/install/setup.bash" ] && source "$REPO/install/setup.bash" \
  || warn "$REPO/install/setup.bash missing — run colcon build"

GLIM_CONFIG="$REPO/glim/glim_config"
[ -f "$GLIM_CONFIG/config_ros.json" ] || fail "GLIM config missing: $GLIM_CONFIG/config_ros.json"

pkg_share() {
  local p; IFS=: read -ra _P <<<"${AMENT_PREFIX_PATH:-}"
  for p in "${_P[@]}"; do [ -f "$p/share/$1/package.xml" ] && { echo "$p/share/$1"; return 0; }; done
  return 1
}

REQUIRED=(livox_ros_driver2 lidar_angle_filter pcd2pgm glim_ros kratos_nav)
[ "$DEPTH" != none ] && REQUIRED+=(kratos_perception zed_wrapper nvblox_ros)
MISSING=()
for p in "${REQUIRED[@]}"; do pkg_share "$p" >/dev/null || MISSING+=("$p"); done
[ ${#MISSING[@]} -eq 0 ] || fail "packages not found: ${MISSING[*]}. Run colcon build."
ok "all required packages present"

KN_SHARE="$(pkg_share kratos_nav)"
PCD_YAML="$(pkg_share pcd2pgm)/config/pcd2pgm_live.yaml"
NAV_PARAMS="$KN_SHARE/config/nav2_params.yaml"
MONITOR="$REPO/src/kratos_nav/scripts/bringup_monitor.py"
[ -f "$MONITOR" ] || MONITOR=""
RVIZ_CFG="$(pkg_share kratos_bringup 2>/dev/null)/rviz/kratos.rviz"
[ -f "$RVIZ_CFG" ] || RVIZ_CFG=""
DRV_CFG="$(pkg_share livox_ros_driver2)/config/MID360_config.json"

# lidar_z invariant check
EXP_MIN=$(awk -v z="$LIDAR_Z" 'BEGIN{printf "%.2f", 0.2-z}')
EXP_MAX=$(awk -v z="$LIDAR_Z" 'BEGIN{printf "%.2f", 1.8-z}')
vals() { grep -E "^\s*$1:" "$2" | awk '{print $2}' | sort -u; }
BAD=''
for v in $(vals min_obstacle_height "$NAV_PARAMS") $(vals thre_z_min "$PCD_YAML"); do
  awk -v a="$v" -v b="$EXP_MIN" 'BEGIN{exit !((a-b)^2<1e-4)}' || BAD="$BAD min=$v"
done
for v in $(vals max_obstacle_height "$NAV_PARAMS") $(vals thre_z_max "$PCD_YAML"); do
  awk -v a="$v" -v b="$EXP_MAX" 'BEGIN{exit !((a-b)^2<1e-4)}' || BAD="$BAD max=$v"
done
[ -z "$BAD" ] || fail "lidar_z $LIDAR_Z expects obstacle band $EXP_MIN/$EXP_MAX but found$BAD in nav2_params.yaml / pcd2pgm_live.yaml"
ok "lidar_z $LIDAR_Z -> obstacle band $EXP_MIN .. $EXP_MAX m (nav2_params + pcd2pgm_live consistent)"

if [ "$DEPTH" = ess ]; then
  [ -d "$ESS_ROOT" ]              || fail "ESS not found at $ESS_ROOT (set KRATOS_ESS_ROOT)"
  [ -f "$ESS_ROOT/.venv/bin/python" ] || fail "ESS venv missing: $ESS_ROOT/.venv"
  [ -f "$ESS_ROOT/ess_stereo_node.py" ] || fail "ess_stereo_node.py not in $ESS_ROOT"
  ok "ESS at $ESS_ROOT"
fi

info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-default}"
[ "${ROS_LOCALHOST_ONLY:-0}" = 1 ] && warn "ROS_LOCALHOST_ONLY=1 — laptop/other nodes cannot see this stack"

# ════════════════════════════════════ STEP 2: LEFTOVER PROCESSES ═════════════
step "2/8  Leftover processes"
LEFTOVER_RE='livox_ros_driver2_node|lidar_angle_filter_node|glim_rosnode|pcd2pgm_node|base_link_to_livox|base_link_to_zed|zed_wrapper|ess_stereo_node|[n]vblox|component_container_mt|rviz2'
MY_ANC=" $$ "; p=$$
while [ "$p" -gt 1 ] 2>/dev/null; do
  p=$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null) || break; MY_ANC="$MY_ANC$p "
done
leftovers() {
  for p in $(pgrep -f -- "$LEFTOVER_RE" 2>/dev/null); do
    [[ "$MY_ANC" == *" $p "* ]] && continue
    case "$(ps -o comm= -p "$p" 2>/dev/null)" in
      ''|bash|sh|dash|zsh|fish|ssh|sshd|sudo|su|timeout|tmux*|screen|watch|*terminal*|xterm|konsole|less|vim|nano|grep) continue ;;
    esac; echo "$p"
  done
}
mapfile -t LEFT < <(leftovers)
if [ ${#LEFT[@]} -gt 0 ]; then
  warn "stopping ${#LEFT[@]} leftover processes:"
  for p in "${LEFT[@]}"; do info "   $(ps -o pid=,args= -p "$p" | cut -c1-150)"; done
  kill -INT "${LEFT[@]}" 2>/dev/null
  for _ in $(seq 1 40); do mapfile -t LEFT < <(leftovers); [ ${#LEFT[@]} -eq 0 ] && break; sleep 0.25; done
  [ ${#LEFT[@]} -gt 0 ] && { kill -TERM "${LEFT[@]}" 2>/dev/null; sleep 2; }
  mapfile -t LEFT < <(leftovers); [ ${#LEFT[@]} -gt 0 ] && kill -KILL "${LEFT[@]}" 2>/dev/null
  ok "leftovers stopped"
else ok "no leftover processes"; fi

PORTS=$(ss -uapn 2>/dev/null | grep -E ":($LIDAR_PORTS)\b" | awk '{print "      "$4"  "$6}')
[ -n "$PORTS" ] && { [ "$CHECK_ONLY" = 1 ] && warn "LiDAR ports in use" || { alert "LiDAR UDP ports still in use:"; _out "$PORTS" "$PORTS"; fail "stop the other program first"; }; }

# ════════════════════════════════════ STEP 3: NETWORK ════════════════════════
step "3/8  LiDAR network"
HOST_IF=$(ip -o -4 addr show | awk -v ip="$HOST_IP" '{split($4,a,"/")} a[1]==ip{print $2;exit}')
[ -n "$HOST_IF" ] || fail "$HOST_IP not on any interface. Add: sudo ip addr add $HOST_IP/24 dev <iface>"
ok "$HOST_IP is on $HOST_IF"

CFG_HOST=$(python3 -c '
import json,sys
h=json.load(open(sys.argv[1]))["MID360"]["host_net_info"]
print(" ".join(sorted({v for k,v in h.items() if k.endswith("_ip") and v})))' "$DRV_CFG" 2>&1)
[ "$CFG_HOST" = "$HOST_IP" ] || fail "MID360_config.json points to '$CFG_HOST' but this machine is $HOST_IP"

info "searching for MID-360 on the network..."
LIDAR_IP_PY="$KN_SHARE/launch/lidar_ip.py"
LIDAR_IP=""
if [ -f "$LIDAR_IP_PY" ]; then
  t0=$SECONDS
  while [ $((SECONDS-t0)) -lt 30 ]; do
    LIDAR_IP=$(python3 "$LIDAR_IP_PY" 2>>"$LOG_DIR/bringup.log") && [ -n "$LIDAR_IP" ] && break
    [ "$CHECK_ONLY" = 1 ] && break; sleep 3
  done
else
  LIDAR_IP=$(python3 -c '
import json,sys
devs=json.load(open(sys.argv[1]))["MID360"].get("lidar_configs",[])
if devs: print(devs[0].get("ip",""))' "$DRV_CFG" 2>/dev/null)
fi
[ -n "$LIDAR_IP" ] && ok "MID-360 at $LIDAR_IP" \
  || { [ "$CHECK_ONLY" = 1 ] && warn "no LiDAR found" || fail "MID-360 not found — check power (green LED) and cable into $HOST_IF"; }
export LIVOX_LIDAR_IP="$LIDAR_IP"

if [ "$DEPTH" != none ]; then
  lsusb | grep -qi '2b03:f880' && ok "ZED 2i found on USB" \
    || warn "ZED 2i NOT on USB (2b03:f880 missing) — perception pipeline may fail"
fi

[ "$CHECK_ONLY" = 1 ] && { step "Pre-flight done (--check)"; set_state stopped; exit 0; }

# ════════════════════════════════════ LAUNCH ══════════════════════════════════
export RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0

# Optional health monitor
if [ -n "$MONITOR" ]; then
  setsid "${SIGRESET[@]}" python3 "$MONITOR" --log-dir "$LOG_DIR" --status-every "$STATUS_EVERY" </dev/null &
  PID[monitor]=$!; STARTED[monitor]=$SECONDS; ORDER+=(monitor)
fi

# Health check helpers
lidar_ok() { ge "$(hv lidar_hz)" 5; }
filt_ok()  { ge "$(hv filt_hz)"  5; }
tf_ok()    { [ "$(hv static_tf)" = 1 ]; }
pose_ok()  { local a; a=$(hv pose_age); [ -n "$a" ] && ge "$a" 0 && ! ge "$a" 3; }
map_ok()   { local a; a=$(hv map_age); [ -n "$a" ] && ge "$a" 0; }
nav_ok()   { [ "$(hv nav_state)" = active ]; }

# Fallback when monitor is absent: just check if topic has a message
topic_once() { timeout 3 ros2 topic echo --once --no-arr "$1" >/dev/null 2>&1; }

# ─── STAGE 1: LiDAR driver ────────────────────────────────────────────────────
step "4/8  LiDAR driver"
start_driver() {
  start_proc driver ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args \
    -p xfer_format:=0 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:=livox_frame \
    -p user_config_path:="$DRV_CFG"
}
DRIVER_FATAL='bind failed|Init lds lidar fail'
start_driver
_lidar_chk() { if [ -n "$MONITOR" ]; then lidar_ok; else topic_once /livox/lidar; fi; }
for attempt in 1 2; do
  wait_for "/livox/lidar >= 5 Hz" 60 driver _lidar_chk "(driver needs ~25 s to connect)" "$DRIVER_FATAL"
  rc=$?; [ "$rc" = 0 ] && break
  if [ "$rc" = 3 ]; then show_log driver 12; fail "driver UDP bind failed — another Livox driver running?"; fi
  lidar_diag
  [ "$rc" = 2 ] && fail "LiDAR driver crashed — see driver.log"
  if [ "$attempt" = 1 ] && [ "$(rx_rate)" -ge 50000 ]; then
    warn "data arrives but no points published after 60 s — restarting driver once"
    stop_proc driver 5; start_driver; continue
  fi
  show_log driver 12
  fail "no LiDAR points. ~0 B/s on $HOST_IF = cable/power issue, not software."
done
ok "LiDAR driver up: $(hv lidar_hz 2>/dev/null || echo '?') Hz point cloud"

# ─── STAGE 2: Angle filter ────────────────────────────────────────────────────
step "4/8  Angle filter"
start_proc filter ros2 launch lidar_angle_filter angle_filter.launch.py
_filt_chk() { if [ -n "$MONITOR" ]; then filt_ok; else topic_once /livox/lidar_filtered; fi; }
wait_for "/livox/lidar_filtered" 30 filter _filt_chk \
  || { show_log filter; fail "angle filter not publishing — check filter.log"; }
ok "filtered cloud on /livox/lidar_filtered"

# ─── STAGE 3: Static TFs + Nav2 ──────────────────────────────────────────────
step "5/8  Static TFs + Nav2"
start_proc tf_lidar ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link --child-frame-id livox_frame \
  --x "$LIDAR_X" --y 0.0 --z "$LIDAR_Z" \
  --yaw 0.0 --pitch 0.0 --roll 0.0

start_proc tf_camera ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link --child-frame-id zed_camera_link \
  --x "$CAM_X" --y "$CAM_Y" --z "$CAM_Z" \
  --yaw "$CAM_YAW" --pitch "$CAM_PITCH" --roll 0.0

start_proc nav2 ros2 launch kratos_nav nav.launch.py \
  lidar_z:="$LIDAR_Z" lidar_x:="$LIDAR_X" \
  cam_x:="$CAM_X" cam_y:="$CAM_Y" cam_z:="$CAM_Z" \
  cam_pitch:="$CAM_PITCH" cam_yaw:="$CAM_YAW"

_tf_chk() { if [ -n "$MONITOR" ]; then tf_ok; else ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null | grep -q translation; fi; }
wait_for "TF base_link->livox_frame" 30 nav2 _tf_chk \
  || { show_log nav2; fail "Nav2 / static TF did not come up — check nav2.log"; }
ok "Static TFs live. Nav2 will finish activating once GLIM provides a pose."

# ─── STAGE 4: GLIM ────────────────────────────────────────────────────────────
step "6/8  GLIM SLAM"
GLIM_CFG_USED="$GLIM_CONFIG"
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && grep -q '"libstandard_viewer.so"' "$GLIM_CONFIG/config_ros.json"; then
  HEADLESS_CFG="/tmp/kratos_glim_headless_$(id -u)"
  rm -rf "$HEADLESS_CFG"; mkdir -p "$HEADLESS_CFG"
  for f in "$GLIM_CONFIG"/*; do
    fname="$(basename "$f")"
    if [ "$fname" = "config_ros.json" ]; then
      grep -v '"libstandard_viewer.so"' "$f" >"$HEADLESS_CFG/$fname"
    else cp "$f" "$HEADLESS_CFG/$fname"; fi
  done
  GLIM_CFG_USED="$HEADLESS_CFG"
  info "headless: stripped libstandard_viewer.so from GLIM config"
fi

_out "${Y}>>> KEEP THE ROVER STILL for ~10 s — GLIM is estimating IMU bias <<<${D}" ">>> KEEP STILL"
sleep 3
start_proc glim ros2 run glim_ros glim_rosnode \
  --ros-args -p config_path:="$(readlink -f "$GLIM_CFG_USED")"

_pose_chk() { if [ -n "$MONITOR" ]; then pose_ok; else topic_once /glim_ros/odom; fi; }
wait_for "GLIM pose (TF map->base_link)" 120 glim _pose_chk "(keep still; needs IMU + scan)" \
  || { show_log glim 25; fail "GLIM not giving a pose — check glim.log"; }
ok "GLIM running. Rover can move now."

# ─── STAGE 5: pcd2pgm ─────────────────────────────────────────────────────────
step "7/8  pcd2pgm  (/glim_ros/map -> /map)"
start_proc pcd2pgm ros2 run pcd2pgm pcd2pgm_node \
  --ros-args --params-file "$PCD_YAML"
_map_chk() { if [ -n "$MONITOR" ]; then map_ok; else topic_once /map; fi; }
if wait_for "first /map" 45 pcd2pgm _map_chk "(comes after GLIM's first submap; move ~2 m to help)"; then
  ok "/map received — global costmap ready"
else
  warn "no /map yet — arrives once GLIM finishes a submap. Monitor will report it."
fi

# ─── STAGE 6: Perception pipeline (ZED + ESS + nvblox) ───────────────────────
if [ "$DEPTH" != none ]; then
  step "7/8  Perception: ZED 2i + ESS + nvblox  (depth:=$DEPTH)"
  KP_SHARE="$(pkg_share kratos_perception)"

  ZED_DEPTH_MODE="NONE"; [ "$DEPTH" = zed ] && ZED_DEPTH_MODE="NEURAL"
  start_proc zed ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    camera_name:=zed \
    ros_params_override_path:="$KP_SHARE/config/zed2i_glim.yaml" \
    publish_tf:=false \
    publish_map_tf:=false \
    publish_urdf:=true \
    "param_overrides:=depth.depth_mode:=$ZED_DEPTH_MODE"
  info "ZED 2i started (depth_mode=$ZED_DEPTH_MODE, TF publishing disabled — GLIM owns the pose)"

  if [ "$DEPTH" = ess ]; then
    start_proc ess_stereo "${ESS_ROOT}/.venv/bin/python" \
      "${ESS_ROOT}/ess_stereo_node.py" \
      --ros-args --params-file "$KP_SHARE/config/ess_zed2i.yaml"
    info "ESS stereo node started (engine: $ESS_ROOT/engines/ess_fp16.engine)"
  fi

  # nvblox ESDF slice heights in odom frame (ground at -lidar_z)
  ESDF_MIN=$(awk -v z="$LIDAR_Z" 'BEGIN{printf "%.4f", 0.2-z}')
  ESDF_MAX=$(awk -v z="$LIDAR_Z" 'BEGIN{printf "%.4f", 1.8-z}')
  [ "$DEPTH" = ess ] && { DEPTH_IMG=/ess/depth; DEPTH_INFO=/ess/camera_info; } \
                     || { DEPTH_IMG=/zed/zed_node/depth/depth_registered; DEPTH_INFO=/zed/zed_node/depth/camera_info; }

  NVBLOX_SHARE="$(pkg_share nvblox_examples_bringup 2>/dev/null || echo '')"
  NV_ARGS=()
  [ -f "${NVBLOX_SHARE}/config/nvblox/nvblox_base.yaml" ] && NV_ARGS+=(--params-file "${NVBLOX_SHARE}/config/nvblox/nvblox_base.yaml")
  [ -f "${NVBLOX_SHARE}/config/nvblox/specializations/nvblox_zed.yaml" ] && NV_ARGS+=(--params-file "${NVBLOX_SHARE}/config/nvblox/specializations/nvblox_zed.yaml")
  NV_ARGS+=(--params-file "$KP_SHARE/config/nvblox_glim.yaml")
  NV_ARGS+=(-p "static_mapper.esdf_slice_min_height:=$ESDF_MIN")
  NV_ARGS+=(-p "static_mapper.esdf_slice_max_height:=$ESDF_MAX")
  NV_ARGS+=(-p "static_mapper.esdf_slice_height:=$ESDF_MIN")
  NV_ARGS+=(--remap "camera_0/depth/image:=$DEPTH_IMG")
  NV_ARGS+=(--remap "camera_0/depth/camera_info:=$DEPTH_INFO")
  NV_ARGS+=(--remap camera_0/color/image:=/zed/zed_node/rgb/color/rect/image)
  NV_ARGS+=(--remap camera_0/color/camera_info:=/zed/zed_node/rgb/color/rect/camera_info)

  start_proc nvblox ros2 run rclcpp_components component_container_mt \
    --ros-args "${NV_ARGS[@]}"
  info "nvblox: ESDF band $ESDF_MIN .. $ESDF_MAX m (odom) = 0.2–1.8 m above ground"

  # Soft wait for ZED RGB
  if timeout 30 ros2 topic echo --once --no-arr /zed/zed_node/rgb/color/rect/image >/dev/null 2>&1; then
    ok "ZED 2i RGB image publishing"
  else
    warn "ZED not yet publishing — nvblox will buffer until images arrive. Check zed.log"
  fi
  alive nvblox && ok "nvblox container running" || { show_log nvblox; warn "nvblox exited — check nvblox.log"; }
fi

# ─── Wait for Nav2 to finish activating ──────────────────────────────────────
_nav_chk() { if [ -n "$MONITOR" ]; then nav_ok; else timeout 2 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState '{}' 2>/dev/null | grep -q 'active'; fi; }
if wait_for "Nav2 bt_navigator active" 90 nav2 _nav_chk; then ok "Nav2 fully active"
else warn "Nav2 not active yet — check nav2.log for errors"; fi

# ─── STAGE 7: RViz (single window — LiDAR + ZED + GLIM + nvblox + costmaps) ──
step "8/8  RViz"
if [ "$RVIZ" != no ] && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v rviz2 >/dev/null; then
  if [ -n "$RVIZ_CFG" ]; then
    start_proc rviz rviz2 -d "$RVIZ_CFG"
    ok "RViz started — ONE window with:"
    ok "   LiDAR raw cloud (/livox/lidar)"
    ok "   GLIM 3D map (/glim_ros/map)"
    ok "   ZED 2i colour image (/zed/zed_node/rgb/color/rect/image)"
    ok "   ESS depth image (/ess/depth)"
    ok "   nvblox mesh (/nvblox_node/mesh)"
    ok "   nvblox ESDF slice (/nvblox_node/static_esdf_pointcloud)"
    ok "   2D map, local/global costmaps, Nav2 plan"
  else
    start_proc rviz rviz2
    ok "RViz started (no kratos.rviz found — add displays manually)"
  fi
else
  info "RViz not started (no display or --no-rviz)."
  info "On the laptop (same ROS_DOMAIN_ID): rviz2 -d <path>/laptop.rviz"
fi

# ═══════════════════════════════════════════════ READY ═══════════════════════
set_state ready
step "READY"
cat <<EOF | while IFS= read -r l; do _out "$l" "$l"; done
  LiDAR: $LIDAR_IP | depth: $DEPTH | lidar_z: $LIDAR_Z | logs: $LOG_DIR
  Nav2 goal:    click "2D Goal Pose" in RViz, or publish to /goal_pose
  Mission:      ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
  Clear costs:  ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
  Stop:         Ctrl+C  (GLIM session saved automatically)
EOF

# ═══════════════════════════════════════════════ SUPERVISION LOOP ════════════
MAX_RESTARTS=4; RESTART_WINDOW=600
declare -A RESTART_T GAVE_UP
NOTED_DEAD=''; LINK_DOWN_SINCE=''; last_link_msg=0

restart_proc() {
  local name="$1"
  [ -n "${GAVE_UP[$name]:-}" ] && return
  [ $((SECONDS-${RESTART_T[$name]:-0})) -gt "$RESTART_WINDOW" ] && RESTARTS[$name]=0
  RESTART_T[$name]=$SECONDS
  RESTARTS[$name]=$(( ${RESTARTS[$name]:-0}+1 ))
  if [ "${RESTARTS[$name]}" -gt "$MAX_RESTARTS" ]; then
    GAVE_UP[$name]=1
    alert "$name crashed $MAX_RESTARTS times in $((RESTART_WINDOW/60)) min — giving up. Check $LOG_DIR/$name.log"
    return
  fi
  warn "restarting $name (${RESTARTS[$name]}/$MAX_RESTARTS)"
  stop_proc "$name" 5
  case "$name" in
    driver)     start_driver ;;
    filter)     start_proc filter ros2 launch lidar_angle_filter angle_filter.launch.py ;;
    pcd2pgm)    start_proc pcd2pgm ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file "$PCD_YAML" ;;
    nav2)       start_proc nav2 ros2 launch kratos_nav nav.launch.py \
                  lidar_z:="$LIDAR_Z" lidar_x:="$LIDAR_X" \
                  cam_x:="$CAM_X" cam_y:="$CAM_Y" cam_z:="$CAM_Z" \
                  cam_pitch:="$CAM_PITCH" cam_yaw:="$CAM_YAW" ;;
    zed)        KP_SH="$(pkg_share kratos_perception)"
                ZDM="NONE"; [ "$DEPTH" = zed ] && ZDM="NEURAL"
                start_proc zed ros2 launch zed_wrapper zed_camera.launch.py \
                  camera_model:=zed2i camera_name:=zed \
                  ros_params_override_path:="$KP_SH/config/zed2i_glim.yaml" \
                  publish_tf:=false publish_map_tf:=false publish_urdf:=true \
                  "param_overrides:=depth.depth_mode:=$ZDM" ;;
    ess_stereo) start_proc ess_stereo "${ESS_ROOT}/.venv/bin/python" \
                  "${ESS_ROOT}/ess_stereo_node.py" \
                  --ros-args --params-file "$(pkg_share kratos_perception)/config/ess_zed2i.yaml" ;;
    nvblox)     start_proc nvblox ros2 run rclcpp_components component_container_mt \
                  --ros-args "${NV_ARGS[@]}" ;;
    rviz)       [ -n "$RVIZ_CFG" ] && start_proc rviz rviz2 -d "$RVIZ_CFG" || start_proc rviz rviz2 ;;
  esac
}

while :; do
  sleep 1; degraded=0
  for name in driver filter nav2 glim pcd2pgm zed ess_stereo nvblox rviz monitor; do
    [ -n "${PID[$name]:-}" ] || continue
    alive "$name" && continue
    degraded=1
    case "$name" in
      glim)
        [[ "$NOTED_DEAD" == *" glim "* ]] && continue
        NOTED_DEAD="$NOTED_DEAD glim "
        alert "GLIM exited — map, pose and waypoints are gone. NOT restarted. Ctrl+C then restart the bringup."
        show_log glim 25 ;;
      monitor)
        [[ "$NOTED_DEAD" == *" monitor "* ]] && continue
        NOTED_DEAD="$NOTED_DEAD monitor "; alert "health monitor exited (stack keeps running)" ;;
      rviz)
        [ -n "${GAVE_UP[rviz]:-}" ] && continue
        wait "${PID[rviz]}" 2>/dev/null; rc=$?
        if [ "$rc" = 0 ]; then
          info "RViz window closed (not reopening). Stack keeps running."; unset 'PID[rviz]'
        else alert "RViz crashed (exit $rc)"; show_log rviz 10; [ -n "$RVIZ_CFG" ] && restart_proc rviz; fi ;;
      *)
        [ -n "${GAVE_UP[$name]:-}" ] && continue
        alert "$name exited"; show_log "$name" 15; restart_proc "$name" ;;
    esac
  done

  # LiDAR stall detection
  la=$(hv lidar_age 2>/dev/null)
  if alive driver && [ -n "$la" ] && ge "$la" 5 && [ $((SECONDS-${STARTED[driver]})) -gt 45 ]; then
    degraded=1; r=$(rx_rate)
    if [ "$r" -lt 50000 ]; then
      [ -z "$LINK_DOWN_SINCE" ] && LINK_DOWN_SINCE=$SECONDS
      if [ $((SECONDS-last_link_msg)) -ge 15 ]; then
        last_link_msg=$SECONDS
        alert "no LiDAR data on $HOST_IF for $((SECONDS-LINK_DOWN_SINCE))s (${r} B/s) — cable/power?"
      fi
    else
      LINK_DOWN_SINCE=''
      ge "$la" 8 && { alert "LiDAR data arrives (${r} B/s) but driver publishes nothing for ${la}s — restarting"; restart_proc driver; }
    fi
  else LINK_DOWN_SINCE=''; fi

  if [ "$degraded" = 1 ]; then set_state degraded; else set_state ready; fi
done
