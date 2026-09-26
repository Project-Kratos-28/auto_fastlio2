#!/usr/bin/env bash
# bringup.sh - start the whole live-mission stack in ONE terminal, in the right order.
#
#   LiDAR driver -> angle filter -> Nav2 (+ base_link->livox_frame TF) -> GLIM
#   -> pcd2pgm (/map) -> rover_bridge (MANUAL) -> RViz
#
# Each stage is started only after the previous one is verified to work (real data
# on its topic, not just "the process is running"). Then it keeps watching: a status
# line every 15 s, an ALERT the moment something breaks, the important log lines of
# every node, automatic restart of the stateless parts. GLIM is never restarted
# automatically: its waypoints live in memory and would be lost.
#
# It never asks for keyboard input (it is started from the GUI button). Stop it with
# Ctrl+C or by closing the window: it saves the waypoints, then stops everything in
# reverse order. If start-up fails, the window stays open until you press Enter.
#
# Files for the GUI / debugging (~/kratos_logs/latest -> the current run):
#   state       starting | ready | degraded | stopping | stopped | failed
#   health      key=value, updated every second (lidar_hz, pose, nav_state, ...)
#   bringup.log everything this terminal printed; <node>.log full output of each node
#
# Usage: ./bringup.sh [--lidar-z 0.60] [--check] ...     ./bringup.sh --help

set +m   # no job control: each 'setsid cmd &' then keeps its PID and leads its own group
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
HOST_IP=192.168.1.10               # this computer's address on the LiDAR network
LIDAR_PORTS='56000|56101|56201|56301|56401|56501'

LIDAR_Z=0.60
LIDAR_X=0.0
LIDAR_IP=''
IFACE=''
TRACK_WIDTH=0.80
MAX_WHEEL_SPEED=1.0
PARAMS=''
GLIM_CONFIG="$REPO/glim/glim_config"
RVIZ=auto
CHECK_ONLY=0
STATUS_EVERY=15

usage() {
  cat <<EOF
Usage: $0 [options]
  --lidar-z M          LiDAR centre height above the ground (default $LIDAR_Z). Must match
                       nav2_params.yaml and pcd2pgm_live.yaml; checked at start-up.
  --lidar-x M          LiDAR forward offset from the turning centre (default $LIDAR_X)
  --lidar-ip IP        use this MID-360 address instead of searching for it
  --iface IF           if $HOST_IP is missing, add it to IF with 'sudo -n' (needs a
                       passwordless sudo rule for exactly that command)
  --track-width M      rover_bridge track width (default $TRACK_WIDTH)
  --max-wheel-speed V  rover_bridge ground speed at full PWM (default $MAX_WHEEL_SPEED)
  --params FILE        Nav2 params file (default kratos_nav/config/nav2_params.yaml)
  --glim-config DIR    GLIM config directory (default $GLIM_CONFIG)
  --no-rviz            don't start RViz (default: start it if there is a display)
  --status-every S     seconds between status lines (default $STATUS_EVERY)
  --check              only run the pre-flight checks, start nothing
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --lidar-z) LIDAR_Z="$2"; shift 2 ;;
    --lidar-x) LIDAR_X="$2"; shift 2 ;;
    --lidar-ip) LIDAR_IP="$2"; shift 2 ;;
    --iface) IFACE="$2"; shift 2 ;;
    --track-width) TRACK_WIDTH="$2"; shift 2 ;;
    --max-wheel-speed) MAX_WHEEL_SPEED="$2"; shift 2 ;;
    --params) PARAMS="$(readlink -f "$2")"; shift 2 ;;
    --glim-config) GLIM_CONFIG="$(readlink -f "$2")"; shift 2 ;;
    --no-rviz) RVIZ=no; shift ;;
    --status-every) STATUS_EVERY="$2"; shift 2 ;;
    --check) CHECK_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1"; usage; exit 2 ;;
  esac
done

# ------------------------------------------------------------------ output helpers
LOG_ROOT="${KRATOS_LOG_DIR:-$HOME/kratos_logs}"
LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR" && ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"
if [ -t 1 ]; then R=$'\e[31;1m'; G=$'\e[32m'; Y=$'\e[33;1m'; B=$'\e[1m'; D=$'\e[0m'; else R=; G=; Y=; B=; D=; fi

_out() { printf '%s\n' "$1" 2>/dev/null; printf '%s %s\n' "$(date +%H:%M:%S)" "$2" >>"$LOG_DIR/bringup.log"; }
info()  { _out "[bringup] $*" "[bringup] $*"; }
ok()    { _out "${G}[bringup] OK  $*${D}" "[bringup] OK  $*"; }
warn()  { _out "${Y}[bringup] WARNING  $*${D}" "[bringup] WARNING  $*"; }
alert() { _out "${R}!! ALERT $(date +%H:%M:%S) $*${D}" "!! ALERT $*"; }
step()  { _out "" ""; _out "${B}===== $* =====${D}" "===== $* ====="; }
set_state() { echo "$1" >"$LOG_DIR/state"; }

FAILED=0
fail() {  # fatal: explain, clean up (trap), keep the window open
  _out "${R}[bringup] FAILED: $*${D}" "[bringup] FAILED: $*"
  FAILED=1
  set_state failed
  exit 1
}

# numeric compare with awk (bash has no floats): ge A B -> A >= B
ge() { awk -v a="$1" -v b="$2" 'BEGIN{exit !((a+0) >= (b+0))}'; }
hv() {  # read a health value; nothing if the monitor stopped updating the file
  local f="$LOG_DIR/health" m
  m=$(stat -c %Y "$f" 2>/dev/null) || return 0
  [ $(($(date +%s) - m)) -le 5 ] && sed -n "s/^$1=//p" "$f"
}

# ------------------------------------------------------------------ process helpers
declare -A PID STARTED RESTARTS LOGOFF
ORDER=()   # start order; stopped in reverse

# Without job control bash starts every 'cmd &' with SIGINT ignored, so a node could
# not be stopped cleanly with Ctrl+C semantics. Put SIGINT back to default first.
if env --default-signal=INT true 2>/dev/null; then SIGRESET=(env --default-signal=INT)
else SIGRESET=(python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])'); fi

start_proc() {  # start_proc NAME cmd... : own session/process group, output to NAME.log
  local name="$1"; shift
  LOGOFF[$name]=$(stat -c %s "$LOG_DIR/$name.log" 2>/dev/null || echo 0)
  echo "----- $(date +%H:%M:%S) start: $*" >>"$LOG_DIR/$name.log"
  setsid "${SIGRESET[@]}" "$@" >>"$LOG_DIR/$name.log" 2>&1 </dev/null &
  PID[$name]=$!
  STARTED[$name]=$SECONDS
  [[ " ${ORDER[*]} " == *" $name "* ]] || ORDER+=("$name")
}

alive() { [ -n "${PID[$1]:-}" ] && kill -0 "${PID[$1]}" 2>/dev/null; }
group_alive() { [ -n "${PID[$1]:-}" ] && kill -0 -- "-${PID[$1]}" 2>/dev/null; }

stop_proc() {  # stop_proc NAME [seconds]: SIGINT (clean ROS shutdown), then TERM, then KILL
  local name="$1" t="${2:-10}" i
  group_alive "$name" || return 0
  kill -INT -- "-${PID[$name]}" 2>/dev/null
  for ((i = 0; i < t * 5; i++)); do group_alive "$name" || break; [ -n "${FORCE:-}" ] && break; sleep 0.2; done
  if group_alive "$name"; then
    kill -TERM -- "-${PID[$name]}" 2>/dev/null; sleep 2
    group_alive "$name" && kill -KILL -- "-${PID[$name]}" 2>/dev/null
  fi
  wait "${PID[$name]}" 2>/dev/null
}

show_log() { _out "      --- last lines of $1.log ---" "--- last lines of $1.log ---"; tail -n "${2:-15}" "$LOG_DIR/$1.log" 2>/dev/null | sed 's/^/      /'; }

log_since_start() { tail -c +$((${LOGOFF[$1]:-0} + 1)) "$LOG_DIR/$1.log" 2>/dev/null; }

# wait_for DESCRIPTION TIMEOUT PROC CHECK_FUNCTION [HINT] [FATAL_LOG_REGEX]
# returns 0 ok, 1 timeout, 2 process exited, 3 fatal line in its log
wait_for() {
  local desc="$1" timeout="$2" proc="$3" check="$4" hint="${5:-}" fatal="${6:-}" t0=$SECONDS
  while ! "$check"; do
    if [ -n "$proc" ] && ! alive "$proc"; then
      alert "$proc exited while waiting for $desc"; show_log "$proc" 25; return 2
    fi
    if [ -n "$fatal" ] && log_since_start "$proc" | grep -qE "$fatal"; then
      return 3
    fi
    local el=$((SECONDS - t0))
    [ "$el" -ge "$timeout" ] && return 1
    if [ "$el" -gt 0 ] && [ $((el % 10)) -eq 0 ]; then info "... waiting for $desc (${el}s / ${timeout}s) $hint"; fi
    sleep 1
  done
  return 0
}

rx_rate() {  # bytes/s arriving on the LiDAR interface, over 1 s
  local f="/sys/class/net/$HOST_IF/statistics/rx_bytes" a b
  a=$(cat "$f" 2>/dev/null || echo 0); sleep 1; b=$(cat "$f" 2>/dev/null || echo 0)
  echo $((b - a))
}

lidar_diag() {  # what the network layer says about the LiDAR
  local r; r=$(rx_rate)
  info "diagnosis: ${r} B/s arriving on $HOST_IF (a streaming MID-360 is ~3,000,000 B/s)"
  info "diagnosis: neighbour entry: $(ip neigh show "$LIDAR_IP" 2>/dev/null | head -1)"
  local ports; ports=$(ss -uapn 2>/dev/null | grep -E ":($LIDAR_PORTS)\b" | sed 's/  */ /g')
  [ -n "$ports" ] && info "diagnosis: LiDAR UDP ports in use: $ports"
}

# ------------------------------------------------------------------ shutdown
CLEANED=0
cleanup() {
  [ "$CLEANED" = 1 ] && return
  CLEANED=1
  trap 'FORCE=1; _out "${R}[bringup] second stop request: killing everything now${D}" "force kill"' INT TERM HUP
  [ "$FAILED" = 1 ] || set_state stopping
  touch "$LOG_DIR/.shutdown"
  if [ "${#ORDER[@]}" -gt 0 ]; then
    step "Stopping (reverse order)"
    if group_alive glim && [ "$(hv wp_srv)" = 1 ]; then
      info "saving waypoints to $LOG_DIR/waypoints.yaml (GLIM forgets them when it stops)"
      rm -f "$LOG_DIR/waypoints.yaml"
      timeout 15 ros2 service call /save_waypoints waypoint_interfaces/srv/SaveWaypoints \
        "{path: $LOG_DIR/waypoints.yaml}" >>"$LOG_DIR/bringup.log" 2>&1
      # the call exits 0 even when the save failed, so look at the file itself
      local counts
      counts=$(python3 -c 'import sys, yaml; d = yaml.safe_load(open(sys.argv[1])) or {}
print(len(d.get("waypoints") or []), len(d.get("pending_waypoints") or []))' \
        "$LOG_DIR/waypoints.yaml" 2>/dev/null)
      if [ -n "$counts" ]; then
        set -- $counts
        ok "waypoints saved: $1 placed on the map, $2 pending"
        [ "$2" -gt 0 ] && warn "pending = GLIM had not finished their submap yet, so they have no map position"
      else
        warn "could not save waypoints"
      fi
    fi
    local i name
    for ((i = ${#ORDER[@]} - 1; i >= 0; i--)); do
      name="${ORDER[$i]}"
      [ "$name" = monitor ] && continue
      if group_alive "$name"; then info "stopping $name"; stop_proc "$name" 10; fi
    done
    stop_proc monitor 3
  fi
  if [ "$FAILED" = 1 ]; then
    info "logs: $LOG_DIR"
    if [ -t 0 ] && [ -z "${FORCE:-}" ]; then
      printf '%s' "${R}Start-up failed (see above). Press Enter to close this window.${D}"
      read -r _
    fi
  else
    set_state stopped
    info "stopped. Logs: $LOG_DIR"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# only one bring-up at a time (a second one would stop this one's nodes)
exec 9>"/tmp/kratos_bringup_$(id -u).lock"
if ! flock -n 9; then
  echo "${R}Another bringup.sh is already running (see ~/kratos_logs/latest). Stop it first (Ctrl+C in its window).${D}"
  [ -t 0 ] && { printf 'Press Enter to close.'; read -r _; }
  CLEANED=1; exit 1
fi
set_state starting

# ================================================================== 1. software
step "1/9 Software checks"
info "repo: $REPO   logs: $LOG_DIR"
[ -f /opt/ros/humble/setup.bash ] || fail "ROS 2 Humble not found at /opt/ros/humble"
KH="$(dirname "$REPO")"
SOURCED=()
src() { [ -f "$1" ] || return 1; source "$1"; SOURCED+=("$1"); }
src /opt/ros/humble/setup.bash
src "$REPO/install/setup.bash" || warn "$REPO/install/setup.bash missing: build the repo (README section 1.6)"
# Team-VM layout: runtime copies of the same code in separate workspaces
src "$KH/glim_ext_ws/install/setup.bash"
src "$KH/kratos_nav_ws/install/setup.bash"
src "$KH/pcd2pgm_live_ws/install/setup.bash"
# The patched GLIM (glim/glim_ros_fix) MUST be sourced last
FIX_WS=''
for d in "${GLIM_FIX_WS:-}" "$KH/glim_ros_fix_ws" "$HOME/glim_ros_fix_ws"; do
  [ -n "$d" ] && [ -f "$d/install/local_setup.bash" ] && { FIX_WS="$d"; break; }
done
[ -n "$FIX_WS" ] && src "$FIX_WS/install/local_setup.bash"
for f in "${SOURCED[@]}"; do info "sourced $f"; done

pkg_share() {  # first share/<pkg> on AMENT_PREFIX_PATH (the one ros2 will use)
  local p; IFS=: read -ra _P <<<"${AMENT_PREFIX_PATH:-}"
  for p in "${_P[@]}"; do [ -f "$p/share/$1/package.xml" ] && { echo "$p/share/$1"; return 0; }; done
  return 1
}
MISSING=()
for p in livox_ros_driver2 lidar_angle_filter kratos_nav pcd2pgm glim_ros waypoint_interfaces nav2_bringup; do
  pkg_share "$p" >/dev/null || MISSING+=("$p")
done
[ ${#MISSING[@]} -eq 0 ] || fail "packages not found: ${MISSING[*]}. Build: cd $REPO && colcon build --symlink-install --packages-select livox_ros_driver2 lidar_angle_filter waypoint_interfaces waypoint_manager glim_dump_export pcd2pgm kratos_nav --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release"
ok "all ROS packages found"
KN_SHARE="$(pkg_share kratos_nav)"
PCD_YAML="$(pkg_share pcd2pgm)/config/pcd2pgm_live.yaml"
NAV_PARAMS="${PARAMS:-$KN_SHARE/config/nav2_params.yaml}"
DRIVER_LAUNCH="$KN_SHARE/launch/livox_driver.launch.py"
LIDAR_IP_PY="$KN_SHARE/launch/lidar_ip.py"
MONITOR="$REPO/src/kratos_nav/scripts/bringup_monitor.py"
for f in "$DRIVER_LAUNCH" "$LIDAR_IP_PY" "$PCD_YAML" "$NAV_PARAMS" "$MONITOR" "$GLIM_CONFIG/config_ros.json"; do
  [ -f "$f" ] || fail "missing $f (after a git pull, rebuild kratos_nav: new files only appear in install/ after colcon build)"
done

# a git pull without a rebuild runs the OLD binaries silently: warn if sources are newer
stale() {  # stale SRC_DIR INSTALL_DIR: C++ sources edited after the last build
  [ -d "$1" ] && [ -d "$2" ] || return 1
  local newest_src newest_inst
  newest_src=$(find "$1" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' \) -printf '%T@\n' 2>/dev/null | sort -n | tail -1)
  newest_inst=$(find "$2" -printf '%T@\n' 2>/dev/null | sort -n | tail -1)
  [ -n "$newest_src" ] && [ -n "$newest_inst" ] && ge "$newest_src" "$newest_inst"
}
for p in lidar_angle_filter pcd2pgm livox_ros_driver2; do
  if stale "$REPO/src/$p" "$REPO/install/$p"; then
    warn "$p sources are newer than its build: rebuild it (colcon build --symlink-install --packages-select $p ...)"
  fi
done
stale "$REPO/glim/glim_ext_addon" "$REPO/install/waypoint_manager" && warn "waypoint_manager sources are newer than its build: rebuild it"

GLIM_SHARE="$(pkg_share glim_ros)"
if [[ "$GLIM_SHARE" == *glim_ros_fix_ws* ]]; then ok "patched GLIM overlay: $GLIM_SHARE"
else warn "GLIM is the UNPATCHED $GLIM_SHARE: /map can get phantom walls (glim/glim_ros_fix/README.md; or set GLIM_FIX_WS=/path/to/ws)"; fi

WPLIB=''
IFS=: read -ra _L <<<"${LD_LIBRARY_PATH:-}"
for d in "${_L[@]}"; do [ -f "$d/libwaypoint_manager.so" ] && { WPLIB="$d/libwaypoint_manager.so"; break; }; done
[ -n "$WPLIB" ] && ok "waypoint_manager: $WPLIB" || warn "libwaypoint_manager.so not on LD_LIBRARY_PATH: GLIM will run but waypoint tagging will not work"

if grep -q '"libstandard_viewer.so"' "$GLIM_CONFIG/config_ros.json" && [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  warn "GLIM config loads libstandard_viewer.so but there is no display: GLIM may fail to start"
fi
info "GLIM config: $GLIM_CONFIG"

# lidar_z invariant (AGENTS.md): obstacle band = ground+0.2 .. ground+1.8 in GLIM's frame
EXP_MIN=$(awk -v z="$LIDAR_Z" 'BEGIN{printf "%.2f", 0.2 - z}')
EXP_MAX=$(awk -v z="$LIDAR_Z" 'BEGIN{printf "%.2f", 1.8 - z}')
vals() { grep -E "^\s*$1:" "$2" | awk '{print $2}' | sort -u; }
BAD=''
for v in $(vals min_obstacle_height "$NAV_PARAMS") $(vals thre_z_min "$PCD_YAML"); do
  awk -v a="$v" -v b="$EXP_MIN" 'BEGIN{exit !((a-b)^2 < 1e-4)}' || BAD="$BAD min=$v"
done
for v in $(vals max_obstacle_height "$NAV_PARAMS") $(vals thre_z_max "$PCD_YAML"); do
  awk -v a="$v" -v b="$EXP_MAX" 'BEGIN{exit !((a-b)^2 < 1e-4)}' || BAD="$BAD max=$v"
done
[ -z "$BAD" ] || fail "lidar_z $LIDAR_Z needs obstacle heights $EXP_MIN / $EXP_MAX, found$BAD in $NAV_PARAMS (min/max_obstacle_height) or $PCD_YAML (thre_z_min/max). Fix them or pass the right --lidar-z."
ok "lidar_z $LIDAR_Z matches nav2_params.yaml and pcd2pgm_live.yaml ($EXP_MIN .. $EXP_MAX)"
info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-default}  (the GUI, drive.py and the micro-ROS agent must use the same domain)"
[ "${ROS_LOCALHOST_ONLY:-0}" = 1 ] && warn "ROS_LOCALHOST_ONLY=1: nodes on other computers (drive.py, laptops) cannot see this stack"

# ================================================================== 2. leftovers
step "2/9 Leftover processes"
LEFTOVER_RE='livox_ros_driver2_node|lidar_angle_filter_node|glim_rosnode|pcd2pgm_node|base_link_to_livox|rover_bridge\.py|waypoint_mission\.py|bringup_monitor\.py|nav2_bringup/launch|/(bt_navigator|controller_server|planner_server|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager)( |$)|launch (kratos_nav|lidar_angle_filter|livox_ros_driver2) |kratos_live\.rviz'
MY_ANCESTORS=" $$ "
p=$$
while [ "$p" -gt 1 ] 2>/dev/null; do
  p=$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null) || break
  MY_ANCESTORS="$MY_ANCESTORS$p "
done
leftovers() {  # stack processes, never a shell/terminal whose command line mentions them
  local p
  for p in $(pgrep -f -- "$LEFTOVER_RE"); do
    [[ "$MY_ANCESTORS" == *" $p "* ]] && continue
    case "$(ps -o comm= -p "$p" 2>/dev/null)" in
      ''|bash|sh|dash|zsh|fish|ssh|sshd|sudo|su|timeout|tmux*|screen|watch|*terminal*|xterm|konsole|less|vim|nano|grep) continue ;;
    esac
    echo "$p"
  done
}
mapfile -t LEFT < <(leftovers)
if [ ${#LEFT[@]} -gt 0 ]; then
  if [ "$CHECK_ONLY" = 1 ]; then
    warn "${#LEFT[@]} stack processes are running (a real start would stop them):"
    for p in "${LEFT[@]}"; do info "   $(ps -o pid=,args= -p "$p" | cut -c1-150)"; done
  else
    warn "stopping ${#LEFT[@]} leftover stack processes (a second driver/GLIM would break this run):"
    for p in "${LEFT[@]}"; do info "   $(ps -o pid=,args= -p "$p" | cut -c1-150)"; done
    kill -INT "${LEFT[@]}" 2>/dev/null
    for i in $(seq 1 40); do
      mapfile -t LEFT < <(leftovers); [ ${#LEFT[@]} -eq 0 ] && break; sleep 0.25
    done
    [ ${#LEFT[@]} -gt 0 ] && { kill -TERM "${LEFT[@]}" 2>/dev/null; sleep 2; }
    mapfile -t LEFT < <(leftovers)
    [ ${#LEFT[@]} -gt 0 ] && { kill -KILL "${LEFT[@]}" 2>/dev/null; sleep 1; }
    ok "leftovers stopped"
  fi
else
  ok "nothing left over"
fi
PORTS=$(ss -uapn 2>/dev/null | grep -E ":($LIDAR_PORTS)\b" | awk '{print "      " $4 "  " $6}')
if [ -n "$PORTS" ]; then
  if [ "$CHECK_ONLY" = 1 ]; then warn "LiDAR UDP ports in use:"; else alert "LiDAR UDP ports still in use:"; fi
  _out "$PORTS" "$PORTS"
  [ "$CHECK_ONLY" = 1 ] || fail "another program holds the LiDAR ports (a second driver?). Stop it, then start again."
fi

# ================================================================== 3. network
step "3/9 LiDAR network"
HOST_IF=$(ip -o -4 addr show | awk -v ip="$HOST_IP" '{split($4, a, "/")} a[1] == ip {print $2; exit}')
if [ -z "$HOST_IF" ] && [ -n "$IFACE" ]; then
  info "$HOST_IP is missing, adding it to $IFACE (sudo -n)"
  sudo -n ip addr add "$HOST_IP/24" dev "$IFACE" 2>&1 | sed 's/^/      /'
  HOST_IF=$(ip -o -4 addr show | awk -v ip="$HOST_IP" '{split($4, a, "/")} a[1] == ip {print $2; exit}')
fi
if [ -z "$HOST_IF" ]; then
  info "wired interfaces here:"; ip -br link | grep -vE '^(lo|docker|veth|br-|virbr|wl)' | sed 's/^/      /'
  fail "this computer does not have $HOST_IP. Add it to the interface the LiDAR cable is in: sudo ip addr add $HOST_IP/24 dev <interface>   (or rerun with --iface <interface>)"
fi
ok "$HOST_IP is on $HOST_IF"
# the LiDAR sends its data to the host IPs in the driver config: they must be this computer
DRV_CFG="$(pkg_share livox_ros_driver2)/config/MID360_config.json"
CFG_HOSTS=$(python3 -c 'import json, sys
h = json.load(open(sys.argv[1]))["MID360"]["host_net_info"]
print(" ".join(sorted({v for k, v in h.items() if k.endswith("_ip") and v})))' "$DRV_CFG" 2>&1)
if [ "$CFG_HOSTS" != "$HOST_IP" ]; then
  fail "the driver config $DRV_CFG sends LiDAR data to '$CFG_HOSTS', but this computer is $HOST_IP. Set every *_ip in host_net_info to $HOST_IP."
fi
if [ "$(cat "/sys/class/net/$HOST_IF/carrier" 2>/dev/null)" = 0 ]; then
  warn "$HOST_IF has no link (cable unplugged or LiDAR unpowered?)"
fi
IN_VM=0; systemd-detect-virt -q 2>/dev/null && IN_VM=1

if [ -n "$LIDAR_IP" ]; then
  info "using --lidar-ip $LIDAR_IP (no search)"
else
  info "searching for the MID-360 on 192.168.1.100-199 ..."
  t0=$SECONDS
  while :; do
    LIDAR_IP=$(python3 "$LIDAR_IP_PY" 2>>"$LOG_DIR/bringup.log")
    [ -n "$LIDAR_IP" ] && break
    [ "$CHECK_ONLY" = 1 ] && break
    if [ $((SECONDS - t0)) -ge 120 ]; then break; fi
    info "... no LiDAR answered yet ($((SECONDS - t0))s / 120s). It needs ~20 s after power-on. Check power and the cable."
    [ "$IN_VM" = 1 ] && info "    (VM: on the Mac, 'ifconfig en11 | grep status' must say active, then 'sudo ifconfig bridge100 addm en11')"
  done
fi
if [ -z "$LIDAR_IP" ]; then
  [ "$CHECK_ONLY" = 1 ] && warn "no LiDAR found right now" || fail "no MID-360 answered on 192.168.1.1xx. Check its power (green light), the cable, and that $HOST_IF is the port it is plugged into."
else
  ok "MID-360 at $LIDAR_IP"
fi
export LIVOX_LIDAR_IP="$LIDAR_IP"   # the driver launch uses it instead of searching again

if [ "$CHECK_ONLY" = 1 ]; then
  step "Pre-flight checks done (--check: nothing started)"
  set_state stopped; exit 0
fi

# ================================================================== 4. start
export RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
setsid "${SIGRESET[@]}" python3 "$MONITOR" --log-dir "$LOG_DIR" --status-every "$STATUS_EVERY" </dev/null &
PID[monitor]=$!; STARTED[monitor]=$SECONDS; ORDER+=(monitor)

lidar_ok()  { ge "$(hv lidar_hz)" 5; }
filt_ok()   { ge "$(hv filt_hz)" 5; }
tf_ok()     { [ "$(hv static_tf)" = 1 ]; }
pose_ok()   { local a; a=$(hv pose_age); [ -n "$a" ] && ge "$a" 0 && ! ge "$a" 3; }
wp_ok()     { [ "$(hv wp_srv)" = 1 ]; }
map_ok()    { local a; a=$(hv map_age); [ -n "$a" ] && ge "$a" 0; }
nav_ok()    { [ "$(hv nav_state)" = active ]; }

start_driver() { start_proc driver ros2 launch kratos_nav livox_driver.launch.py; }

DRIVER_FATAL='bind failed|Init lds lidar fail'
DRIVER_WAIT=60
step "4/9 LiDAR driver"
start_driver
for attempt in 1 2; do
  wait_for "points on /livox/lidar" "$DRIVER_WAIT" driver lidar_ok "(the driver needs ~25 s)" "$DRIVER_FATAL"
  rc=$?
  [ "$rc" = 0 ] && break
  if [ "$rc" = 3 ]; then
    show_log driver 12
    fail "the driver could not open its UDP ports on $HOST_IP ('bind failed'): another Livox driver is running, or $HOST_IP was removed from $HOST_IF."
  fi
  lidar_diag
  [ "$rc" = 2 ] && fail "the LiDAR driver crashed at start-up (log above)"
  if [ "$attempt" = 1 ] && [ "$(rx_rate)" -ge 50000 ]; then
    warn "data arrives but the driver publishes no points after ${DRIVER_WAIT} s: restarting the driver once"
    stop_proc driver 5; start_driver
    continue
  fi
  show_log driver 12
  fail "no points from the LiDAR. With ~0 B/s arriving it's the cable, power or network, not the software: check the green light, the cable into $HOST_IF, then start again."
done
ok "LiDAR streaming: $(hv lidar_hz) Hz, IMU $(hv imu_hz) Hz"

step "5/9 Angle filter (masks the rover body, drops blocked scans)"
start_proc filter ros2 launch lidar_angle_filter angle_filter.launch.py
wait_for "/livox/lidar_filtered" 30 filter filt_ok "(if the LiDAR is covered, the scan gate drops every scan)" \
  || { show_log filter; fail "angle filter is not publishing"; }
ok "filtered cloud: $(hv filt_hz) Hz"

step "6/9 Nav2 + base_link->livox_frame TF (must be up before GLIM)"
NAV_ARGS=(lidar_z:="$LIDAR_Z" lidar_x:="$LIDAR_X")
[ -n "$PARAMS" ] && NAV_ARGS+=(params_file:="$PARAMS")
start_proc nav2 ros2 launch kratos_nav nav.launch.py "${NAV_ARGS[@]}"
wait_for "static TF base_link->livox_frame" 30 nav2 tf_ok || { show_log nav2; fail "nav.launch.py did not publish the base_link->livox_frame TF"; }
ok "TF base_link->livox_frame published (Nav2 finishes activating once GLIM gives a pose)"

step "7/9 GLIM"
_out "${Y}>>> KEEP THE ROVER STILL for the next ~10 s (IMU initialization) <<<${D}" ">>> KEEP STILL"
start_proc glim ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$(readlink -f "$GLIM_CONFIG")"
wait_for "GLIM pose (TF map->base_link)" 120 glim pose_ok "(needs the IMU and points; keep still)" \
  || { show_log glim 25; fail "GLIM gives no pose"; }
ok "GLIM running, pose $(hv pose). You can move now."
GPID=$(pgrep -g "${PID[glim]}" -x glim_rosnode | head -1)
if [ -n "$GPID" ]; then   # which rviz_viewer did the running GLIM actually load?
  if grep -q 'glim_ros_fix_ws.*librviz_viewer' "/proc/$GPID/maps" 2>/dev/null; then ok "GLIM loaded the patched rviz_viewer"
  else warn "GLIM did not load the patched rviz_viewer from glim_ros_fix_ws (phantom-wall risk in /map)"; fi
fi
if wait_for "waypoint services (/add_waypoint)" 30 glim wp_ok; then ok "waypoint tagging available"
else warn "no /add_waypoint service: waypoint_manager did not load (tagging will not work). See glim.log"; fi

step "8/9 pcd2pgm (/glim_ros/map -> /map) + rover_bridge"
start_proc pcd2pgm ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file "$PCD_YAML"
start_bridge() {
  start_proc bridge ros2 run kratos_nav rover_bridge.py --ros-args \
    -p track_width:="$TRACK_WIDTH" -p max_wheel_speed:="$MAX_WHEEL_SPEED"
}
start_bridge
if wait_for "first /map" 45 pcd2pgm map_ok "(it comes after GLIM finishes a submap: moving a few metres helps)"; then ok "/map received"
else warn "no /map yet. It appears once GLIM has a submap; the monitor will say 'first /map' when it arrives"; fi
sleep 1
alive bridge && ok "rover_bridge running in MANUAL (joystick drives; set_auto true hands the wheels to Nav2). drive.py must run with -r /rover:=/rover_joy" \
  || { show_log bridge; warn "rover_bridge exited"; }

step "9/9 RViz + Nav2 activation"
if [ "$RVIZ" = auto ] && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v rviz2 >/dev/null; then
  RVIZ_CFG="$KN_SHARE/config/kratos_live.rviz"
  [ -f "$RVIZ_CFG" ] || RVIZ_CFG="$(pkg_share nav2_bringup)/rviz/nav2_default_view.rviz"
  start_proc rviz rviz2 -d "$RVIZ_CFG"
  ok "RViz started ($RVIZ_CFG)"
else
  info "RViz not started (no display, or --no-rviz)"
fi
if wait_for "Nav2 bt_navigator active" 90 nav2 nav_ok; then ok "Nav2 active"
else warn "Nav2 is not active yet (state: $(hv nav_state)). Look for [nav2] errors above"; fi

set_state ready
step "READY"
cat <<EOF | while IFS= read -r l; do _out "$l" "$l"; done
  LiDAR $LIDAR_IP  |  lidar_z $LIDAR_Z  |  logs $LOG_DIR
  Tag (stand still 2-3 s first): ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
  List:      ros2 service call /list_waypoints waypoint_interfaces/srv/ListWaypoints "{}"
  Mission:   ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
  AUTO/MAN:  ros2 service call /rover_bridge/set_auto std_srvs/srv/SetBool "{data: true}"   (false = MANUAL)
  Clear costmaps: ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
  Stop everything: Ctrl+C here (waypoints are saved to the log folder first)
EOF

# ================================================================== supervise
MAX_RESTARTS=4        # per stage, within RESTART_WINDOW seconds
RESTART_WINDOW=600
declare -A RESTART_T GAVE_UP
NOTED_DEAD=''
LINK_DOWN_SINCE=''
last_link_msg=0
restart_proc() {  # restart a stateless stage
  local name="$1"
  [ -n "${GAVE_UP[$name]:-}" ] && return
  if [ $((SECONDS - ${RESTART_T[$name]:-0})) -gt "$RESTART_WINDOW" ]; then RESTARTS[$name]=0; fi
  RESTART_T[$name]=$SECONDS
  RESTARTS[$name]=$(( ${RESTARTS[$name]:-0} + 1 ))
  if [ "${RESTARTS[$name]}" -gt "$MAX_RESTARTS" ]; then
    GAVE_UP[$name]=1
    alert "$name failed $MAX_RESTARTS times in $((RESTART_WINDOW / 60)) min, not restarting it again. Check $LOG_DIR/$name.log, then restart the bring-up."
    return
  fi
  warn "restarting $name (${RESTARTS[$name]}/$MAX_RESTARTS in $((RESTART_WINDOW / 60)) min)"
  stop_proc "$name" 5
  case "$name" in
    driver) start_driver ;;
    filter) start_proc filter ros2 launch lidar_angle_filter angle_filter.launch.py ;;
    pcd2pgm) start_proc pcd2pgm ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file "$PCD_YAML" ;;
    bridge) start_bridge; alert "rover_bridge restarted: it is back in MANUAL" ;;
    rviz) start_proc rviz rviz2 -d "$RVIZ_CFG" ;;
    nav2) start_proc nav2 ros2 launch kratos_nav nav.launch.py "${NAV_ARGS[@]}" ;;
  esac
}

while :; do
  sleep 1
  degraded=0
  for name in driver filter nav2 glim pcd2pgm bridge rviz monitor; do
    [ -n "${PID[$name]:-}" ] || continue
    if ! alive "$name"; then
      degraded=1
      case "$name" in
        glim)
          if [[ "$NOTED_DEAD" != *" glim "* ]]; then
            NOTED_DEAD="$NOTED_DEAD glim "
            alert "GLIM exited. The map, the pose and ALL WAYPOINTS are gone. Not restarting automatically. To start over: Ctrl+C here, then start the bring-up again."
            show_log glim 25
          fi ;;
        monitor)
          if [[ "$NOTED_DEAD" != *" monitor "* ]]; then NOTED_DEAD="$NOTED_DEAD monitor "; alert "health monitor exited: no more status lines or LiDAR stall detection (the stack itself keeps running)"; fi ;;
        rviz)
          [ -n "${GAVE_UP[rviz]:-}" ] && continue
          wait "${PID[rviz]}" 2>/dev/null; rc=$?
          if [ "$rc" = 0 ]; then   # window closed on purpose: leave it closed
            info "RViz was closed (not reopening it). Everything else keeps running."
            unset 'PID[rviz]'
          else
            alert "RViz crashed (exit $rc)"; show_log rviz 10; restart_proc rviz
          fi ;;
        *)
          [ -n "${GAVE_UP[$name]:-}" ] && continue
          alert "$name exited"; show_log "$name" 15
          restart_proc "$name" ;;
      esac
    fi
  done

  # LiDAR stall: a link problem (no bytes arriving) vs a stuck driver (bytes arrive, no points)
  la=$(hv lidar_age)
  if alive driver && [ -n "$la" ] && ge "$la" 5 && [ $((SECONDS - ${STARTED[driver]})) -gt 45 ]; then
    degraded=1
    r=$(rx_rate)
    if [ "$r" -lt 50000 ]; then
      [ -z "$LINK_DOWN_SINCE" ] && LINK_DOWN_SINCE=$SECONDS
      if [ $((SECONDS - last_link_msg)) -ge 15 ]; then
        last_link_msg=$SECONDS
        alert "no LiDAR data reaching $HOST_IF for $((SECONDS - LINK_DOWN_SINCE))s (${r} B/s): cable, power or network. Restarting the driver won't help; it will be restarted when data flows again."
        [ "$IN_VM" = 1 ] && info "    (VM: is the Mac's en11 still active and in bridge100?)"
      fi
    else
      LINK_DOWN_SINCE=''
      if ge "$la" 8; then
        alert "LiDAR data arrives (${r} B/s) but the driver publishes nothing for ${la}s: restarting the driver"
        restart_proc driver
      fi
    fi
  else
    LINK_DOWN_SINCE=''
  fi

  if [ "$degraded" = 1 ]; then set_state degraded; else set_state ready; fi
done
