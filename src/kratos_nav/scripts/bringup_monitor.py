#!/usr/bin/env python3
"""Health monitor + log relay for bringup.sh (repo root). Not meant to be run by hand.

It does three jobs in the bring-up terminal:
  1. Relay: tails every <name>.log in the log directory and prints the lines that
     matter (errors, warnings, key events), with a [name] prefix. Repeats are
     collapsed so a spamming node can't flood the terminal.
  2. Health file: every second writes <log_dir>/health (key=value lines) that
     bringup.sh reads to decide when a stage is up and when the LiDAR has stalled.
  3. Alerts + status: prints an ALERT the moment something goes wrong (LiDAR silent,
     GLIM pose stopped or jumped, /map stale, Nav2 not active, rover sitting on a
     lethal costmap cell, two /rover publishers) and a one-line status every N s.

Ages are measured on this computer's clock when a message ARRIVES, so they are right
even if the LiDAR's timestamps are in another time base.
"""
import argparse
import math
import os
import re
import sys
import threading
import time
from collections import deque

COLORS = sys.stdout.isatty()


def c(code, text):
    return f'\033[{code}m{text}\033[0m' if COLORS else text


SOURCE_COLOR = {'driver': '36', 'filter': '36', 'glim': '35', 'nav2': '34',
                'pcd2pgm': '33', 'rviz': '37', 'bridge': '32', 'mission': '32;1',
                'cmd': '32'}

# name -> (regex of lines worth showing (None = all), seconds to collapse repeats)
RELAY = {
    'driver': (r'error|fail|warn|MID-360 found|success|exception|already in use|'
               r'process has died', 10),
    'filter': (r'\[(WARN|ERROR|FATAL)\]|process has died|exception', 10),
    'glim': (r'\[(warning|error|critical)\]|exception|abort|terminate|what\(\)|waypoint|'
             r'segmentation|core dumped|process has died', 10),
    'nav2': (r'\[(ERROR|FATAL)\]|Aborting|Managed nodes are active|process has died|'
             r'waiting for transform|failed to create plan|Failed to make progress|'
             r'exception', 30),
    'pcd2pgm': (r'\[(WARN|ERROR|FATAL)\]|Live map update took|keeping the previous|'
                r'process has died|exception', 60),
    'rviz': (r'\[(ERROR|FATAL)\]|process has died', 60),
    'bridge': (None, 2),
    'mission': (None, 2),
    'cmd': (None, 0),
}
# also skipped: GLIM's 10 s autosave, and nodes dying from our own INT/TERM
SKIP_ALWAYS = re.compile(r'^\s*$|^requester: making request|^waiting for service|^----- |'
                         r'waypoints_autosave\.yaml|process has died .*exit code -(2|15),')


def ts():
    return time.strftime('%H:%M:%S')


class Out:
    """Prints whole lines (safe to interleave with bringup.sh) and keeps a copy."""

    def __init__(self, log_dir):
        self.lock = threading.Lock()
        self.copy = open(os.path.join(log_dir, 'bringup.log'), 'a')

    def line(self, text, plain=None):
        with self.lock:
            print(text, flush=True)
            self.copy.write(f'{ts()} {plain if plain is not None else text}\n')
            self.copy.flush()


class Relay(threading.Thread):
    def __init__(self, log_dir, out):
        super().__init__(daemon=True)
        self.log_dir, self.out = log_dir, out
        self.files = {}          # path -> [fh, partial]
        self.last = {}           # (name, key) -> [time printed, suppressed count]

    def run(self):
        while True:
            try:
                self.scan()
            except Exception as e:  # never let the relay die
                self.out.line(c('33', f'[monitor] relay error: {e}'))
            time.sleep(0.3)

    def scan(self):
        for fn in os.listdir(self.log_dir):
            if not fn.endswith('.log') or fn == 'bringup.log':
                continue
            path = os.path.join(self.log_dir, fn)
            if path not in self.files:
                self.files[path] = [open(path, errors='replace'), '']
            fh, partial = self.files[path]
            chunk = fh.read()
            if not chunk:
                continue
            lines = (partial + chunk).split('\n')
            self.files[path][1] = lines.pop()
            name = fn[:-4].split('_')[0]
            for ln in lines:
                self.show(name, ln.rstrip())

    def show(self, name, ln):
        ln = re.sub(r'\x1b\[[0-9;]*m', '', ln)
        if SKIP_ALWAYS.search(ln):
            return
        # once bringup.sh is stopping things, a node dying (even a Nav2 segfault on
        # exit, common in Humble) is expected and not worth an ERROR line
        if 'process has died' in ln and os.path.exists(os.path.join(self.log_dir, '.shutdown')):
            return
        pattern, window = RELAY.get(name, (None, 5))
        if pattern and not re.search(pattern, ln, re.IGNORECASE):
            return
        key = (name, re.sub(r'\d+(\.\d+)?', '#', ln))
        now = time.monotonic()
        prev = self.last.get(key)
        if prev and now - prev[0] < window:
            prev[1] += 1
            return
        extra = f'  (+{prev[1]} similar)' if prev and prev[1] else ''
        self.last[key] = [now, 0]
        if len(ln) > 300:
            ln = ln[:300] + ' ...'
        tag = f'[{name}]'
        self.out.line(f'{c(SOURCE_COLOR.get(name, "37"), tag)} {ln}{extra}', f'{tag} {ln}{extra}')


class Rate:
    def __init__(self):
        self.stamps = deque(maxlen=400)
        self.last = None

    def tick(self, *_):
        self.last = time.monotonic()
        self.stamps.append(self.last)

    def hz(self, window=3.0):
        now = time.monotonic()
        return sum(1 for t in self.stamps if now - t <= window) / window

    def age(self):
        return -1.0 if self.last is None else time.monotonic() - self.last


def run_monitor(args, out):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from rclpy.time import Time
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import OccupancyGrid, Path
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import GetState
    import tf2_ros

    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Monitor(Node):
        def __init__(self):
            super().__init__('bringup_monitor')
            self.lidar, self.imu, self.filt, self.cmd = Rate(), Rate(), Rate(), Rate()
            # raw=True: count clouds without deserializing 20k points 20 times a second
            self.create_subscription(PointCloud2, '/livox/lidar', self.lidar.tick,
                                     qos_profile_sensor_data, raw=True)
            self.create_subscription(PointCloud2, '/livox/lidar_filtered', self.filt.tick,
                                     qos_profile_sensor_data, raw=True)
            self.create_subscription(Imu, '/livox/imu', self.imu.tick,
                                     qos_profile_sensor_data, raw=True)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                     self.on_costmap, latched)
            self.create_subscription(Path, '/plan', self.on_plan, 10)
            self.tf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf, self)
            self.nav_cli = self.create_client(GetState, '/bt_navigator/get_state')

            self.map = None; self.map_rx = None; self.map_n = 0
            self.cost = None
            self.plan_rx = None; self.plan_len = 0.0
            self.last_cmd = (0.0, 0.0)
            self.pose = None; self.pose_stamp = None; self.pose_rx = None
            self.static_tf = False
            self.nav_state = 'unknown'; self.nav_pending = False; self.nav_ever_active = False
            self.wp_srv = False; self.rover_pubs = 0
            self.alerts = {}                 # key -> currently bad?
            self.lidar_seen = self.pose_seen = self.map_seen = False
            self.started = time.monotonic()
            self.last_status = time.monotonic()
            self.last_slow = 0.0
            self.create_timer(1.0, self.tick)

        # ---- callbacks ----
        def on_cmd(self, msg):
            self.cmd.tick()
            self.last_cmd = (msg.linear.x, msg.angular.z)

        def on_map(self, msg):
            self.map, self.map_rx = msg, time.monotonic()
            self.map_n += 1
            if self.map_n == 1:
                occ = msg.data.count(100) if hasattr(msg.data, 'count') else 0
                out.line(c('32', f'[monitor] first /map: {msg.info.width}x{msg.info.height} '
                                 f'@ {msg.info.resolution:.2f} m, {occ} occupied cells'))

        def on_costmap(self, msg):
            self.cost = msg

        def on_plan(self, msg):
            self.plan_rx = time.monotonic()
            p = msg.poses
            self.plan_len = sum(math.dist((p[i].pose.position.x, p[i].pose.position.y),
                                          (p[i + 1].pose.position.x, p[i + 1].pose.position.y))
                                for i in range(len(p) - 1))

        # ---- helpers ----
        def alert(self, key, bad, text, fixed_text=None):
            was = self.alerts.get(key, False)
            if bad and not was:
                out.line(c('31;1', f'!! ALERT {ts()} {text}'), f'!! ALERT {text}')
            elif was and not bad and fixed_text:
                out.line(c('32', f'OK {ts()} {fixed_text}'), f'OK {fixed_text}')
            self.alerts[key] = bad

        def cell(self, grid, x, y):
            info = grid.info
            ix = int((x - info.origin.position.x) / info.resolution)
            iy = int((y - info.origin.position.y) / info.resolution)
            if 0 <= ix < info.width and 0 <= iy < info.height:
                return grid.data[iy * info.width + ix]
            return None

        def rover_cost(self):
            """Worst costmap value within 0.3 m of base_link, and the /map value there."""
            if self.cost is None or self.pose is None:
                return 'none', None
            x, y = self.pose[0], self.pose[1]
            r = self.cost.info.resolution
            worst = -1
            n = int(0.3 / r)
            for dx in range(-n, n + 1):
                for dy in range(-n, n + 1):
                    if dx * dx + dy * dy <= n * n:
                        v = self.cell(self.cost, x + dx * r, y + dy * r)
                        if v is not None:
                            worst = max(worst, v)
            label = ('lethal' if worst >= 100 else 'inscribed' if worst >= 99 else
                     'unknown' if worst < 0 else 'free')
            static = self.cell(self.map, x, y) if self.map is not None else None
            return label, static

        # ---- 1 Hz ----
        def tick(self):
            try:
                self.update()
                self.write_health()
                self.check_alerts()
                self.maybe_status()
            except Exception as e:
                out.line(c('33', f'[monitor] error: {e}'))

        def update(self):
            now = time.monotonic()
            try:
                t = self.tf.lookup_transform('map', 'base_link', Time())
                stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
                tr = t.transform.translation
                newpose = (tr.x, tr.y, tr.z)
                if stamp != self.pose_stamp:
                    if self.pose is not None and self.pose_rx is not None:
                        jump = math.dist(self.pose[:2], newpose[:2])
                        dt = max(now - self.pose_rx, 1e-3)
                        if jump > 1.5 and jump / dt > 1.5:
                            out.line(c('31;1', f'!! ALERT {ts()} GLIM pose jumped {jump:.1f} m in '
                                               f'{dt:.1f} s: divergence or a big loop closure. '
                                               f'Check RViz: if the map smeared, GLIM must be restarted.'),
                                     f'!! ALERT GLIM pose jumped {jump:.1f} m in {dt:.1f} s')
                    self.pose, self.pose_stamp, self.pose_rx = newpose, stamp, now
                    self.pose_seen = True
            except tf2_ros.TransformException:
                pass
            if not self.static_tf:
                self.static_tf = self.tf.can_transform('base_link', 'livox_frame', Time())
            if now - self.last_slow >= 3.0:
                self.last_slow = now
                names = {n for n, _ in self.get_service_names_and_types()}
                self.wp_srv = '/add_waypoint' in names
                self.rover_pubs = self.count_publishers('/rover')
                if self.nav_cli.service_is_ready() and not self.nav_pending:
                    self.nav_pending = True
                    fut = self.nav_cli.call_async(GetState.Request())
                    fut.add_done_callback(self.on_nav_state)
                elif not self.nav_cli.service_is_ready():
                    self.nav_state = 'not running'
            if self.lidar.last is not None:
                self.lidar_seen = True
            if self.map_rx is not None:
                self.map_seen = True

        def on_nav_state(self, fut):
            self.nav_pending = False
            try:
                self.nav_state = fut.result().current_state.label
                if self.nav_state == 'active':
                    self.nav_ever_active = True
            except Exception:
                self.nav_state = 'unknown'

        def ages(self):
            now = time.monotonic()
            return {
                'pose_age': -1.0 if self.pose_rx is None else now - self.pose_rx,
                'map_age': -1.0 if self.map_rx is None else now - self.map_rx,
                'plan_age': -1.0 if self.plan_rx is None else now - self.plan_rx,
            }

        def write_health(self):
            a = self.ages()
            cost, static = self.rover_cost()
            kv = {
                'lidar_hz': f'{self.lidar.hz():.1f}', 'lidar_age': f'{self.lidar.age():.1f}',
                'imu_hz': f'{self.imu.hz():.0f}', 'filt_hz': f'{self.filt.hz():.1f}',
                'filt_age': f'{self.filt.age():.1f}', 'static_tf': int(self.static_tf),
                'pose_age': f'{a["pose_age"]:.1f}',
                'pose': 'none' if self.pose is None else '%.2f,%.2f,%.2f' % self.pose,
                'map_age': f'{a["map_age"]:.1f}', 'nav_state': self.nav_state.replace(' ', '_'),
                'wp_srv': int(self.wp_srv), 'cost_rover': cost,
                'map_rover': 'none' if static is None else static,
                'rover_pubs': self.rover_pubs, 'plan_age': f'{a["plan_age"]:.1f}',
                'cmd_hz': f'{self.cmd.hz():.1f}',
            }
            tmp = os.path.join(args.log_dir, '.health.tmp')
            with open(tmp, 'w') as f:
                f.write(''.join(f'{k}={v}\n' for k, v in kv.items()))
            os.replace(tmp, os.path.join(args.log_dir, 'health'))

        def check_alerts(self):
            if os.path.exists(os.path.join(args.log_dir, '.shutdown')):
                return
            a = self.ages()
            la = self.lidar.age()
            self.alert('lidar', self.lidar_seen and la > 2.0,
                       f'LiDAR /livox/lidar silent for {la:.0f}s. GLIM is blind: if this lasts, '
                       'its pose can diverge. (bringup.sh checks the link and restarts the driver.)',
                       f'LiDAR back ({self.lidar.hz():.1f} Hz)')
            self.alert('filter', self.lidar_seen and la < 1.0 and self.filt.last is not None
                       and self.filt.age() > 3.0,
                       'angle filter stopped publishing while the LiDAR is fine: scan gate dropping '
                       'scans (LiDAR covered?) or the filter died.', 'angle filter publishing again')
            self.alert('pose', self.pose_seen and a['pose_age'] > 3.0,
                       f'GLIM pose (TF map->base_link) not updated for {a["pose_age"]:.0f}s: '
                       'GLIM stalled, crashed, or gets no scans.', 'GLIM pose updating again')
            self.alert('map', self.map_seen and a['map_age'] > 45.0,
                       f'/map not updated for {a["map_age"]:.0f}s: pcd2pgm down, or GLIM stopped '
                       'publishing /glim_ros/map.', '/map updating again')
            self.alert('nav', self.nav_ever_active and self.nav_state != 'active',
                       f'Nav2 bt_navigator is "{self.nav_state}" (was active).', 'Nav2 active again')
            cost, static = self.rover_cost()
            why = ('/map is FREE there, so the obstacle layer (live LiDAR points near the rover) '
                   'is marking it: check lidar_z vs min_obstacle_height, then clear the costmap '
                   '(command in the READY box).'
                   if static == 0 else f'/map value there: {static}.')
            self.alert('cost', cost == 'lethal',
                       f'rover is on a LETHAL global-costmap cell. A mission started now aborts '
                       f'(status 6). {why}', 'rover is off lethal costmap cells')
            self.alert('rover', self.rover_pubs > 1,
                       f'/rover has {self.rover_pubs} publishers: drive.py is not remapped to '
                       '/rover_joy, the wheels will stutter.', '/rover has one publisher again')

        def maybe_status(self):
            flag = os.path.join(args.log_dir, '.status_now')
            now_flag = os.path.exists(flag)
            if now_flag:
                os.remove(flag)
            if os.path.exists(os.path.join(args.log_dir, '.shutdown')):
                return
            if not now_flag and time.monotonic() - self.last_status < args.status_every:
                return
            self.last_status = time.monotonic()
            a = self.ages()

            def g(ok, text):
                return c('32' if ok else '31', text)

            lh, fh = self.lidar.hz(), self.filt.hz()
            pose = ('no pose' if self.pose is None else
                    'pose %.1f,%.1f,%.1f (%.0fs)' % (*self.pose, a['pose_age']))
            mapt = 'no /map' if self.map is None else f'/map {a["map_age"]:.0f}s ago'
            cost, _ = self.rover_cost()
            plan = '' if self.plan_rx is None else f' | plan {self.plan_len:.1f} m {a["plan_age"]:.0f}s ago'
            cmd = '' if self.cmd.last is None or self.cmd.age() > 5 else \
                f' | cmd_vel {self.last_cmd[0]:.2f} m/s {self.last_cmd[1]:.2f} rad/s'
            parts = [
                g(lh > 5, f'lidar {lh:.1f}Hz'), g(self.imu.hz() > 50, f'imu {self.imu.hz():.0f}Hz'),
                g(fh > 5, f'filtered {fh:.1f}Hz'),
                g(self.pose is not None and a['pose_age'] < 3, pose),
                g(self.map is not None and a['map_age'] < 45, mapt),
                g(self.nav_state == 'active', f'nav2 {self.nav_state}'),
                g(self.wp_srv, 'waypoints ' + ('ready' if self.wp_srv else 'n/a')),
                g(cost in ('free', 'none'), f'costmap@rover {cost}'),
            ]
            text = f'[status {ts()}] ' + ' | '.join(parts) + plan + cmd
            out.line(text, re.sub(r'\x1b\[[0-9;]*m', '', text))

    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log-dir', required=True)
    ap.add_argument('--status-every', type=float, default=15.0)
    args = ap.parse_args()
    out = Out(args.log_dir)
    Relay(args.log_dir, out).start()
    try:
        run_monitor(args, out)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Ctrl+C inside rclpy.spin surfaces as ExternalShutdownException: normal stop
        if os.path.exists(os.path.join(args.log_dir, '.shutdown')) or \
                type(e).__name__ == 'ExternalShutdownException':
            return
        # otherwise keep relaying logs even if ROS introspection is broken
        out.line(c('31;1', f'[monitor] health monitor failed ({type(e).__name__}: {e}); '
                           'log relay still running'))
        while True:
            time.sleep(1)


if __name__ == '__main__':
    main()
