#!/usr/bin/env python3
"""Drive to GLIM waypoints, one at a time, with Nav2.

Why not Nav2's own follow_waypoints?
  follow_waypoints takes a list of poses ONCE and never looks at them again.
  GLIM's waypoint_manager stores every waypoint relative to a submap, and
  loop closures move submaps. So the same waypoint's map pose can shift while
  we drive. This node re-asks GLIM (/get_waypoint) every refresh_period
  seconds and, if the waypoint moved more than replan_threshold, re-sends the
  goal. Nav2 preempts the old goal and replans.

Pose conversion:
  /get_waypoint returns the LiDAR (livox_frame) pose in map. Nav2 wants
  base_link. We look up the static livox_frame->base_link TF, apply it, and
  flatten to x, y, yaw (z, roll, pitch zeroed) because Nav2 is 2D.

Usage:
  ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
  (no waypoints param = every waypoint GLIM lists, pending ones included:
   bound ones in tag order, then pending ones)
"""
import math
import signal
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from waypoint_interfaces.srv import GetWaypoint, ListWaypoints


PENDING_SUFFIX = ' (pending)'


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        self.declare_parameter('waypoints', [''])
        self.declare_parameter('get_waypoint_service', '/get_waypoint')
        self.declare_parameter('list_waypoints_service', '/list_waypoints')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'livox_frame')
        self.declare_parameter('refresh_period', 2.0)
        self.declare_parameter('replan_threshold', 0.3)
        # A waypoint tagged moments ago stays "pending" inside GLIM until its
        # submap is finalized (needs a bit more driving). Wait this long for it.
        self.declare_parameter('wait_for_waypoint_sec', 30.0)
        self.declare_parameter('stop_on_failure', True)

        self.get_wp = self.create_client(GetWaypoint, self.get_parameter('get_waypoint_service').value)
        self.list_wp = self.create_client(ListWaypoints, self.get_parameter('list_waypoints_service').value)
        self.nav = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_handle = None
        # Goal requests Nav2 did not answer in time. If Nav2 accepts one late,
        # it gets cancelled so the rover never drives to a goal nobody tracks.
        self.unanswered = []

    # ---- small blocking helpers (this node is only spun from here) ----------

    def wait(self, future, timeout_sec):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def call(self, client, request, timeout_sec=3.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'service {client.srv_name} not available (is GLIM running?)')
            return None
        return self.wait(client.call_async(request), timeout_sec)

    # ---- waypoint -> Nav2 goal ----------------------------------------------

    def sensor_to_base_offset(self):
        """(x, y, yaw) of base_link expressed in livox_frame, from the static TF."""
        sensor = self.get_parameter('sensor_frame').value
        base = self.get_parameter('base_frame').value
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                t = self.tf_buffer.lookup_transform(sensor, base, Time())
                tr = t.transform.translation
                return tr.x, tr.y, yaw_from_quat(t.transform.rotation)
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(f'no TF {sensor} -> {base}; is nav.launch.py running?')
        return None

    def resolve(self, name):
        """Ask GLIM for the waypoint NOW and return a flat base_link goal, or None."""
        res = self.call(self.get_wp, GetWaypoint.Request(name=name))
        if res is None or not res.found:
            return None
        offset = self.sensor_to_base_offset()
        if offset is None:
            return None

        # T_map_base = T_map_sensor * T_sensor_base, done in 2D.
        p = res.pose.pose.position
        yaw = yaw_from_quat(res.pose.pose.orientation)
        ox, oy, oyaw = offset
        base_yaw = yaw + oyaw
        goal = PoseStamped()
        goal.header.frame_id = res.pose.header.frame_id or 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = p.x + math.cos(yaw) * ox - math.sin(yaw) * oy
        goal.pose.position.y = p.y + math.sin(yaw) * ox + math.cos(yaw) * oy
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(base_yaw / 2.0)
        goal.pose.orientation.w = math.cos(base_yaw / 2.0)
        return goal

    def resolve_with_wait(self, name):
        deadline = self.get_clock().now() + Duration(seconds=self.get_parameter('wait_for_waypoint_sec').value)
        warned = False
        while rclpy.ok():
            goal = self.resolve(name)
            if goal is not None:
                return goal
            if self.get_clock().now() > deadline:
                return None
            if not warned:
                self.get_logger().warn(f"'{name}' not resolvable yet (pending submap?) - waiting")
                warned = True
            rclpy.spin_once(self, timeout_sec=1.0)
        return None

    def send(self, goal):
        """Send goal; returns (handle, result_future) or (None, None)."""
        request = self.nav.send_goal_async(NavigateToPose.Goal(pose=goal))
        handle = self.wait(request, 5.0)
        if handle is None:
            self.get_logger().error('Nav2 did not answer the goal request within 5 s')
            self.unanswered.append(request)
            return None, None
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return None, None
        self.current_handle = handle
        return handle, handle.get_result_async()

    # ---- mission ------------------------------------------------------------

    def go_to(self, name):
        goal = self.resolve_with_wait(name)
        if goal is None:
            self.get_logger().error(f"waypoint '{name}' not found in GLIM")
            return False

        self.get_logger().info(
            f"-> '{name}' at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        handle, result_future = self.send(goal)
        if handle is None:
            return False

        period = Duration(seconds=self.get_parameter('refresh_period').value)
        threshold = self.get_parameter('replan_threshold').value
        next_refresh = self.get_clock().now() + period

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.cancel_late_goals()

            if result_future.done():
                status = result_future.result().status
                self.current_handle = None
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f"reached '{name}'")
                    return True
                self.get_logger().error(f"failed '{name}' (status {status})")
                return False

            if self.get_clock().now() >= next_refresh:
                next_refresh = self.get_clock().now() + period
                fresh = self.resolve(name)
                if fresh is None:
                    continue  # transient service hiccup: keep the current goal
                moved = math.hypot(fresh.pose.position.x - goal.pose.position.x,
                                   fresh.pose.position.y - goal.pose.position.y)
                if moved > threshold:
                    self.get_logger().warn(
                        f"'{name}' moved {moved:.2f} m (loop closure) - re-sending goal")
                    # Sending a new goal preempts the old one inside Nav2; we
                    # simply stop listening to the old result future.
                    new_handle, new_future = self.send(fresh)
                    if new_handle is not None:
                        goal, handle, result_future = fresh, new_handle, new_future
        return False

    def waypoint_names(self):
        names = [n for n in self.get_parameter('waypoints').value if n]
        if names:
            return names
        res = self.call(self.list_wp, ListWaypoints.Request())
        if res is None:
            return []
        # waypoint_manager lists not-yet-bound tags as "<name> (pending)";
        # /get_waypoint only knows the bare name.
        return [n[:-len(PENDING_SUFFIX)] if n.endswith(PENDING_SUFFIX) else n for n in res.names]

    def run(self):
        if not self.nav.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Nav2 navigate_to_pose not available (is nav.launch.py up and active?)')
            return 1

        names = self.waypoint_names()
        if not names:
            self.get_logger().error('no waypoints (tag some with /add_waypoint first)')
            return 1
        self.get_logger().info(f'mission: {names}')

        stop_on_failure = self.get_parameter('stop_on_failure').value
        failed = []
        for name in names:
            if not self.go_to(name):
                failed.append(name)
                if stop_on_failure:
                    break
        if failed:
            self.get_logger().error(f'mission finished with failures: {failed}')
            return 1
        self.get_logger().info('mission complete')
        return 0

    def cancel_late_goals(self):
        """Cancel any timed-out goal request that Nav2 has since accepted.

        Polled rather than done-callback: rclpy never runs a future's done
        callback for these once spin_until_future_complete has timed out.
        """
        for request in [r for r in self.unanswered if r.done()]:
            self.unanswered.remove(request)
            handle = request.result()
            if handle is not None and handle.accepted:
                self.get_logger().warn('Nav2 accepted a timed-out goal late - cancelling it')
                self.wait(handle.cancel_goal_async(), 2.0)

    def cancel_current(self):
        if self.current_handle is not None:
            self.get_logger().warn('cancelling current Nav2 goal')
            self.wait(self.current_handle.cancel_goal_async(), 2.0)

    def cleanup(self):
        """Stop anything still driving before the node goes away."""
        self.cancel_current()
        # Give unanswered goal requests a last chance to arrive, so they can
        # still be cancelled.
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while rclpy.ok() and self.unanswered and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.cancel_late_goals()
        if self.unanswered:
            self.get_logger().error('Nav2 never answered a goal request - check RViz that the rover is not driving')


def main():
    # rclpy's own SIGINT handler shuts ROS down before `finally` runs, which
    # would make the cancel below fail. Handle the signals here instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):  # SIGINT may be ignored when started in the background
        signal.signal(sig, signal.default_int_handler)
    node = WaypointMission()
    rc = 1
    try:
        rc = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C (or any early exit) must stop the rover, not just this script.
        try:
            node.cleanup()
        except Exception as e:  # never let cleanup hide the real exit reason
            node.get_logger().error(f'cleanup failed: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
