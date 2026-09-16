#!/usr/bin/env python3
"""The only node that talks to the rover's wheels: joystick OR Nav2, never both.

The drive team's ESP32 firmware (ARC_26 repo) listens on /rover (Int32MultiArray, 6 signed PWM
values, order [R, L, R, L, R, L], magnitude 0-255). ARC_26's drive.py turns
/joy into that message but knows nothing about Nav2's /cmd_vel. This node sits
in between:

  drive.py --(/rover_joy)--> rover_bridge --(/rover)--> ESP32
  Nav2     --(/cmd_vel)----> rover_bridge

  MANUAL (default): /rover_joy is passed through unchanged.
  AUTO:             /cmd_vel is converted to wheel PWM (open loop).
                    Touching either stick drops straight back to MANUAL.

Safety: the rover firmware keeps its last /rover command forever if messages
stop, so this node publishes at `rate` all the time. Stale input (drive.py
dead, Nav2 silent) becomes zeros, and zeros are sent on shutdown.

Run (drive.py must be remapped so it no longer writes /rover itself):
  ros2 run drive_controls drive.py --ros-args -r /rover:=/rover_joy
  ros2 run kratos_nav rover_bridge.py --ros-args -p track_width:=<m> -p max_wheel_speed:=<m/s>
Switch mode:
  ros2 service call /rover_bridge/set_auto std_srvs/srv/SetBool "{data: true}"
"""
import math

ZEROS = [0] * 6


def twist_to_pwm(v, w, track_width, max_wheel_speed, max_pwm=255, min_pwm=0):
    """Differential-drive (left, right) signed PWM for linear v [m/s], angular w [rad/s].

    If either wheel would exceed max_wheel_speed, both are scaled down by the
    same factor, so the rover keeps the curvature Nav2 asked for.
    min_pwm lifts small non-zero commands above the motors' stall threshold.
    """
    left = v - w * track_width / 2.0
    right = v + w * track_width / 2.0
    biggest = max(abs(left), abs(right))
    if biggest > max_wheel_speed:
        left *= max_wheel_speed / biggest
        right *= max_wheel_speed / biggest

    def pwm(speed):
        value = int(round(speed / max_wheel_speed * max_pwm))
        if value != 0 and abs(value) < min_pwm:
            value = int(math.copysign(min_pwm, value))
        return max(-max_pwm, min(max_pwm, value))

    return pwm(left), pwm(right)


def wheel_array(left, right):
    """/rover layout. Older firmware picks BOTH sides' directions only when both
    values are non-zero; if exactly one is 0 the moving side keeps a stale
    direction. PWM 1 is below any motor's stall point, so use it instead of 0.
    """
    if (left == 0) != (right == 0):
        left = left or 1
        right = right or 1
    return [right, left, right, left, right, left]


def sticks_touched(axes, stick_axes, deadzone):
    return any(i < len(axes) and abs(axes[i]) >= deadzone for i in stick_axes)


def main():
    import signal
    import time

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    from rclpy.duration import Duration
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Int32MultiArray
    from std_srvs.srv import SetBool

    class RoverBridge(Node):
        def __init__(self):
            super().__init__('rover_bridge')
            # PLACEHOLDERS: measure on the rover. track_width = distance between
            # left and right wheel centres. max_wheel_speed = ground speed at PWM 255.
            self.declare_parameter('track_width', 0.80)
            self.declare_parameter('max_wheel_speed', 1.0)
            self.declare_parameter('max_pwm', 255)
            self.declare_parameter('min_pwm', 0)
            self.declare_parameter('input_timeout', 0.5)
            self.declare_parameter('rate', 50.0)
            # Same axes/deadzone drive.py uses; any of them past the deadzone
            # while in AUTO hands control back to the driver.
            self.declare_parameter('stick_axes', [0, 1])
            self.declare_parameter('override_deadzone', 0.5)
            # Joystick button that toggles AUTO; -1 = service only.
            self.declare_parameter('auto_button', -1)

            self.auto = False
            self.joy_rover = None
            self.joy_rover_time = None
            self.cmd = None
            self.cmd_time = None
            self.last_buttons = []

            self.pub = self.create_publisher(Int32MultiArray, '/rover', 10)
            self.create_subscription(Int32MultiArray, '/rover_joy', self.on_joy_rover, 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
            self.create_subscription(Joy, '/joy', self.on_joy, 10)
            self.create_service(SetBool, '~/set_auto', self.on_set_auto)
            self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
            self.get_logger().info('rover_bridge ready in MANUAL (set_auto to hand over to Nav2)')

        def param(self, name):
            return self.get_parameter(name).value

        def set_mode(self, auto, why):
            if auto != self.auto:
                self.auto = auto
                self.cmd = None  # never act on a /cmd_vel from before the switch
                self.get_logger().warn(f"{'AUTO' if auto else 'MANUAL'} ({why})")

        def on_set_auto(self, request, response):
            self.set_mode(request.data, 'service')
            response.success = True
            response.message = 'AUTO' if self.auto else 'MANUAL'
            return response

        def on_joy_rover(self, msg):
            if len(msg.data) >= 6:
                self.joy_rover = list(msg.data[:6])
                self.joy_rover_time = self.get_clock().now()

        def on_cmd_vel(self, msg):
            self.cmd = (msg.linear.x, msg.angular.z)
            self.cmd_time = self.get_clock().now()

        def on_joy(self, msg):
            if self.auto and sticks_touched(msg.axes, self.param('stick_axes'),
                                            self.param('override_deadzone')):
                self.set_mode(False, 'stick override')
            button = self.param('auto_button')
            if 0 <= button < len(msg.buttons):
                was = self.last_buttons[button] if button < len(self.last_buttons) else 0
                if msg.buttons[button] and not was:
                    self.set_mode(not self.auto, f'button {button}')
            self.last_buttons = list(msg.buttons)

        def fresh(self, stamp):
            timeout = Duration(seconds=self.param('input_timeout'))
            return stamp is not None and self.get_clock().now() - stamp < timeout

        def command(self):
            if not self.auto:
                return self.joy_rover if self.fresh(self.joy_rover_time) else ZEROS
            if self.cmd is None or not self.fresh(self.cmd_time):
                return ZEROS
            left, right = twist_to_pwm(*self.cmd, self.param('track_width'),
                                       self.param('max_wheel_speed'),
                                       self.param('max_pwm'), self.param('min_pwm'))
            return wheel_array(left, right)

        def tick(self):
            self.publish(self.command())

        def publish(self, data):
            self.pub.publish(Int32MultiArray(data=[int(x) for x in data]))

    # rclpy's own SIGINT handler shuts ROS down before `finally` runs, so the
    # stop command below could never be published. Handle the signals here.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):  # SIGINT may be ignored when started in the background
        signal.signal(sig, signal.default_int_handler)
    node = RoverBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Old firmware holds the last command: leave it at zero. Spread the sends
        # out; destroying the node right after publishing can drop them.
        try:
            for _ in range(5):
                node.publish(ZEROS)
                time.sleep(0.05)
            node.get_logger().warn('stopped: wheels set to zero')
        except Exception as e:
            node.get_logger().error(f'could not send the stop command: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
