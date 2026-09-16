#!/usr/bin/env python3
"""Pure-math tests for rover_bridge.py (no ROS needed): python3 test/test_rover_bridge.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from rover_bridge import sticks_touched, twist_to_pwm, wheel_array  # noqa: E402


class TwistToPwm(unittest.TestCase):
    def test_stop(self):
        self.assertEqual(twist_to_pwm(0.0, 0.0, 0.8, 1.0), (0, 0))

    def test_straight(self):
        self.assertEqual(twist_to_pwm(0.5, 0.0, 0.8, 1.0), (128, 128))
        self.assertEqual(twist_to_pwm(-0.3, 0.0, 0.8, 1.0), (-76, -76))

    def test_left_turn_speeds_up_right_wheel(self):
        left, right = twist_to_pwm(0.4, 0.5, 0.8, 1.0)  # wheels 0.2 / 0.6 m/s
        self.assertEqual((left, right), (51, 153))

    def test_spin_in_place(self):
        left, right = twist_to_pwm(0.0, 1.0, 0.8, 1.0)
        self.assertEqual((left, right), (-102, 102))

    def test_saturation_keeps_curvature(self):
        left, right = twist_to_pwm(1.5, 1.0, 0.8, 1.0)  # wheels 1.1 / 1.9
        self.assertEqual(right, 255)
        self.assertAlmostEqual(left / right, 1.1 / 1.9, delta=0.01)

    def test_clamped_to_max_pwm(self):
        for v, w in [(9, 0), (-9, 0), (0, 9), (3, -7)]:
            left, right = twist_to_pwm(v, w, 0.8, 1.0)
            self.assertLessEqual(max(abs(left), abs(right)), 255)

    def test_min_pwm_lifts_small_commands_only(self):
        self.assertEqual(twist_to_pwm(0.02, 0.0, 0.8, 1.0, min_pwm=60), (60, 60))
        self.assertEqual(twist_to_pwm(-0.02, 0.0, 0.8, 1.0, min_pwm=60), (-60, -60))
        self.assertEqual(twist_to_pwm(0.0, 0.0, 0.8, 1.0, min_pwm=60), (0, 0))
        self.assertEqual(twist_to_pwm(0.5, 0.0, 0.8, 1.0, min_pwm=60), (128, 128))


class WheelArray(unittest.TestCase):
    def test_layout_is_right_left(self):
        self.assertEqual(wheel_array(-10, 20), [20, -10, 20, -10, 20, -10])

    def test_all_zero_stays_zero(self):
        self.assertEqual(wheel_array(0, 0), [0] * 6)

    def test_one_zero_side_never_sent(self):
        # firmware only updates directions when both sides are non-zero
        for left, right in [(0, 100), (0, -100), (100, 0), (-100, 0)]:
            data = wheel_array(left, right)
            self.assertNotIn(0, data)
            self.assertEqual(max(abs(data[0]), abs(data[1])), 100)


class Sticks(unittest.TestCase):
    def test_override(self):
        self.assertFalse(sticks_touched([0.1, -0.2, 1.0], [0, 1], 0.5))
        self.assertTrue(sticks_touched([0.0, -0.7, 1.0], [0, 1], 0.5))
        self.assertFalse(sticks_touched([], [0, 1], 0.5))  # short axes array


if __name__ == '__main__':
    unittest.main()
