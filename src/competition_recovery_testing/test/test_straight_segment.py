#!/usr/bin/env python3

import math
import unittest

from competition_recovery_testing.straight_segment import (
    LocalizationSample,
    RuntimeGateError,
    SegmentConfig,
    SegmentController,
    SegmentState,
    build_waypoints,
    validate_runtime_gate,
)


class StraightSegmentGeometryTest(unittest.TestCase):
    def test_waypoints_follow_frozen_heading_at_required_distances(self):
        waypoints = build_waypoints(1.0, -2.0, math.pi / 2.0)
        self.assertEqual(len(waypoints), 3)
        for waypoint, distance in zip(waypoints, (0.25, 0.50, 0.75)):
            self.assertAlmostEqual(waypoint.x, 1.0, places=12)
            self.assertAlmostEqual(waypoint.y, -2.0 + distance, places=12)
            self.assertAlmostEqual(waypoint.yaw, math.pi / 2.0, places=12)

    def test_controller_freezes_first_valid_pose(self):
        controller = SegmentController()
        first = sample(3.0, 4.0, 0.4, 10.0)
        transition = controller.observe(first, now=10.1, status_stamp=10.0)
        self.assertEqual(transition.state, SegmentState.ACTIVE)
        self.assertTrue(transition.publish_path)
        self.assertEqual(controller.origin, (3.0, 4.0, 0.4))

        controller.observe(sample(3.1, 4.0, 0.5, 10.2), now=10.2,
                           status_stamp=10.2)
        self.assertEqual(controller.origin, (3.0, 4.0, 0.4))


class StraightSegmentStateTransitionTest(unittest.TestCase):
    def test_distance_reached_requests_single_terminal_cancel(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 5.0), now=5.0,
                           status_stamp=5.0)
        transition = controller.observe(
            sample(0.55, 0.0, 0.0, 6.0), now=6.0, status_stamp=6.0)
        self.assertEqual(transition.state, SegmentState.SUCCEEDED)
        self.assertTrue(transition.cancel_path)
        self.assertFalse(
            controller.tick(now=6.1, status_stamp=6.1).cancel_path)

    def test_timeout_cancels(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 2.0), now=2.0,
                           status_stamp=2.0)
        transition = controller.tick(now=10.01, status_stamp=10.01)
        self.assertEqual(transition.state, SegmentState.TIMEOUT)
        self.assertTrue(transition.cancel_path)

    def test_yaw_deviation_cancels(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 1.0), now=1.0,
                           status_stamp=1.0)
        transition = controller.observe(
            sample(0.1, 0.0, 0.151, 1.1), now=1.1, status_stamp=1.1)
        self.assertEqual(transition.state, SegmentState.BLOCKED)
        self.assertIn("yaw deviation", transition.detail)
        self.assertTrue(transition.cancel_path)

    def test_cross_track_deviation_cancels(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 1.0), now=1.0,
                           status_stamp=1.0)
        transition = controller.observe(
            sample(0.1, 0.151, 0.0, 1.1), now=1.1, status_stamp=1.1)
        self.assertEqual(transition.state, SegmentState.BLOCKED)
        self.assertIn("cross-track", transition.detail)
        self.assertTrue(transition.cancel_path)

    def test_stale_localization_cancels(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 1.0), now=1.0,
                           status_stamp=1.0)
        transition = controller.tick(now=2.01, status_stamp=2.01)
        self.assertEqual(transition.state, SegmentState.TIMEOUT)
        self.assertIn("odometry stale", transition.detail)
        self.assertTrue(transition.cancel_path)

    def test_stale_localization_status_cancels(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 1.0), now=1.0,
                           status_stamp=1.0)
        transition = controller.tick(now=2.01, status_stamp=1.0)
        self.assertEqual(transition.state, SegmentState.TIMEOUT)
        self.assertIn("localization status is stale", transition.detail)
        self.assertTrue(transition.cancel_path)

    def test_invalid_status_cancels_active_segment(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 1.0), now=1.0,
                           status_stamp=1.0)
        transition = controller.tick(
            now=1.1, status_valid=False, status_stamp=1.1)
        self.assertEqual(transition.state, SegmentState.BLOCKED)
        self.assertTrue(transition.cancel_path)


class StraightSegmentInvalidInputTest(unittest.TestCase):
    def test_nonfinite_pose_is_rejected(self):
        controller = SegmentController()
        transition = controller.observe(
            sample(float("nan"), 0.0, 0.0, 1.0), now=1.0,
            status_stamp=1.0)
        self.assertEqual(transition.state, SegmentState.FAILED)
        self.assertFalse(transition.publish_path)

    def test_wrong_frames_are_rejected(self):
        controller = SegmentController()
        transition = controller.observe(
            sample(0.0, 0.0, 0.0, 1.0, frame="odom"), now=1.0,
            status_stamp=1.0)
        self.assertEqual(transition.state, SegmentState.FAILED)

    def test_future_stamp_is_rejected(self):
        controller = SegmentController()
        transition = controller.observe(
            sample(0.0, 0.0, 0.0, 2.0), now=1.0,
            status_stamp=1.0)
        self.assertEqual(transition.state, SegmentState.FAILED)

    def test_ros_time_rollback_cancels(self):
        controller = SegmentController()
        controller.observe(sample(0.0, 0.0, 0.0, 3.0), now=3.0,
                           status_stamp=3.0)
        transition = controller.tick(now=2.9, status_stamp=2.9)
        self.assertEqual(transition.state, SegmentState.TIMEOUT)
        self.assertTrue(transition.cancel_path)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            SegmentConfig(timeout=0.0)
        with self.assertRaises(ValueError):
            SegmentConfig(waypoint_distances=(0.25, 0.20, 0.75))


class StraightSegmentRuntimeGateTest(unittest.TestCase):
    def test_only_explicit_safe_combination_is_accepted(self):
        validate_runtime_gate(
            competition_mode=False,
            test_only_acknowledged=True,
            enable_ground_truth="false",
        )

    def test_default_or_ground_truth_enabled_runtime_is_rejected(self):
        cases = (
            (True, True, "false"),
            (False, False, "false"),
            (False, True, "true"),
            (False, True, None),
        )
        for competition_mode, acknowledged, enable_ground_truth in cases:
            with self.subTest(
                    competition_mode=competition_mode,
                    acknowledged=acknowledged,
                    enable_ground_truth=enable_ground_truth):
                with self.assertRaises(RuntimeGateError):
                    validate_runtime_gate(
                        competition_mode,
                        acknowledged,
                        enable_ground_truth,
                    )


def sample(x, y, yaw, stamp, frame="world", child="base"):
    return LocalizationSample(
        x=x,
        y=y,
        yaw=yaw,
        stamp=stamp,
        frame_id=frame,
        child_frame_id=child,
    )


if __name__ == "__main__":
    unittest.main()
