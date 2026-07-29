#!/usr/bin/env python3
"""Unit tests for interface_validator.py — pure Python with mocked rospy.

Covers:
  - Initial state != ready
  - Scene file missing / corrupted
  - Competition mode: topic check not yet executed → not ready
  - Competition mode: forbidden topic check passed → ready
  - Detection of /Odometry_gazebo and /ground_truth/* topics
  - ROS Master query exception
  - Test mode: explicitly skips forbidden topic check (marked completed + passed)
  - get_published_topics() tuple parsing (topic name extraction)
  - Duplicate error suppression
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

# Add src/ to sys.path
_SRC = str(Path(__file__).resolve().parents[1] / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


# ---------------------------------------------------------------------------
# Mock rospy
# ---------------------------------------------------------------------------

class _MockLog:
    """Collect log calls for inspection in tests."""
    calls: list = []

    @classmethod
    def reset(cls):
        cls.calls.clear()

    @classmethod
    def info(cls, *args, **kw):
        cls.calls.append(("info", args))

    @classmethod
    def warn(cls, *args, **kw):
        cls.calls.append(("warn", args))

    @classmethod
    def logwarn(cls, *args, **kw):
        cls.calls.append(("warn", args))

    @classmethod
    def error(cls, *args, **kw):
        cls.calls.append(("error", args))

    @classmethod
    def logerr(cls, *args, **kw):
        cls.calls.append(("error", args))

    @classmethod
    def loginfo(cls, *args, **kw):
        cls.calls.append(("info", args))


class _MockRospyModule:
    """Module-level mock for 'import rospy'."""
    ROSException = type("ROSException", (Exception,), {})

    def __getattr__(self, name):
        if name in ("ROSException",):
            return getattr(self, name)
        return getattr(_MockLog, name, _MockLog.info)


if "rospy" not in sys.modules:
    sys.modules["rospy"] = _MockRospyModule()


# ---------------------------------------------------------------------------
# Actual test module import
# ---------------------------------------------------------------------------

from interface_validator import InterfaceValidator


class TestInterfaceValidatorInitialState(unittest.TestCase):
    """Verify that a freshly-constructed validator is NOT ready."""

    def setUp(self):
        _MockLog.reset()

    def test_initial_state_not_ready(self):
        v = InterfaceValidator(competition_mode=True)
        self.assertFalse(v.is_ready(),
                         "Fresh validator must not be ready (no checks executed).")
        self.assertFalse(v.scene_info_loaded)
        self.assertFalse(v.forbidden_topic_check_completed)
        self.assertFalse(v.forbidden_topic_check_passed)
        self.assertFalse(v.forbidden_file_check_completed)

    def test_initial_state_not_ready_test_mode(self):
        v = InterfaceValidator(competition_mode=False)
        self.assertFalse(v.is_ready(),
                         "Fresh test-mode validator must not be ready (no scene info loaded).")
        self.assertFalse(v.scene_info_loaded)


class TestInterfaceValidatorSceneInfo(unittest.TestCase):
    """Scene file loading."""

    def setUp(self):
        _MockLog.reset()
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _make_scene_file(self, content: dict) -> Path:
        """Create generated_building/team_scene_info.json under tmpdir."""
        p = Path(self.tmpdir) / "generated_building"
        p.mkdir(parents=True, exist_ok=True)
        scene_path = p / "team_scene_info.json"
        scene_path.write_text(json.dumps(content), encoding="utf-8")
        return scene_path

    def test_load_scene_success(self):
        self._make_scene_file({"schema": "v1", "referee_only": {}})
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNotNone(data)
        self.assertTrue(v.scene_info_loaded)
        # Still not ready — forbidden topic check not run yet
        self.assertFalse(v.is_ready())

    def test_scene_file_missing(self):
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)  # no file created
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)
        self.assertFalse(v.is_ready())

    def test_scene_json_corrupted(self):
        p = Path(self.tmpdir) / "generated_building"
        p.mkdir(parents=True, exist_ok=True)
        (p / "team_scene_info.json").write_text("not valid json {{{", encoding="utf-8")
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)

    def test_scene_loaded_then_scene_info_loaded_true(self):
        self._make_scene_file({"schema": "v1"})
        v = InterfaceValidator(competition_mode=True)
        self.assertFalse(v.scene_info_loaded)
        v.load_scene_info(self.tmpdir)
        self.assertTrue(v.scene_info_loaded)


class TestInterfaceValidatorForbiddenTopics(unittest.TestCase):
    """Forbidden topic check logic — uses mocked rospy.get_published_topics."""

    def setUp(self):
        _MockLog.reset()
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _make_scene(self, forbidden_topics: list) -> InterfaceValidator:
        p = Path(self.tmpdir) / "generated_building"
        p.mkdir(parents=True, exist_ok=True)
        (p / "team_scene_info.json").write_text(
            json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": forbidden_topics}}),
            encoding="utf-8",
        )
        v = InterfaceValidator(competition_mode=True)
        v.load_scene_info(self.tmpdir)
        return v

    # ------------------------------------------------------------------
    # Tuple parsing
    # ------------------------------------------------------------------

    def test_get_published_topics_tuple_parsing(self):
        """Verify that (topic, type) tuples are correctly parsed to topic names."""
        # This test simulates what happens when rospy returns tuples.
        # We patch rospy.get_published_topics to return realistic data.
        import rospy as mock_rospy

        mock_rospy.get_published_topics = lambda timeout: [
            ("/rosout", "rosgraph_msgs/Log"),
            ("/clock", "rosgraph_msgs/Clock"),
            ("/Odometry_gazebo", "nav_msgs/Odometry"),
            ("/ground_truth/pose", "geometry_msgs/Pose"),
            ("/competition/state", "competition_interfaces/CompetitionState"),
        ]

        v = self._make_scene(["/Odometry_gazebo", "/ground_truth/pose"])
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertFalse(result, "Should FAIL when forbidden topics are published.")
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertFalse(v.forbidden_topic_check_passed)
        self.assertFalse(v.is_ready())

        # Clean up
        del mock_rospy.get_published_topics

    def test_get_published_topics_clean(self):
        """No forbidden topics published → pass."""
        import rospy as mock_rospy
        mock_rospy.get_published_topics = lambda timeout: [
            ("/rosout", "rosgraph_msgs/Log"),
            ("/clock", "rosgraph_msgs/Clock"),
            ("/competition/state", "competition_interfaces/CompetitionState"),
        ]

        v = self._make_scene(["/Odometry_gazebo", "/ground_truth/pose"])
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertTrue(result, "Should PASS when no forbidden topics are published.")
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertTrue(v.forbidden_topic_check_passed)

        del mock_rospy.get_published_topics

    def test_detect_odometry_gazebo(self):
        """Specifically detect /Odometry_gazebo."""
        import rospy as mock_rospy
        mock_rospy.get_published_topics = lambda timeout: [
            ("/Odometry_gazebo", "nav_msgs/Odometry"),
        ]

        v = self._make_scene(["/Odometry_gazebo"])
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertFalse(result)
        self.assertIn("FORBIDDEN TOPIC DETECTED", str(_MockLog.calls))

        del mock_rospy.get_published_topics

    def test_detect_ground_truth(self):
        """Specifically detect /ground_truth/* topics."""
        import rospy as mock_rospy
        mock_rospy.get_published_topics = lambda timeout: [
            ("/ground_truth/odom", "nav_msgs/Odometry"),
            ("/ground_truth/pose", "geometry_msgs/Pose"),
        ]

        v = self._make_scene(["/ground_truth/odom", "/ground_truth/pose"])
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertFalse(result)

        del mock_rospy.get_published_topics

    def test_ros_master_query_exception(self):
        """ROS Master query failure → check fails."""
        import rospy as mock_rospy
        mock_rospy.get_published_topics = lambda timeout: (_ for _ in ()).throw(
            mock_rospy.ROSException("master unreachable")
        )

        v = self._make_scene(["/Odometry_gazebo"])
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertFalse(result)
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertFalse(v.forbidden_topic_check_passed)

        del mock_rospy.get_published_topics

    def test_no_forbidden_topics_in_scene(self):
        """Empty forbidden list → automatic pass."""
        import rospy as mock_rospy
        mock_rospy.get_published_topics = lambda timeout: [
            ("/Odometry_gazebo", "nav_msgs/Odometry"),
        ]

        v = self._make_scene([])  # empty forbidden list
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertTrue(result)
        self.assertTrue(v.forbidden_topic_check_passed)

        del mock_rospy.get_published_topics


class TestInterfaceValidatorTestMode(unittest.TestCase):
    """Test mode behaviour — forbidden topic check explicitly skipped."""

    def setUp(self):
        _MockLog.reset()

    def test_test_mode_skips_topic_check(self):
        v = InterfaceValidator(competition_mode=False)
        # No scene loaded, but test mode should still mark check as completed+passed
        result = v.check_forbidden_topics(timeout=1.0)
        self.assertTrue(result)
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertTrue(v.forbidden_topic_check_passed)

    def test_test_mode_not_ready_without_scene(self):
        v = InterfaceValidator(competition_mode=False)
        v.check_forbidden_topics(timeout=1.0)
        # Forbidden topic check passed, but scene not loaded → NOT ready
        self.assertFalse(v.is_ready(),
                         "Should not be ready without scene info, even in test mode.")


class TestInterfaceValidatorDuplicateErrors(unittest.TestCase):
    """Duplicate error suppression."""

    def setUp(self):
        _MockLog.reset()

    def test_no_duplicate_identical_errors(self):
        import rospy as mock_rospy

        mock_rospy.get_published_topics = lambda timeout: [
            ("/Odometry_gazebo", "nav_msgs/Odometry"),
        ]

        v = InterfaceValidator(competition_mode=True)
        # Simulate scene load with the forbidden topic
        import tempfile
        tmpdir = tempfile.mkdtemp()
        p = Path(tmpdir) / "generated_building"
        p.mkdir(parents=True, exist_ok=True)
        (p / "team_scene_info.json").write_text(
            json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": ["/Odometry_gazebo"]}}),
            encoding="utf-8",
        )
        v.load_scene_info(tmpdir)

        # Run check twice
        v.check_forbidden_topics(timeout=1.0)
        error_count_1 = len(v.errors)
        v.check_forbidden_topics(timeout=1.0)
        error_count_2 = len(v.errors)

        self.assertEqual(error_count_1, error_count_2,
                         "Running check_forbidden_topics twice should not add duplicate errors.")

        del mock_rospy.get_published_topics
        import shutil
        shutil.rmtree(tmpdir, ignore_errors=True)


class TestInterfaceValidatorIsReady(unittest.TestCase):
    """Comprehensive is_ready() behaviour."""

    def setUp(self):
        _MockLog.reset()

    def test_empty_errors_does_not_mean_ready(self):
        """An empty error list does NOT imply readiness — checks must be explicitly completed."""
        v = InterfaceValidator(competition_mode=True)
        # No checks run, no errors — but we're NOT ready
        self.assertEqual(len(v.errors), 0)
        self.assertFalse(v.is_ready(),
                         "is_ready() requires explicit check completion, not just empty errors.")

    def test_competition_mode_full_ready_flow(self):
        """Full ready flow in competition mode."""
        import rospy as mock_rospy
        import tempfile
        tmpdir = tempfile.mkdtemp()

        try:
            # Create scene file
            p = Path(tmpdir) / "generated_building"
            p.mkdir(parents=True, exist_ok=True)
            (p / "team_scene_info.json").write_text(
                json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": ["/Odometry_gazebo"]}}),
                encoding="utf-8",
            )

            mock_rospy.get_published_topics = lambda timeout: [
                ("/rosout", "rosgraph_msgs/Log"),
            ]

            v = InterfaceValidator(competition_mode=True)
            self.assertFalse(v.is_ready())

            v.load_scene_info(tmpdir)
            self.assertTrue(v.scene_info_loaded)
            self.assertFalse(v.is_ready())  # topic check not done

            v.check_forbidden_topics(timeout=1.0)
            self.assertTrue(v.forbidden_topic_check_completed)
            self.assertTrue(v.forbidden_topic_check_passed)
            self.assertTrue(v.is_ready())

        finally:
            del mock_rospy.get_published_topics
            import shutil
            shutil.rmtree(tmpdir, ignore_errors=True)

    def test_test_mode_full_ready_flow(self):
        """Full ready flow in test mode."""
        import tempfile
        tmpdir = tempfile.mkdtemp()

        try:
            p = Path(tmpdir) / "generated_building"
            p.mkdir(parents=True, exist_ok=True)
            (p / "team_scene_info.json").write_text(json.dumps({"schema": "v1"}), encoding="utf-8")

            v = InterfaceValidator(competition_mode=False)
            self.assertFalse(v.is_ready())

            v.load_scene_info(tmpdir)
            self.assertTrue(v.scene_info_loaded)
            self.assertFalse(v.is_ready())  # topic check not done yet (even for test mode)

            v.check_forbidden_topics(timeout=1.0)
            self.assertTrue(v.forbidden_topic_check_completed)
            self.assertTrue(v.forbidden_topic_check_passed)
            self.assertTrue(v.is_ready())

        finally:
            import shutil
            shutil.rmtree(tmpdir, ignore_errors=True)


class TestInterfaceValidatorReport(unittest.TestCase):
    """Report format."""

    def setUp(self):
        _MockLog.reset()

    def test_report_includes_state_flags(self):
        v = InterfaceValidator(competition_mode=True)
        r = v.report()
        self.assertIn("Scene info:", r)
        self.assertIn("Forbidden topic check:", r)
        self.assertIn("NOT LOADED", r)
        self.assertIn("NOT EXECUTED", r)


class TestInterfaceValidatorForbiddenFiles(unittest.TestCase):
    """Forbidden file check — warns but does not block."""

    def setUp(self):
        _MockLog.reset()
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_no_forbidden_files_present(self):
        v = InterfaceValidator(competition_mode=True)
        result = v.check_forbidden_files(self.tmpdir)
        self.assertTrue(result)
        self.assertTrue(v.forbidden_file_check_completed)

    def test_forbidden_file_warns_in_competition_mode(self):
        # Create a forbidden file
        fp = Path(self.tmpdir) / InterfaceValidator.FORBIDDEN_FILES[0]
        fp.parent.mkdir(parents=True, exist_ok=True)
        fp.write_text("{}", encoding="utf-8")

        v = InterfaceValidator(competition_mode=True)
        v.check_forbidden_files(self.tmpdir)
        self.assertTrue(v.forbidden_file_check_completed)
        # Should have a warning logged
        warn_msgs = [a for level, a in _MockLog.calls if level == "warn"]
        self.assertTrue(any("MUST NOT read" in str(m) for m in warn_msgs))

    def test_forbidden_file_does_not_block_startup(self):
        """Forbidden file presence alone does NOT prevent is_ready() (warn only)."""
        fp = Path(self.tmpdir) / InterfaceValidator.FORBIDDEN_FILES[0]
        fp.parent.mkdir(parents=True, exist_ok=True)
        fp.write_text("{}", encoding="utf-8")

        import rospy as mock_rospy
        mock_rospy.get_published_topics = lambda timeout: [("/rosout", "rosgraph_msgs/Log")]

        try:
            # Create scene with no forbidden topics
            p = Path(self.tmpdir) / "generated_building"
            p.mkdir(parents=True, exist_ok=True)
            (p / "team_scene_info.json").write_text(
                json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": []}}),
                encoding="utf-8",
            )

            v = InterfaceValidator(competition_mode=True)
            v.load_scene_info(self.tmpdir)
            v.check_forbidden_topics(timeout=1.0)
            v.check_forbidden_files(self.tmpdir)

            # Should still be ready — forbidden files warn, don't block
            self.assertTrue(v.is_ready())
        finally:
            del mock_rospy.get_published_topics


if __name__ == "__main__":
    unittest.main()
