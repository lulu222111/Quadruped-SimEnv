#!/usr/bin/env python3
"""Unit tests for interface_validator.py — pure Python with mocked rospy.

Covers:
  - Initial state != ready
  - Scene file missing / corrupted / wrong root type
  - Competition mode: topic check not yet executed → not ready
  - Competition mode: forbidden topic check passed → ready
  - Detection of /Odometry_gazebo and /ground_truth/* topics
  - ROS Master query exception → transient (not permanent)
  - Master retry: first fails, second succeeds → ready
  - Test mode: explicitly skips forbidden topic check
  - get_published_topics() with correct API (no-arg call, data from list-of-lists)
  - Duplicate error suppression
  - Warnings vs errors separation
  - Forbidden file presence = warning, not error
  - report() includes warnings section
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
    def logwarn_throttle(cls, *args, **kw):
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


class _MockTime:
    """Simple mock for rospy.Time."""
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    @staticmethod
    def now():
        return _MockTime(100.0)

    @classmethod
    def from_sec(cls, secs):
        return cls(secs)

    def to_sec(self):
        return self._secs

    def __add__(self, other):
        if isinstance(other, _MockDuration):
            return _MockTime(self._secs + other.to_sec())
        return NotImplemented

    def __lt__(self, other):
        if isinstance(other, _MockTime):
            return self._secs < other._secs
        return NotImplemented

    def is_zero(self):
        return self._secs == 0.0


class _MockDuration:
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    @classmethod
    def from_sec(cls, secs):
        return cls(secs)

    def to_sec(self):
        return self._secs

    def __add__(self, other):
        if isinstance(other, _MockDuration):
            return _MockDuration(self._secs + other._secs)
        return NotImplemented


class _MockRospyModule:
    """Module-level mock for 'import rospy'."""
    ROSException = type("ROSException", (Exception,), {})
    Time = _MockTime
    Duration = _MockDuration

    def __getattr__(self, name):
        if name in ("ROSException", "Time", "Duration"):
            return getattr(self, name)
        return getattr(_MockLog, name, _MockLog.info)


if "rospy" not in sys.modules:
    sys.modules["rospy"] = _MockRospyModule()

import rospy as mock_rospy

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

    def _make_scene_file(self, content) -> Path:
        """Create generated_building/team_scene_info.json under tmpdir."""
        p = Path(self.tmpdir) / "generated_building"
        p.mkdir(parents=True, exist_ok=True)
        scene_path = p / "team_scene_info.json"
        scene_path.write_text(json.dumps(content), encoding="utf-8")
        return scene_path

    def _make_scene_raw(self, raw: str) -> Path:
        p = Path(self.tmpdir) / "generated_building"
        p.mkdir(parents=True, exist_ok=True)
        scene_path = p / "team_scene_info.json"
        scene_path.write_text(raw, encoding="utf-8")
        return scene_path

    def test_load_scene_success(self):
        self._make_scene_file({"schema": "v1", "referee_only": {}})
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNotNone(data)
        self.assertTrue(v.scene_info_loaded)
        self.assertFalse(v.is_ready())

    def test_scene_file_missing(self):
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)
        self.assertFalse(v.is_ready())

    def test_scene_json_corrupted(self):
        self._make_scene_raw("not valid json {{{")
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

    # --- JSON root type checks ---

    def test_scene_root_is_list_fails(self):
        self._make_scene_file([1, 2, 3])
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)
        self.assertIn("not a JSON object", str(v.errors))

    def test_scene_root_is_string_fails(self):
        self._make_scene_file("just a string")
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)

    def test_scene_root_is_number_fails(self):
        self._make_scene_file(42)
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)

    def test_scene_root_is_null_fails(self):
        self._make_scene_file(None)
        v = InterfaceValidator(competition_mode=True)
        data = v.load_scene_info(self.tmpdir)
        self.assertIsNone(data)
        self.assertFalse(v.scene_info_loaded)


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
    # get_published_topics() correct API tests
    # ------------------------------------------------------------------

    def test_get_published_topics_called_with_default_namespace(self):
        """Verify get_published_topics() is called with no args (default namespace='/')."""
        call_args = []

        def fake_get_published_topics(*args, **kwargs):
            call_args.append((args, kwargs))
            return [("/rosout", "rosgraph_msgs/Log")]

        mock_rospy.get_published_topics = fake_get_published_topics
        try:
            v = self._make_scene(["/Odometry_gazebo"])
            v.check_forbidden_topics()
            # Must be called with no args (default namespace='/')
            self.assertEqual(len(call_args), 1)
            self.assertEqual(call_args[0], ((), {}),
                             "get_published_topics must be called with no args, not with timeout")
        finally:
            del mock_rospy.get_published_topics

    def test_get_published_topics_list_parsing(self):
        """get_published_topics returns [[name, type], ...] — extract names correctly."""
        mock_rospy.get_published_topics = lambda: [
            ["/rosout", "rosgraph_msgs/Log"],
            ["/clock", "rosgraph_msgs/Clock"],
            ["/Odometry_gazebo", "nav_msgs/Odometry"],
            ["/ground_truth/pose", "geometry_msgs/Pose"],
            ["/competition/state", "competition_interfaces/CompetitionState"],
        ]

        v = self._make_scene(["/Odometry_gazebo", "/ground_truth/pose"])
        result = v.check_forbidden_topics()
        self.assertFalse(result, "Should FAIL when forbidden topics are published.")
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertFalse(v.forbidden_topic_check_passed)
        self.assertFalse(v.is_ready())

        del mock_rospy.get_published_topics

    def test_get_published_topics_clean(self):
        """No forbidden topics published → pass."""
        mock_rospy.get_published_topics = lambda: [
            ["/rosout", "rosgraph_msgs/Log"],
            ["/clock", "rosgraph_msgs/Clock"],
            ["/competition/state", "competition_interfaces/CompetitionState"],
        ]

        v = self._make_scene(["/Odometry_gazebo", "/ground_truth/pose"])
        result = v.check_forbidden_topics()
        self.assertTrue(result, "Should PASS when no forbidden topics are published.")
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertTrue(v.forbidden_topic_check_passed)

        del mock_rospy.get_published_topics

    def test_detect_odometry_gazebo(self):
        mock_rospy.get_published_topics = lambda: [
            ["/Odometry_gazebo", "nav_msgs/Odometry"],
        ]
        v = self._make_scene(["/Odometry_gazebo"])
        result = v.check_forbidden_topics()
        self.assertFalse(result)
        # Check errors list rather than shared mock log (which may differ
        # when multiple test files share sys.modules["rospy"])
        self.assertTrue(any("FORBIDDEN TOPIC DETECTED" in e for e in v.errors),
                        f"Expected FORBIDDEN TOPIC DETECTED in errors: {v.errors}")
        del mock_rospy.get_published_topics

    def test_detect_ground_truth(self):
        mock_rospy.get_published_topics = lambda: [
            ["/ground_truth/odom", "nav_msgs/Odometry"],
            ["/ground_truth/pose", "geometry_msgs/Pose"],
        ]
        v = self._make_scene(["/ground_truth/odom", "/ground_truth/pose"])
        result = v.check_forbidden_topics()
        self.assertFalse(result)
        del mock_rospy.get_published_topics

    def test_no_forbidden_topics_in_scene(self):
        """Empty forbidden list → automatic pass (no Master query needed)."""
        v = self._make_scene([])
        result = v.check_forbidden_topics()
        self.assertTrue(result)
        self.assertTrue(v.forbidden_topic_check_passed)
        self.assertTrue(v.forbidden_topic_check_completed)

    # ------------------------------------------------------------------
    # ROS Master transient error → retry (NOT permanent block)
    # ------------------------------------------------------------------

    def test_master_exception_keeps_check_incomplete(self):
        """Master exception → check stays incomplete (retry, not permanent fail)."""
        mock_rospy.get_published_topics = lambda: (_ for _ in ()).throw(
            mock_rospy.ROSException("master unreachable")
        )

        v = self._make_scene(["/Odometry_gazebo"])
        result = v.check_forbidden_topics()
        self.assertFalse(result)
        # Check is NOT completed — transient error, will retry
        self.assertFalse(v.forbidden_topic_check_completed,
                         "Master exception must NOT mark check as completed.")
        self.assertFalse(v.forbidden_topic_check_passed)

        del mock_rospy.get_published_topics

    def test_master_retry_first_fails_then_succeeds(self):
        """First call: Master exception → incomplete. Second call: clean → ready."""
        call_count = [0]

        def flaky_master():
            call_count[0] += 1
            if call_count[0] == 1:
                raise mock_rospy.ROSException("master unreachable")
            return [["/rosout", "rosgraph_msgs/Log"]]

        mock_rospy.get_published_topics = flaky_master

        v = self._make_scene(["/Odometry_gazebo"])

        # First call: Master exception — check stays incomplete
        result1 = v.check_forbidden_topics()
        self.assertFalse(result1)
        self.assertFalse(v.forbidden_topic_check_completed)

        # Force retry by clearing the retry timer
        v._next_topic_retry_time = None

        # Second call: succeeds
        result2 = v.check_forbidden_topics()
        self.assertTrue(result2)
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertTrue(v.forbidden_topic_check_passed)
        self.assertTrue(v.is_ready())

        self.assertEqual(call_count[0], 2)
        del mock_rospy.get_published_topics

    def test_master_exception_respects_retry_interval(self):
        """After exception, early retry is skipped (respects retry interval)."""
        call_count = [0]

        def flaky_master():
            call_count[0] += 1
            raise mock_rospy.ROSException("master unreachable")

        mock_rospy.get_published_topics = flaky_master

        v = self._make_scene(["/Odometry_gazebo"])

        # First call: exception, sets next_retry_time in the future
        v.check_forbidden_topics()
        self.assertEqual(call_count[0], 1)

        # Second call immediately: should NOT call Master again
        # (next_retry_time is in the future)
        v.check_forbidden_topics()
        self.assertEqual(call_count[0], 1, "Retry interval must prevent immediate re-query")

        del mock_rospy.get_published_topics


class TestInterfaceValidatorTestMode(unittest.TestCase):
    """Test mode behaviour."""

    def setUp(self):
        _MockLog.reset()

    def test_test_mode_skips_topic_check(self):
        v = InterfaceValidator(competition_mode=False)
        result = v.check_forbidden_topics()
        self.assertTrue(result)
        self.assertTrue(v.forbidden_topic_check_completed)
        self.assertTrue(v.forbidden_topic_check_passed)

    def test_test_mode_not_ready_without_scene(self):
        v = InterfaceValidator(competition_mode=False)
        v.check_forbidden_topics()
        self.assertFalse(v.is_ready(),
                         "Should not be ready without scene info, even in test mode.")


class TestInterfaceValidatorDuplicateErrors(unittest.TestCase):
    """Duplicate error suppression."""

    def setUp(self):
        _MockLog.reset()

    def test_no_duplicate_identical_errors(self):
        mock_rospy.get_published_topics = lambda: [
            ["/Odometry_gazebo", "nav_msgs/Odometry"],
        ]

        v = InterfaceValidator(competition_mode=True)
        import tempfile
        tmpdir = tempfile.mkdtemp()
        try:
            p = Path(tmpdir) / "generated_building"
            p.mkdir(parents=True, exist_ok=True)
            (p / "team_scene_info.json").write_text(
                json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": ["/Odometry_gazebo"]}}),
                encoding="utf-8",
            )
            v.load_scene_info(tmpdir)

            v.check_forbidden_topics()
            error_count_1 = len(v.errors)
            v.check_forbidden_topics()
            error_count_2 = len(v.errors)

            self.assertEqual(error_count_1, error_count_2,
                             "Running check_forbidden_topics twice should not add duplicate errors.")
        finally:
            del mock_rospy.get_published_topics
            import shutil
            shutil.rmtree(tmpdir, ignore_errors=True)


class TestInterfaceValidatorIsReady(unittest.TestCase):
    """Comprehensive is_ready() behaviour."""

    def setUp(self):
        _MockLog.reset()

    def test_empty_errors_does_not_mean_ready(self):
        v = InterfaceValidator(competition_mode=True)
        self.assertEqual(len(v.errors), 0)
        self.assertFalse(v.is_ready(),
                         "is_ready() requires explicit check completion, not just empty errors.")

    def test_competition_mode_full_ready_flow(self):
        import tempfile
        tmpdir = tempfile.mkdtemp()
        try:
            p = Path(tmpdir) / "generated_building"
            p.mkdir(parents=True, exist_ok=True)
            (p / "team_scene_info.json").write_text(
                json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": ["/Odometry_gazebo"]}}),
                encoding="utf-8",
            )
            mock_rospy.get_published_topics = lambda: [["/rosout", "rosgraph_msgs/Log"]]

            v = InterfaceValidator(competition_mode=True)
            self.assertFalse(v.is_ready())
            v.load_scene_info(tmpdir)
            self.assertTrue(v.scene_info_loaded)
            self.assertFalse(v.is_ready())
            v.check_forbidden_topics()
            self.assertTrue(v.forbidden_topic_check_completed)
            self.assertTrue(v.forbidden_topic_check_passed)
            self.assertTrue(v.is_ready())
        finally:
            del mock_rospy.get_published_topics
            import shutil
            shutil.rmtree(tmpdir, ignore_errors=True)

    def test_test_mode_full_ready_flow(self):
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
            self.assertFalse(v.is_ready())
            v.check_forbidden_topics()
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

    def test_report_includes_warnings_section(self):
        v = InterfaceValidator(competition_mode=True)
        r = v.report()
        # Should mention warnings (even if empty; section for them exists)
        # The report format includes "Warnings:" only when there are warnings
        # So we just verify the report method doesn't crash
        self.assertIsInstance(r, str)


class TestInterfaceValidatorWarnings(unittest.TestCase):
    """Warnings are separate from errors and do not affect is_ready()."""

    def setUp(self):
        _MockLog.reset()
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_warnings_property_is_separate(self):
        v = InterfaceValidator(competition_mode=True)
        self.assertEqual(v.warnings, [])
        self.assertEqual(v.errors, [])

    def test_forbidden_file_adds_warning_not_error(self):
        fp = Path(self.tmpdir) / InterfaceValidator.FORBIDDEN_FILES[0]
        fp.parent.mkdir(parents=True, exist_ok=True)
        fp.write_text("{}", encoding="utf-8")

        v = InterfaceValidator(competition_mode=True)
        v.check_forbidden_files(self.tmpdir)
        self.assertTrue(len(v.warnings) > 0, "Forbidden file should add a warning")
        self.assertEqual(len(v.errors), 0, "Forbidden file should NOT add an error")

    def test_forbidden_file_does_not_block_is_ready(self):
        fp = Path(self.tmpdir) / InterfaceValidator.FORBIDDEN_FILES[0]
        fp.parent.mkdir(parents=True, exist_ok=True)
        fp.write_text("{}", encoding="utf-8")

        mock_rospy.get_published_topics = lambda: [["/rosout", "rosgraph_msgs/Log"]]

        try:
            p = Path(self.tmpdir) / "generated_building"
            p.mkdir(parents=True, exist_ok=True)
            (p / "team_scene_info.json").write_text(
                json.dumps({"schema": "v1", "referee_only": {"forbidden_topics": []}}),
                encoding="utf-8",
            )

            v = InterfaceValidator(competition_mode=True)
            v.load_scene_info(self.tmpdir)
            v.check_forbidden_topics()
            v.check_forbidden_files(self.tmpdir)

            # Has warnings but still ready
            self.assertTrue(len(v.warnings) > 0)
            self.assertTrue(v.is_ready(),
                            "Forbidden file presence must NOT block is_ready()")
        finally:
            del mock_rospy.get_published_topics

    def test_no_forbidden_files_present(self):
        v = InterfaceValidator(competition_mode=True)
        result = v.check_forbidden_files(self.tmpdir)
        self.assertTrue(result)
        self.assertTrue(v.forbidden_file_check_completed)
        self.assertEqual(v.warnings, [])

    def test_report_shows_warnings(self):
        fp = Path(self.tmpdir) / InterfaceValidator.FORBIDDEN_FILES[0]
        fp.parent.mkdir(parents=True, exist_ok=True)
        fp.write_text("{}", encoding="utf-8")

        v = InterfaceValidator(competition_mode=True)
        v.check_forbidden_files(self.tmpdir)
        r = v.report()
        self.assertIn("Warnings:", r)
        self.assertIn("MUST NOT read", r)


if __name__ == "__main__":
    unittest.main()
