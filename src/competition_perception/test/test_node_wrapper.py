#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS-wrapper-level tests for perception_node, without Gazebo or a ROS
master.

A small fake ROS layer (rospy / message_filters / cv_bridge / message
classes) is installed into sys.modules and the real perception_node
script is loaded through importlib.  The real perception_core and real
OpenCV/numpy run underneath, so detection results are genuine; only
the transport layer is stubbed.

The fake clock can assert that every rospy.Time.now() sample taken
after node construction happens while the node holds its state lock —
this turns the P0-2 fix ("sample all gate clock reads under the lock")
into a deterministic invariant, in addition to a concurrent
watchdog/image interleaving stress test that must never produce a
false rollback.

Covered behaviors:
  1. synthetic valid RGB-D through image_callback
     -> HazardCandidateArray + RUNNING/valid=true
  2. empty valid RGB-D -> empty candidates + RUNNING/valid=true
  3. healthy -> timeout -> exactly one TIMEOUT/valid=false transition,
     generation incremented exactly once
  4. a frame produced before the timeout transition must not recover
  5. a fresh frame produced after the timeout transition recovers
  6. monotonic clock + watchdog/image interleaving -> no false rollback
  7. real rollback -> exactly one new generation, subscriptions rebuilt,
     old-timeline data rejected, new-timeline data recovers
  +  in-flight frame dropped when a timeout/rollback happens while it
     is being processed (generation snapshot)
  +  fail-fast parameter validation
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import os
import sys
import threading
import types
import unittest

import numpy as np

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(TEST_DIR, "..", "scripts")

if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import test_perception  # noqa: E402 — reuse the synthetic scenes

# ---------------------------------------------------------------------------
# Fake clock and time
# ---------------------------------------------------------------------------

class FakeClock(object):
    """Controllable ROS clock.  Optionally asserts that every
    rospy.Time.now() sample happens while the node holds its state lock."""

    def __init__(self):
        self.t = 0.0
        self.lock = None  # node state lock, injected after construction
        self.assert_lock_held = False

    def advance(self, dt):
        self.t += dt

    def set(self, t):
        self.t = float(t)


_CLOCK = FakeClock()


class FakeTime(object):
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    def to_sec(self):
        return self._secs

    def __eq__(self, other):
        return isinstance(other, FakeTime) and self._secs == other._secs

    def __ne__(self, other):
        return not self.__eq__(other)

    def __repr__(self):
        return "FakeTime(%.3f)" % self._secs

    @classmethod
    def now(cls):
        if _CLOCK.assert_lock_held and _CLOCK.lock is not None:
            assert _CLOCK.lock._is_owned(), (
                "rospy.Time.now() sampled outside the node state lock"
            )
        return cls(_CLOCK.t)


class FakeDuration(object):
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    def to_sec(self):
        return self._secs


# ---------------------------------------------------------------------------
# Fake ROS layer state and objects
# ---------------------------------------------------------------------------

class _FakeState(object):
    def __init__(self):
        self.params = {}
        self.publishers = {}
        self.timers = []

    def reset(self):
        self.params.clear()
        self.publishers.clear()
        del self.timers[:]


STATE = _FakeState()

# Global ordering log for cross-object sequencing assertions (e.g. the
# rollback TIMEOUT publish must precede the subscription teardown).
EVENTS = []


class FakePublisher(object):
    def __init__(self, name, data_class, queue_size=None, latch=False):
        self.name = name
        self.data_class = data_class
        self.queue_size = queue_size
        self.latch = latch
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)
        EVENTS.append(
            ("publish:" + self.name, getattr(msg, "state", None))
        )


class FakeTimer(object):
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback


class FakeSubscriber(object):
    instances = []

    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
        self.unregistered = False
        FakeSubscriber.instances.append(self)

    def unregister(self):
        self.unregistered = True
        EVENTS.append(("unregister", self.topic))

    @classmethod
    def reset(cls):
        del cls.instances[:]


class FakeApproximateTimeSynchronizer(object):
    instances = []
    creation_hook = None  # fired mid-reset by tests

    def __init__(self, fs, queue_size, slop, allow_headerless=False):
        self.fs = list(fs)
        self.queue_size = queue_size
        self.slop = slop
        self.allow_headerless = allow_headerless
        self.callback = None
        FakeApproximateTimeSynchronizer.instances.append(self)
        EVENTS.append(("sync_created", None))
        if FakeApproximateTimeSynchronizer.creation_hook is not None:
            FakeApproximateTimeSynchronizer.creation_hook()

    def registerCallback(self, cb):
        self.callback = cb

    @classmethod
    def reset(cls):
        del cls.instances[:]
        cls.creation_hook = None


class FakeCvBridgeError(Exception):
    pass


class FakeBridge(object):
    """
    Stand-in for cv_bridge.CvBridge.  Returns pre-configured numpy
    arrays; an optional one-shot ``on_convert`` hook fires on the first
    conversion of a frame so tests can inject a watchdog tick exactly
    while the image callback is processing.
    """

    def __init__(self):
        self.rgb_array = None
        self.depth_mm = None
        self.depth_f32 = None
        self.on_convert = None

    def imgmsg_to_cv2(self, msg, desired_encoding="passthrough"):
        if self.on_convert is not None:
            hook, self.on_convert = self.on_convert, None
            hook()

        if desired_encoding == "bgr8":
            return self.rgb_array

        encoding = msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            return self.depth_mm
        if encoding in ("32fc1", "32fc"):
            return self.depth_f32

        raise FakeCvBridgeError(
            "unsupported encoding: %s" % msg.encoding
        )


class FakeHeader(object):
    def __init__(self):
        self.stamp = FakeTime(0.0)
        self.frame_id = ""


class FakeImage(object):
    def __init__(self):
        self.header = FakeHeader()
        self.encoding = ""
        self.width = 0
        self.height = 0


class FakeCameraInfo(object):
    def __init__(self, K=None, frame_id="", width=0, height=0):
        self.header = FakeHeader()
        self.header.frame_id = frame_id
        self.K = list(K) if K is not None else [0.0] * 9
        self.width = width
        self.height = height


class FakePoint(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakeHazardCandidate(object):
    def __init__(self):
        self.header = FakeHeader()
        self.position = FakePoint()
        self.label = ""
        self.confidence = 0.0
        self.position_valid = False


class FakeHazardCandidateArray(object):
    def __init__(self):
        self.header = FakeHeader()
        self.candidates = []


class FakeModuleStatus(object):
    UNKNOWN = 0
    STARTING = 1
    READY = 2
    RUNNING = 3
    SUCCEEDED = 4
    BLOCKED = 5
    TIMEOUT = 6
    FAILED = 7

    def __init__(self):
        self.header = FakeHeader()
        self.state = 0
        self.valid = False
        self.confidence = 0.0
        self.detail = ""


# ---------------------------------------------------------------------------
# Fake module installation and node loading
# ---------------------------------------------------------------------------

def _module(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


def _install_fake_modules():
    def _loginfo(msg, *args):  # noqa: ARG001
        pass

    def _logwarn_throttle(period, msg, *args):  # noqa: ARG001
        pass

    def _logerr(msg, *args):  # noqa: ARG001
        pass

    def _logfatal(msg, *args):  # noqa: ARG001
        pass

    def _init_node(name, log_level=None):  # noqa: ARG001
        pass

    def _get_param(name, default=None):
        return STATE.params.get(name, default)

    def _publisher(name, data_class, queue_size=None, latch=False):
        pub = FakePublisher(name, data_class, queue_size, latch)
        STATE.publishers[name] = pub
        return pub

    def _timer(duration, callback):
        timer = FakeTimer(duration, callback)
        STATE.timers.append(timer)
        return timer

    def _spin():
        pass

    def _is_shutdown():
        return False

    rospy_mod = _module(
        "rospy",
        Time=FakeTime,
        Duration=FakeDuration,
        INFO=20,
        WARN=30,
        ERROR=40,
        FATAL=50,
        init_node=_init_node,
        get_param=_get_param,
        Publisher=_publisher,
        Timer=_timer,
        loginfo=_loginfo,
        logwarn_throttle=_logwarn_throttle,
        logerr=_logerr,
        logfatal=_logfatal,
        spin=_spin,
        is_shutdown=_is_shutdown,
    )
    del rospy_mod  # referenced through sys.modules

    _module(
        "message_filters",
        Subscriber=FakeSubscriber,
        ApproximateTimeSynchronizer=FakeApproximateTimeSynchronizer,
    )

    _module("cv_bridge", CvBridge=FakeBridge, CvBridgeError=FakeCvBridgeError)

    sensor_msgs_pkg = types.ModuleType("sensor_msgs")
    sensor_msgs_pkg.__path__ = []
    sys.modules["sensor_msgs"] = sensor_msgs_pkg
    _module("sensor_msgs.msg", Image=FakeImage, CameraInfo=FakeCameraInfo)

    competition_pkg = types.ModuleType("competition_interfaces")
    competition_pkg.__path__ = []
    sys.modules["competition_interfaces"] = competition_pkg
    _module(
        "competition_interfaces.msg",
        HazardCandidate=FakeHazardCandidate,
        HazardCandidateArray=FakeHazardCandidateArray,
        ModuleStatus=FakeModuleStatus,
    )


def _load_perception_node():
    node_path = os.path.join(SCRIPTS, "perception_node")
    loader = importlib.machinery.SourceFileLoader(
        "perception_node",
        node_path,
    )
    spec = importlib.util.spec_from_loader("perception_node", loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules["perception_node"] = module
    loader.exec_module(module)
    return module


_install_fake_modules()
PERCEPTION_NODE = _load_perception_node()

# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

K9 = [554.2563, 0.0, 319.5, 0.0, 554.2563, 239.5, 0.0, 0.0, 1.0]
FRAME = "real_sense_optical_frame"


def make_frame(stamp_sec, depth_encoding="32FC1", rgb_stamp=None,
               rgb_info_stamp=None, depth_stamp=None,
               depth_info_stamp=None):
    rgb = FakeImage()
    rgb.header.stamp = FakeTime(
        stamp_sec if rgb_stamp is None else rgb_stamp
    )
    rgb.header.frame_id = FRAME
    rgb.encoding = "bgr8"
    rgb.width = 640
    rgb.height = 480

    rgb_info = FakeCameraInfo(K=K9, frame_id=FRAME, width=640, height=480)
    rgb_info.header.stamp = FakeTime(
        stamp_sec if rgb_info_stamp is None else rgb_info_stamp
    )

    depth = FakeImage()
    depth.header.stamp = FakeTime(
        stamp_sec if depth_stamp is None else depth_stamp
    )
    depth.header.frame_id = FRAME
    depth.encoding = depth_encoding
    depth.width = 640
    depth.height = 480

    depth_info = FakeCameraInfo(K=K9, frame_id=FRAME, width=640, height=480)
    depth_info.header.stamp = FakeTime(
        stamp_sec if depth_info_stamp is None else depth_info_stamp
    )

    return rgb, rgb_info, depth, depth_info


def run_threads(*functions):
    """Run functions on parallel threads, re-raising their exceptions."""

    errors = []

    def wrapped(fn):
        def runner():
            try:
                fn()
            except BaseException as exc:  # noqa: BLE001
                errors.append(exc)

        return runner

    threads = [threading.Thread(target=wrapped(fn)) for fn in functions]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    if errors:
        raise errors[0]


def statuses(node):
    return node.status_pub.messages


def candidate_publishes(node):
    return node.candidate_pub.messages


class NodeWrapperTest(unittest.TestCase):
    """Wrapper-level tests driving the real PerceptionNode callbacks."""

    def setUp(self):
        FakeSubscriber.reset()
        FakeApproximateTimeSynchronizer.reset()
        STATE.reset()
        del EVENTS[:]
        _CLOCK.t = 1.0
        _CLOCK.lock = None
        _CLOCK.assert_lock_held = False

        self.node = PERCEPTION_NODE.PerceptionNode()
        _CLOCK.lock = self.node._lock

        bgr, depth_m = test_perception.red_sphere_scene()
        self.node.bridge.rgb_array = bgr
        self.node.bridge.depth_f32 = depth_m

    def feed(self, stamp_sec, now_sec=None):
        if now_sec is not None:
            _CLOCK.set(now_sec)
        self.node.image_callback(*make_frame(stamp_sec))

    def test_1_synthetic_rgbd_publishes_candidates_and_running(self):
        self.feed(0.9, now_sec=1.0)

        status = statuses(self.node)[-1]
        self.assertEqual(status.state, FakeModuleStatus.RUNNING)
        self.assertTrue(status.valid)

        publishes = candidate_publishes(self.node)
        self.assertEqual(len(publishes), 1)
        candidates = publishes[-1].candidates
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].label, "red_sphere")
        self.assertTrue(candidates[0].position_valid)
        self.assertAlmostEqual(candidates[0].position.z, 2.0, delta=0.01)
        self.assertEqual(candidates[0].header.frame_id, FRAME)

    def test_2_empty_scene_publishes_empty_candidates_and_running(self):
        bgr, depth_m = test_perception.blank_scene()
        self.node.bridge.rgb_array = bgr
        self.node.bridge.depth_f32 = depth_m

        self.feed(0.9, now_sec=1.0)

        status = statuses(self.node)[-1]
        self.assertEqual(status.state, FakeModuleStatus.RUNNING)
        self.assertTrue(status.valid)

        candidates = candidate_publishes(self.node)[-1].candidates
        self.assertEqual(candidates, [])

    def test_3_timeout_single_transition_single_generation(self):
        self.feed(0.9, now_sec=1.0)
        self.assertTrue(self.node.gate.valid)
        generation_before = self.node.gate.generation_id

        _CLOCK.advance(1.2)  # now == 2.2
        self.node.watchdog_callback(None)

        timeouts = [
            msg for msg in statuses(self.node)
            if msg.state == FakeModuleStatus.TIMEOUT
        ]
        self.assertEqual(len(timeouts), 1)
        self.assertFalse(timeouts[0].valid)
        self.assertFalse(self.node.gate.valid)
        self.assertEqual(
            self.node.gate.generation_id,
            generation_before + 1,
        )

        # Repeated ticks: no further transition.
        _CLOCK.advance(0.1)
        self.node.watchdog_callback(None)
        _CLOCK.advance(0.2)
        self.node.watchdog_callback(None)

        timeouts = [
            msg for msg in statuses(self.node)
            if msg.state == FakeModuleStatus.TIMEOUT
        ]
        self.assertEqual(len(timeouts), 1)
        self.assertEqual(
            self.node.gate.generation_id,
            generation_before + 1,
        )

    def test_4_pre_timeout_frame_cannot_recover(self):
        self.feed(0.9, now_sec=1.0)
        _CLOCK.advance(1.2)  # timeout transition at 2.2
        self.node.watchdog_callback(None)

        # Frame stamped 1.3 arrives at 2.3: age 1.0 s (inside
        # max_input_age) but produced before the transition -> rejected.
        self.feed(1.3, now_sec=2.3)

        self.assertFalse(self.node.gate.valid)
        status = statuses(self.node)[-1]
        self.assertEqual(status.state, FakeModuleStatus.TIMEOUT)
        running = [
            msg for msg in statuses(self.node)
            if msg.state == FakeModuleStatus.RUNNING
        ]
        self.assertEqual(len(running), 1)  # only the healthy accept

    def test_5_post_timeout_fresh_frame_recovers(self):
        self.feed(0.9, now_sec=1.0)
        _CLOCK.advance(1.2)  # timeout at 2.2
        self.node.watchdog_callback(None)
        self.assertFalse(self.node.gate.valid)

        self.feed(2.4, now_sec=2.4)

        self.assertTrue(self.node.gate.valid)
        status = statuses(self.node)[-1]
        self.assertEqual(status.state, FakeModuleStatus.RUNNING)
        self.assertTrue(status.valid)

    def test_6_monotonic_interleaving_no_false_rollback(self):
        self.feed(0.9, now_sec=1.0)
        _CLOCK.assert_lock_held = True

        for _ in range(10):
            _CLOCK.advance(0.1)
            stamp = _CLOCK.t - 0.05
            frame = make_frame(stamp)

            run_threads(
                lambda: self.node.watchdog_callback(None),
                lambda: self.node.image_callback(*frame),
            )

        rollback_statuses = [
            msg for msg in statuses(self.node)
            if "clock moved backward" in msg.detail
        ]
        self.assertEqual(rollback_statuses, [])
        self.assertEqual(self.node.gate.last_clock_time, _CLOCK.t)
        self.assertTrue(self.node.gate.valid)

    def test_7_real_rollback_rebuilds_subscriptions_and_recovers(self):
        _CLOCK.set(10.0)
        self.feed(9.9)  # healthy accept in the old timeline

        sync_count_before = len(FakeApproximateTimeSynchronizer.instances)
        old_subs = list(FakeSubscriber.instances)
        self.assertEqual(len(old_subs), 4)
        generation_before = self.node.gate.generation_id

        _CLOCK.set(1.0)  # rollback
        self.node.watchdog_callback(None)

        # Exactly one new generation, exactly one subscription rebuild.
        self.assertEqual(
            self.node.gate.generation_id,
            generation_before + 1,
        )
        self.assertEqual(
            len(FakeApproximateTimeSynchronizer.instances),
            sync_count_before + 1,
        )
        for sub in old_subs:
            self.assertTrue(sub.unregistered)
        self.assertIs(
            self.node._sync,
            FakeApproximateTimeSynchronizer.instances[-1],
        )
        # Bound methods compare by function identity (each attribute
        # access creates a new bound method object).
        self.assertIs(
            self.node._sync.callback.__func__,
            self.node.image_callback.__func__,
        )

        status = statuses(self.node)[-1]
        self.assertEqual(status.state, FakeModuleStatus.TIMEOUT)
        self.assertIn("clock moved backward", status.detail)
        self.assertFalse(self.node.gate.valid)

        # Repeated ticks: no further transition or rebuild.
        _CLOCK.advance(0.1)
        self.node.watchdog_callback(None)
        self.assertEqual(
            len(FakeApproximateTimeSynchronizer.instances),
            sync_count_before + 1,
        )
        self.assertEqual(
            self.node.gate.generation_id,
            generation_before + 1,
        )

        # Exactly one invalidation transition overall, and the reset
        # completed (pending cleared).
        timeouts = [
            msg for msg in statuses(self.node)
            if msg.state == FakeModuleStatus.TIMEOUT
        ]
        self.assertEqual(len(timeouts), 1)
        self.assertFalse(self.node.rollback_reset_pending)

        # Old-timeline frame must be rejected (far future).
        self.feed(9.9, now_sec=1.1)
        self.assertFalse(self.node.gate.valid)

        # Fresh new-timeline frame recovers.
        self.feed(1.1, now_sec=1.1)
        self.assertTrue(self.node.gate.valid)
        self.assertEqual(
            statuses(self.node)[-1].state,
            FakeModuleStatus.RUNNING,
        )

    def test_mixed_stamps_boundary_rejects_whole_set(self):
        # Regression for the v2 review: EVERY stream of the synchronized
        # set must belong to the current generation.  RGB/RGBInfo stamps
        # (2.15) are pre-boundary while depth stamps (2.25) are current;
        # the stamp spread (0.10) is inside the ATS slop, yet the whole
        # set must be rejected.
        self.feed(0.9, now_sec=1.0)
        _CLOCK.advance(1.2)  # timeout transition at 2.2
        self.node.watchdog_callback(None)

        running_before = len(
            [
                msg for msg in statuses(self.node)
                if msg.state == FakeModuleStatus.RUNNING
            ]
        )
        candidates_before = len(candidate_publishes(self.node))

        _CLOCK.set(2.3)
        frame = make_frame(2.25, rgb_stamp=2.15, rgb_info_stamp=2.15)
        self.node.image_callback(*frame)

        self.assertFalse(self.node.gate.valid)
        self.assertEqual(
            len(
                [
                    msg for msg in statuses(self.node)
                    if msg.state == FakeModuleStatus.RUNNING
                ]
            ),
            running_before,
        )
        self.assertEqual(
            len(candidate_publishes(self.node)),
            candidates_before,
        )

    def test_aux_stale_stamp_rejects_set(self):
        # A stale auxiliary stream (RGB stamped 0.5 at now 2.3, age 1.8)
        # rejects the whole set even though the depth stamp is fresh.
        self.feed(0.9, now_sec=1.0)
        _CLOCK.advance(1.2)
        self.node.watchdog_callback(None)

        running_before = len(
            [
                msg for msg in statuses(self.node)
                if msg.state == FakeModuleStatus.RUNNING
            ]
        )

        _CLOCK.set(2.3)
        frame = make_frame(2.25, rgb_stamp=0.5)
        self.node.image_callback(*frame)

        self.assertFalse(self.node.gate.valid)
        self.assertEqual(
            len(
                [
                    msg for msg in statuses(self.node)
                    if msg.state == FakeModuleStatus.RUNNING
                ]
            ),
            running_before,
        )

    def test_rollback_publish_precedes_subscription_reset(self):
        # The TIMEOUT/valid=false invalidation must be observable BEFORE
        # any teardown/rebuild of the subscriptions, and only once.
        _CLOCK.set(10.0)
        self.feed(9.9)
        del EVENTS[:]

        _CLOCK.set(1.0)  # rollback
        self.node.watchdog_callback(None)

        events = [
            event for event in EVENTS
            if event[0] in (
                "publish:/competition/perception/status",
                "unregister",
                "sync_created",
            )
        ]
        self.assertEqual(
            events[0],
            (
                "publish:/competition/perception/status",
                FakeModuleStatus.TIMEOUT,
            ),
        )
        self.assertEqual(events[-1][0], "sync_created")
        for event in events[1:-1]:
            self.assertEqual(event[0], "unregister")

        timeout_publishes = [
            event for event in events
            if event
            == (
                "publish:/competition/perception/status",
                FakeModuleStatus.TIMEOUT,
            )
        ]
        self.assertEqual(len(timeout_publishes), 1)
        self.assertFalse(self.node.rollback_reset_pending)

    def test_reset_pending_window_image_callback_cannot_recover(self):
        # An image callback running while the subscription reset is
        # pending must not accept observations or publish RUNNING.
        _CLOCK.set(10.0)
        self.feed(9.9)

        def during_reset():
            self.assertTrue(self.node.rollback_reset_pending)
            self.feed(1.05, now_sec=1.05)

        FakeApproximateTimeSynchronizer.creation_hook = during_reset
        _CLOCK.set(1.0)  # rollback
        self.node.watchdog_callback(None)
        FakeApproximateTimeSynchronizer.creation_hook = None

        self.assertFalse(self.node.gate.valid)
        self.assertFalse(self.node.rollback_reset_pending)

        # After the reset completes, a fresh current-generation frame
        # recovers normally.
        self.feed(1.1, now_sec=1.1)
        self.assertTrue(self.node.gate.valid)

    def test_inflight_timeout_during_processing_drops_frame(self):
        bgr, depth_m = test_perception.blank_scene()
        self.node.bridge.rgb_array = bgr
        self.node.bridge.depth_f32 = depth_m

        self.feed(0.9, now_sec=1.0)
        publishes_before = len(candidate_publishes(self.node))
        generation_before = self.node.gate.generation_id

        # The frame is admitted at t=2.0; while it is being processed
        # (inside cv_bridge conversion) the watchdog fires the timeout
        # transition at t=2.2.
        def hook():
            _CLOCK.set(2.2)
            self.node.watchdog_callback(None)

        self.node.bridge.on_convert = hook
        self.feed(2.0, now_sec=2.0)

        # The in-flight frame belongs to the old generation: dropped.
        self.assertEqual(
            len(candidate_publishes(self.node)),
            publishes_before,
        )
        timeouts = [
            msg for msg in statuses(self.node)
            if msg.state == FakeModuleStatus.TIMEOUT
        ]
        self.assertEqual(len(timeouts), 1)
        self.assertFalse(self.node.gate.valid)
        self.assertEqual(
            self.node.gate.generation_id,
            generation_before + 1,
        )

    def test_inflight_rollback_during_processing_drops_frame(self):
        _CLOCK.set(10.0)
        self.feed(9.9)
        publishes_before = len(candidate_publishes(self.node))
        sync_count_before = len(FakeApproximateTimeSynchronizer.instances)

        # The frame is admitted at t=10.0; while it is being processed
        # the watchdog detects the clock rollback to 1.0.
        def hook():
            _CLOCK.set(1.0)
            self.node.watchdog_callback(None)

        self.node.bridge.on_convert = hook
        self.feed(9.95, now_sec=10.0)

        self.assertEqual(
            len(candidate_publishes(self.node)),
            publishes_before,
        )
        self.assertEqual(
            len(FakeApproximateTimeSynchronizer.instances),
            sync_count_before + 1,
        )
        rollback_statuses = [
            msg for msg in statuses(self.node)
            if "clock moved backward" in msg.detail
        ]
        self.assertEqual(len(rollback_statuses), 1)
        self.assertFalse(self.node.gate.valid)

    def test_depth_to_meters_readonly_32fc1_is_writable_and_sanitized(self):
        # Production blocker regression: cv_bridge passthrough returns a
        # READ-ONLY view of the ROS message buffer for 32FC1, and the
        # real competition plugin publishes exactly this encoding.
        raw = np.full((480, 640), 2.0, dtype=np.float32)
        raw[100, 100] = float("nan")
        raw[101, 101] = float("inf")
        raw[102, 102] = float("-inf")
        raw.setflags(write=False)
        self.assertFalse(raw.flags.writeable)

        self.node.bridge.depth_f32 = raw
        msg = make_frame(0.9, depth_encoding="32FC1")[2]
        original = raw.copy()

        result = self.node.depth_to_meters(msg)  # must not raise

        self.assertIsNotNone(result)
        self.assertEqual(result.dtype, np.float32)
        self.assertTrue(result.flags.writeable)
        self.assertFalse(np.shares_memory(result, raw))
        self.assertEqual(result[100, 100], 0.0)  # NaN -> 0
        self.assertEqual(result[101, 101], 0.0)  # +Inf -> 0
        self.assertEqual(result[102, 102], 0.0)  # -Inf -> 0
        self.assertEqual(result[103, 103], 2.0)
        np.testing.assert_array_equal(raw, original)  # source untouched

    def test_readonly_32fc1_depth_through_image_callback(self):
        # Full production path regression: a read-only float32 depth
        # stream must be PROCESSED (not skipped), publishing candidates
        # and RUNNING/valid=true without raising.
        bgr, _ = test_perception.red_sphere_scene()
        self.node.bridge.rgb_array = bgr

        depth_ro = np.full((480, 640), 2.0, dtype=np.float32)
        depth_ro[100, 100] = float("nan")
        depth_ro.setflags(write=False)
        self.node.bridge.depth_f32 = depth_ro

        frame = make_frame(0.9, depth_encoding="32FC1")
        self.node.image_callback(*frame)  # must not raise

        status = statuses(self.node)[-1]
        self.assertEqual(status.state, FakeModuleStatus.RUNNING)
        self.assertTrue(status.valid)

        candidates = candidate_publishes(self.node)[-1].candidates
        self.assertEqual(len(candidates), 1)
        self.assertTrue(candidates[0].position_valid)
        self.assertAlmostEqual(candidates[0].position.z, 2.0, delta=0.01)

    def test_16uc1_path_unchanged(self):
        # The millimetre path must keep its existing behavior, including
        # on a read-only source buffer.
        mm = np.full((480, 640), 2500, dtype=np.uint16)
        mm.setflags(write=False)
        self.node.bridge.depth_mm = mm

        msg = make_frame(0.9, depth_encoding="16UC1")[2]
        result = self.node.depth_to_meters(msg)  # must not raise

        self.assertIsNotNone(result)
        self.assertEqual(result.dtype, np.float32)
        self.assertTrue(result.flags.writeable)
        self.assertFalse(np.shares_memory(result, mm))
        self.assertAlmostEqual(result[240, 320], 2.5, delta=1e-6)

    def test_param_validation_fails_fast(self):
        bad_cases = [
            ("~input_timeout_sec", 0.0),
            ("~input_timeout_sec", float("nan")),
            ("~watchdog_period_sec", -1.0),
            ("~max_input_age_sec", 0.0),
            ("~future_stamp_tolerance_sec", -0.1),
            ("~min_contour_area", 0.0),
            ("~min_depth", -1.0),
            ("~max_depth", 0.3),  # <= default min_depth 0.4
        ]

        for key, value in bad_cases:
            STATE.params[key] = value
            try:
                with self.assertRaises(ValueError):
                    PERCEPTION_NODE.PerceptionNode()
            finally:
                STATE.params.pop(key, None)


if __name__ == "__main__":
    unittest.main()
