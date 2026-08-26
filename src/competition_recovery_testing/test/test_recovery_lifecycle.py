#!/usr/bin/env python3

import unittest

from competition_recovery_testing.lifecycle import (
    FreshEvidence,
    RecoveryError,
    RecoveryLifecycle,
)


class FakeBackend:
    def __init__(self):
        self.clock = 10.0
        self.calls = []
        self.generation = 0
        self.stale_stage = None
        self.fail_stage = None
        self.safe_stop_count = 0

    def _call(self, name):
        self.calls.append(name)
        if self.fail_stage == name:
            raise RuntimeError("injected {} failure".format(name))

    def now(self):
        self.clock += 0.1
        return self.clock

    def begin_generation(self, generation):
        self._call("begin_generation")
        self.generation = generation

    def request_safety_stop(self): self._call("request_safety_stop")
    def wait_safety_stop(self): self._call("wait_safety_stop")
    def wait_cmd_vel_zero(self): self._call("wait_cmd_vel_zero")
    def stop_controller(self): self._call("stop_controller")
    def stop_pose_derived_dut(self): self._call("stop_pose_derived_dut")
    def teleport(self): self._call("teleport")
    def start_controller_safe(self): self._call("start_controller_safe")
    def wait_sensors_stable(self, _boundary): self._call("wait_sensors_stable")
    def start_pose_derived_dut_safe(self): self._call("start_pose_derived_dut_safe")
    def reenable_control(self): self._call("reenable_control")
    def wait_control_reenabled(self): self._call("wait_control_reenabled")

    def enforce_safe_stop(self):
        self._call("enforce_safe_stop")
        self.safe_stop_count += 1

    def _fresh(self, stage, boundary, generation):
        self._call(stage)
        self.clock = max(self.clock + 1.0, boundary + 1.0)
        evidence_generation = generation
        stamp = self.clock
        if self.stale_stage == stage:
            evidence_generation = max(0, generation - 1)
            stamp = boundary
        return FreshEvidence(evidence_generation, stamp, self.clock)

    def wait_fresh_fastlio(self, boundary, generation):
        return self._fresh("wait_fresh_fastlio", boundary, generation)

    def wait_fresh_localization(self, boundary, generation):
        return self._fresh("wait_fresh_localization", boundary, generation)

    def wait_fresh_map(self, boundary, generation):
        return self._fresh("wait_fresh_map", boundary, generation)

    def wait_fresh_path(self, boundary, generation):
        return self._fresh("wait_fresh_path", boundary, generation)


class RecoveryLifecycleTest(unittest.TestCase):
    def test_full_recovery_is_strictly_ordered_and_reenables_last(self):
        backend = FakeBackend()
        lifecycle = RecoveryLifecycle(backend)
        record = lifecycle.recover("fall")

        self.assertTrue(record.succeeded)
        self.assertEqual(
            backend.calls,
            [
                "begin_generation",
                "request_safety_stop", "wait_safety_stop",
                "wait_cmd_vel_zero", "stop_controller",
                "stop_pose_derived_dut", "teleport",
                "start_controller_safe", "wait_sensors_stable",
                "start_pose_derived_dut_safe", "wait_fresh_fastlio",
                "wait_fresh_localization", "wait_fresh_map",
                "wait_fresh_path", "reenable_control",
                "wait_control_reenabled",
            ],
        )
        fresh = [value for key, value in record.times.items()
                 if key.startswith("FIRST_FRESH_")]
        self.assertGreaterEqual(record.times["CONTROL_REENABLE_TIME"], max(fresh))
        self.assertLess(record.times["CMD_VEL_ZERO_TIME"],
                        record.times["TELEPORT_TIME"])

    def test_old_localization_generation_cannot_recover_system(self):
        backend = FakeBackend()
        backend.stale_stage = "wait_fresh_localization"
        lifecycle = RecoveryLifecycle(backend)
        with self.assertRaises(RecoveryError):
            lifecycle.recover("fall")
        self.assertNotIn("wait_fresh_map", backend.calls)
        self.assertNotIn("reenable_control", backend.calls)
        self.assertEqual(backend.safe_stop_count, 1)

    def test_old_map_cannot_revive_downstream(self):
        backend = FakeBackend()
        backend.stale_stage = "wait_fresh_map"
        with self.assertRaises(RecoveryError):
            RecoveryLifecycle(backend).recover("fall")
        self.assertNotIn("wait_fresh_path", backend.calls)
        self.assertNotIn("reenable_control", backend.calls)

    def test_old_path_cannot_reenable_motion(self):
        backend = FakeBackend()
        backend.stale_stage = "wait_fresh_path"
        with self.assertRaises(RecoveryError):
            RecoveryLifecycle(backend).recover("fall")
        self.assertNotIn("reenable_control", backend.calls)
        self.assertEqual(backend.safe_stop_count, 1)

    def test_fresh_localization_precedes_all_downstream_recovery(self):
        backend = FakeBackend()
        record = RecoveryLifecycle(backend).recover("fall")
        self.assertLess(
            record.times["FIRST_FRESH_LOCALIZATION_VALID_TIME"],
            record.times["FIRST_FRESH_MAP_TIME"],
        )
        self.assertLess(
            record.times["FIRST_FRESH_MAP_TIME"],
            record.times["FIRST_FRESH_PATH_TIME"],
        )

    def test_multiple_recoveries_get_distinct_generations(self):
        backend = FakeBackend()
        lifecycle = RecoveryLifecycle(backend)
        first = lifecycle.recover("fall-1")
        second = lifecycle.recover("fall-2")
        self.assertEqual((first.generation, second.generation), (1, 2))
        self.assertTrue(first.succeeded and second.succeeded)
        self.assertEqual(backend.calls.count("stop_pose_derived_dut"), 2)
        self.assertEqual(backend.calls.count("teleport"), 2)

    def test_failure_keeps_control_disabled(self):
        backend = FakeBackend()
        backend.fail_stage = "wait_sensors_stable"
        with self.assertRaises(RecoveryError):
            RecoveryLifecycle(backend).recover("fall")
        self.assertNotIn("reenable_control", backend.calls)
        self.assertEqual(backend.safe_stop_count, 1)

    def test_stationary_localization_only_recovery_never_reenables(self):
        backend = FakeBackend()
        lifecycle = RecoveryLifecycle(
            backend, require_map=False, require_path=False,
            resume_control=False,
        )
        record = lifecycle.recover("S1 fall")
        self.assertTrue(record.succeeded)
        self.assertNotIn("wait_fresh_map", backend.calls)
        self.assertNotIn("wait_fresh_path", backend.calls)
        self.assertNotIn("reenable_control", backend.calls)


if __name__ == "__main__":
    unittest.main()
