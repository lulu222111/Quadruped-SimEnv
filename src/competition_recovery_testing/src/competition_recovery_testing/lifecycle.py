"""Deterministic lifecycle ordering for a test-only Gazebo teleport.

The backend owns all ROS and process details.  Keeping the ordering here free
of ROS makes the safety boundary independently regression-testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List


class RecoveryError(RuntimeError):
    """A recovery could not establish a fresh, safe generation."""


@dataclass(frozen=True)
class FreshEvidence:
    """Evidence that one stage belongs to the active recovery generation."""

    generation: int
    stamp: float
    observed_at: float


@dataclass
class RecoveryRecord:
    generation: int
    reason: str
    events: List[str] = field(default_factory=list)
    times: Dict[str, float] = field(default_factory=dict)
    succeeded: bool = False
    error: str = ""


class RecoveryLifecycle:
    """Execute an atomic test recovery around a pose discontinuity.

    Backend methods are intentionally explicit.  There is no generic callback
    which could accidentally reorder teleport, process restart or readiness.
    """

    def __init__(self, backend, require_map=True, require_path=True,
                 resume_control=True):
        self._backend = backend
        self._require_map = bool(require_map)
        self._require_path = bool(require_path)
        self._resume_control = bool(resume_control)
        self._generation = 0
        self.records: List[RecoveryRecord] = []

    @property
    def generation(self):
        return self._generation

    def recover(self, reason):
        self._generation += 1
        record = RecoveryRecord(self._generation, str(reason))
        self.records.append(record)
        generation = record.generation

        try:
            self._event(record, "recovery_started")
            self._backend.begin_generation(generation)

            # Keep the authoritative loop/guard alive until a zero command is
            # observed.  The harness never publishes competing safety topics.
            self._backend.request_safety_stop()
            self._backend.wait_safety_stop()
            record.times["CONTROL_STOP_TIME"] = self._backend.now()
            self._event(record, "control_stopped")

            self._backend.wait_cmd_vel_zero()
            record.times["CMD_VEL_ZERO_TIME"] = self._backend.now()
            self._event(record, "cmd_vel_zero")

            self._backend.stop_controller()
            self._event(record, "controller_stopped")
            self._backend.stop_pose_derived_dut()
            self._event(record, "pose_derived_dut_stopped")

            self._backend.teleport()
            teleport_time = self._backend.now()
            record.times["TELEPORT_TIME"] = teleport_time
            self._event(record, "teleport_complete")

            # The low-level controller may stand the robot while motion stays
            # gated.  It starts with no retained command because it is a fresh
            # process and the competition loop will restart in WAITING.
            self._backend.start_controller_safe()
            self._event(record, "controller_started_safe")
            self._backend.wait_sensors_stable(teleport_time)
            self._event(record, "sensors_stable")

            # start_pose_derived_dut must start the competition loop with
            # auto_start=false.  This preserves stop/estop until every required
            # current-generation input is ready.
            self._backend.start_pose_derived_dut_safe()
            self._event(record, "pose_derived_dut_started_safe")

            fastlio = self._backend.wait_fresh_fastlio(teleport_time, generation)
            self._accept_fresh(record, "FIRST_FRESH_FASTLIO_ODOM_TIME",
                               fastlio, teleport_time)

            localization = self._backend.wait_fresh_localization(
                teleport_time, generation
            )
            self._accept_fresh(record, "FIRST_FRESH_LOCALIZATION_VALID_TIME",
                               localization, teleport_time)

            if self._require_map:
                mapping = self._backend.wait_fresh_map(teleport_time, generation)
                self._accept_fresh(record, "FIRST_FRESH_MAP_TIME",
                                   mapping, teleport_time)

            if self._require_path:
                path = self._backend.wait_fresh_path(teleport_time, generation)
                self._accept_fresh(record, "FIRST_FRESH_PATH_TIME",
                                   path, teleport_time)

            if self._resume_control:
                self._backend.reenable_control()
                self._backend.wait_control_reenabled()
                reenable_time = self._backend.now()
                required_times = [
                    value for key, value in record.times.items()
                    if key.startswith("FIRST_FRESH_")
                ]
                if required_times and reenable_time < max(required_times):
                    raise RecoveryError(
                        "control re-enabled before all fresh-state boundaries"
                    )
                record.times["CONTROL_REENABLE_TIME"] = reenable_time
                self._event(record, "control_reenabled")

            record.succeeded = True
            self._event(record, "recovery_complete")
            return record
        except Exception as exc:
            record.error = str(exc)
            self._event(record, "recovery_failed")
            try:
                self._backend.enforce_safe_stop()
                self._event(record, "failure_safe_stop_enforced")
            except Exception as safety_exc:
                record.error += "; failure safe-stop error: {}".format(safety_exc)
            if isinstance(exc, RecoveryError):
                raise
            raise RecoveryError(str(exc)) from exc

    def _accept_fresh(self, record, key, evidence, teleport_time):
        if not isinstance(evidence, FreshEvidence):
            raise RecoveryError("{} did not return FreshEvidence".format(key))
        if evidence.generation != record.generation:
            raise RecoveryError(
                "{} belongs to generation {}, expected {}".format(
                    key, evidence.generation, record.generation
                )
            )
        if evidence.stamp <= teleport_time:
            raise RecoveryError(
                "{} stamp {:.9f} is not newer than teleport {:.9f}".format(
                    key, evidence.stamp, teleport_time
                )
            )
        if evidence.observed_at < teleport_time:
            raise RecoveryError(
                "{} was observed before the teleport boundary".format(key)
            )
        record.times[key] = evidence.observed_at
        self._event(record, key.lower())

    def _event(self, record, name):
        record.events.append(name)
