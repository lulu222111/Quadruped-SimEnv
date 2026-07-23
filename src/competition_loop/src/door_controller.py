#!/usr/bin/env python3
"""door_controller.py — thin wrapper around building door/elevator services.

This module calls the existing /set_door_state and /call_elevator services
provided by building_generator_classic. It does NOT implement autonomous
elevator strategies.
"""

from __future__ import annotations

import rospy
from building_generator_interfaces.srv import SetDoorState, SetDoorStateRequest
from building_generator_interfaces.srv import CallElevator, CallElevatorRequest


class DoorController:
    """Minimal client for door and elevator ROS services."""

    DOOR_SERVICE = "/set_door_state"
    ELEVATOR_SERVICE = "/call_elevator"

    def __init__(self, timeout: float = 30.0):
        self._timeout = rospy.Duration.from_sec(float(timeout))

    # ------------------------------------------------------------------
    # Door operations
    # ------------------------------------------------------------------

    def _ensure_door_service(self):
        try:
            rospy.wait_for_service(self.DOOR_SERVICE, timeout=self._timeout)
        except rospy.ROSException as exc:
            raise RuntimeError(
                f"Door service {self.DOOR_SERVICE} not available within "
                f"{self._timeout.to_sec():.1f} s"
            ) from exc

    def open_door(self, door_id: str) -> dict:
        """Open a dynamic door. Returns the service response as a dict."""
        self._ensure_door_service()
        req = SetDoorStateRequest(door_id=door_id, open=True)
        try:
            resp = rospy.ServiceProxy(self.DOOR_SERVICE, SetDoorState).call(req)
        except rospy.ServiceException as exc:
            raise RuntimeError(f"open_door('{door_id}') failed: {exc}") from exc
        return {"accepted": resp.accepted, "state": resp.state, "message": resp.message}

    def close_door(self, door_id: str) -> dict:
        """Close a dynamic door."""
        self._ensure_door_service()
        req = SetDoorStateRequest(door_id=door_id, open=False)
        try:
            resp = rospy.ServiceProxy(self.DOOR_SERVICE, SetDoorState).call(req)
        except rospy.ServiceException as exc:
            raise RuntimeError(f"close_door('{door_id}') failed: {exc}") from exc
        return {"accepted": resp.accepted, "state": resp.state, "message": resp.message}

    # ------------------------------------------------------------------
    # Elevator operations
    # ------------------------------------------------------------------

    def _ensure_elevator_service(self):
        try:
            rospy.wait_for_service(self.ELEVATOR_SERVICE, timeout=self._timeout)
        except rospy.ROSException as exc:
            raise RuntimeError(
                f"Elevator service {self.ELEVATOR_SERVICE} not available within "
                f"{self._timeout.to_sec():.1f} s"
            ) from exc

    def call_elevator(self, elevator_id: str, target_floor: int, open_doors: bool = False) -> dict:
        """Call an elevator to the target floor.

        Args:
            elevator_id: e.g. 'elevator_main'
            target_floor: 0 = 1F, 1 = 2F, 2 = 3F
            open_doors: whether to open doors after arrival.
        """
        self._ensure_elevator_service()
        req = CallElevatorRequest(
            elevator_id=elevator_id,
            target_floor=int(target_floor),
            open_doors=bool(open_doors),
        )
        try:
            resp = rospy.ServiceProxy(self.ELEVATOR_SERVICE, CallElevator).call(req)
        except rospy.ServiceException as exc:
            raise RuntimeError(f"call_elevator('{elevator_id}', floor={target_floor}) failed: {exc}") from exc
        return {
            "accepted": resp.accepted,
            "current_floor": resp.current_floor,
            "state": resp.state,
            "message": resp.message,
        }
