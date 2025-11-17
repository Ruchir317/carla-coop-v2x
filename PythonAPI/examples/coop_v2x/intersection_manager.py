from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import carla

from vehicle_agent import VehicleAgent


@dataclass
class VehicleState:
    vehicle_id: int
    arrival_time: float
    in_box: bool = False
    cleared: bool = False
    permission_time: Optional[float] = None
    enter_time: Optional[float] = None
    exit_time: Optional[float] = None
    exported: bool = False


class IntersectionManager:
    """
    Minimal FCFS intersection manager.
    - Tracks vehicles entering an approach radius.
    - Grants permission to one vehicle at a time.
    - Clears permission after the vehicle passes the intersection box.
    """

    def __init__(self, center: carla.Location, approach_radius: float = 25.0, box_half_extent: float = 8.0):
        self.center = center
        self.approach_radius = approach_radius
        self.box_half_extent = box_half_extent
        self._states: Dict[int, VehicleState] = {}
        self._current_id: Optional[int] = None

    def update(self, agents: List[VehicleAgent], timestamp: float):
        # Register arrivals.
        for agent in agents:
            vid = agent.vehicle.id
            dist = agent.distance_to(self.center)
            state = self._states.get(vid)
            if dist <= self.approach_radius and state is None:
                self._states[vid] = VehicleState(vehicle_id=vid, arrival_time=timestamp)

        # Drop cleared/removed vehicles from tracking.
        dead_ids = [vid for vid in self._states if not any(a.vehicle.id == vid for a in agents)]
        for vid in dead_ids:
            self._states.pop(vid, None)
            if self._current_id == vid:
                self._current_id = None

        # Update in_box flags and clear exiting vehicles.
        for agent in agents:
            vid = agent.vehicle.id
            state = self._states.get(vid)
            if state is None:
                continue
            in_box = self._is_in_box(agent.vehicle.get_location())
            if in_box and not state.in_box:
                state.in_box = True
                if state.enter_time is None:
                    state.enter_time = timestamp
            if state.in_box and not in_box and dist > self.approach_radius * 0.75:
                state.cleared = True
                if state.exit_time is None:
                    state.exit_time = timestamp
                if self._current_id == vid:
                    self._current_id = None

        # Assign permission to the next waiting vehicle if none is active.
        if self._current_id is None:
            waiting = [
                (vid, st.arrival_time)
                for vid, st in self._states.items()
                if not st.cleared
            ]
            if waiting:
                waiting.sort(key=lambda item: item[1])
                self._current_id = waiting[0][0]
                state = self._states.get(self._current_id)
                if state and state.permission_time is None:
                    state.permission_time = timestamp

    def current_permissions(self) -> Dict[int, bool]:
        if self._current_id is None:
            return {}
        return {self._current_id: True}

    def poll_completed(self) -> List[VehicleState]:
        finished: List[VehicleState] = []
        for vid, state in self._states.items():
            if state.cleared and not state.exported:
                finished.append(state)
                state.exported = True
        return finished

    def _is_in_box(self, location: carla.Location) -> bool:
        # Axis-aligned box around center. Adjust for your target junction geometry if needed.
        return (
            abs(location.x - self.center.x) <= self.box_half_extent and
            abs(location.y - self.center.y) <= self.box_half_extent
        )
