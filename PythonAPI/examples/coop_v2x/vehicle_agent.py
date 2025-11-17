from __future__ import annotations

from typing import Optional

import carla
from agents.navigation.basic_agent import BasicAgent


class VehicleAgent:
    """
    Wraps a CARLA vehicle with BasicAgent and exposes a stop/go step.
    """

    def __init__(self, world: carla.World, vehicle: carla.Vehicle, target_speed_kmh: float = 25.0,
                 destination: Optional[carla.Location] = None):
        self.world = world
        self.vehicle = vehicle
        self.agent = BasicAgent(vehicle, target_speed=target_speed_kmh)
        # Pick a destination further along the map; BasicAgent will replan as needed.
        spawn_points = world.get_map().get_spawn_points()
        dest = destination if destination is not None else spawn_points[-1].location
        self.agent.set_destination(dest)

    def step(self, permission_to_go: bool):
        if not permission_to_go:
            control = carla.VehicleControl(throttle=0.0, brake=1.0)
        else:
            control = self.agent.run_step()
        self.vehicle.apply_control(control)

    def distance_to(self, location: carla.Location) -> float:
        return self.vehicle.get_location().distance(location)

    @property
    def is_alive(self) -> bool:
        return self.vehicle.is_alive

    def destroy(self):
        if self.vehicle.is_alive:
            self.vehicle.destroy()
