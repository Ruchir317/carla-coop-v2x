import argparse
import math
import random
import time
from typing import List, Optional

import carla

from intersection_manager import IntersectionManager, VehicleState
from logging_utils import MetricsLogger
from vehicle_agent import VehicleAgent


def get_client(host: str, port: int) -> carla.Client:
    client = carla.Client(host, port)
    client.set_timeout(5.0)
    return client


def enable_synchronous_mode(world: carla.World, delta_seconds: float = 0.05):
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = delta_seconds
    settings.substepping = False
    world.apply_settings(settings)


def spawn_vehicles(world: carla.World, count: int, spawn_indices: Optional[List[int]] = None) -> List[carla.Vehicle]:
    blueprint_library = world.get_blueprint_library()
    bp_candidates = [
        bp for bp in blueprint_library.filter("vehicle.*")
        if int(bp.get_attribute("number_of_wheels").as_int()) == 4
    ]

    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < count:
        raise RuntimeError(f"Not enough spawn points for {count} vehicles")
    if spawn_indices is None:
        random.shuffle(spawn_points)
    else:
        spawn_points = [spawn_points[i] for i in spawn_indices[:count]]

    vehicles = []
    for i in range(count):
        blueprint = random.choice(bp_candidates)
        blueprint.set_attribute("role_name", f"ego_{i}")
        vehicle = world.spawn_actor(blueprint, spawn_points[i])
        vehicle.set_autopilot(True)
        vehicles.append(vehicle)
        print(f"Spawned vehicle {vehicle.id} ({vehicle.type_id}) at {spawn_points[i].location}")
    return vehicles


def parse_indices(raw: Optional[str]) -> Optional[List[int]]:
    if raw is None:
        return None
    try:
        return [int(x) for x in raw.split(",") if x.strip() != ""]
    except ValueError:
        raise ValueError(f"Could not parse indices from '{raw}'")


def average_location(points: List[carla.Transform]) -> carla.Location:
    loc = carla.Location()
    for t in points:
        loc.x += t.location.x
        loc.y += t.location.y
        loc.z += t.location.z
    n = max(len(points), 1)
    loc.x /= n
    loc.y /= n
    loc.z /= n
    return loc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default="Town05", help="CARLA map to load (e.g., Town05)")
    parser.add_argument("--vehicles", type=int, default=3, help="number of test vehicles to spawn")
    parser.add_argument("--delta", type=float, default=0.05, help="fixed delta seconds for sync mode")
    parser.add_argument("--approach", type=float, default=25.0, help="approach radius (m) to join queue")
    parser.add_argument("--box", type=float, default=8.0, help="half-extent (m) of intersection box")
    parser.add_argument("--center-x", type=float, default=None, help="intersection center X (optional)")
    parser.add_argument("--center-y", type=float, default=None, help="intersection center Y (optional)")
    parser.add_argument("--center-z", type=float, default=None, help="intersection center Z (optional)")
    parser.add_argument("--spawn-indices", type=str, default=None, help="comma-separated spawn point indices")
    parser.add_argument("--dest-indices", type=str, default=None, help="comma-separated destination indices")
    parser.add_argument("--logfile", type=str, default="metrics.csv", help="CSV log output")
    parser.add_argument("--list-spawns", action="store_true", help="list spawn points and exit")
    parser.add_argument("--list-junctions", action="store_true", help="list junction centers/extents and exit")
    args = parser.parse_args()

    client = get_client(args.host, args.port)
    world = client.get_world()
    if args.map not in world.get_map().name:
        world = client.load_world(args.map)
    spawn_points = world.get_map().get_spawn_points()

    if args.list_spawns:
        for idx, sp in enumerate(spawn_points):
            loc = sp.location
            print(f"{idx}: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
        return
    if args.list_junctions:
        junctions = world.get_map().get_junctions()
        for j_idx, j in enumerate(junctions):
            c = j.bounding_box.location
            e = j.bounding_box.extent
            print(
                f"{j_idx}: center=({c.x:.2f}, {c.y:.2f}, {c.z:.2f}) "
                f"extent=({e.x:.2f}, {e.y:.2f}, {e.z:.2f})"
            )
        return

    original_settings = world.get_settings()
    traffic_manager = client.get_trafficmanager()
    vehicles: List[carla.Vehicle] = []
    agents: List[VehicleAgent] = []
    manager: IntersectionManager | None = None
    logger: Optional[MetricsLogger] = None
    try:
        enable_synchronous_mode(world, args.delta)
        traffic_manager.set_synchronous_mode(True)

        spawn_indices = parse_indices(args.spawn_indices)
        dest_indices = parse_indices(args.dest_indices)
        vehicles = spawn_vehicles(world, args.vehicles, spawn_indices=spawn_indices)

        agents = []
        for i, v in enumerate(vehicles):
            dest_idx = dest_indices[i] if dest_indices and i < len(dest_indices) else None
            destination = spawn_points[dest_idx].location if dest_idx is not None else spawn_points[-1].location
            agents.append(VehicleAgent(world, v, target_speed_kmh=25, destination=destination))

        # Choose intersection center:
        # - user-specified center_x/center_y if provided
        # - otherwise, average of spawn_indices if provided
        # - fallback to first spawn point location
        if args.center_x is not None and args.center_y is not None:
            center_location = carla.Location(x=args.center_x, y=args.center_y,
                                             z=args.center_z if args.center_z is not None else spawn_points[0].location.z)
        elif spawn_indices:
            subset = [spawn_points[i] for i in spawn_indices]
            center_location = average_location(subset)
        else:
            center_location = spawn_points[0].location

        manager = IntersectionManager(center_location, approach_radius=args.approach, box_half_extent=args.box)
        logger = MetricsLogger(args.logfile)

        start_sim_time: Optional[float] = None
        while True:
            world.tick()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds
            if start_sim_time is None:
                start_sim_time = timestamp

            manager.update(agents, timestamp)
            permissions = manager.current_permissions()

            for agent in agents:
                go = permissions.get(agent.vehicle.id, False)
                agent.step(go)

            # Log any vehicles that have cleared the box.
            if logger is not None:
                completed = manager.poll_completed()
                for state in completed:
                    logger.log_state(state)

            # Basic exit after 2 simulated minutes to avoid runaway runs.
            if start_sim_time is not None and (timestamp - start_sim_time) > 120:
                break
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        for agent in agents:
            agent.destroy()
        if logger is not None:
            logger.close()
        for vehicle in vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)


if __name__ == "__main__":
    main()
