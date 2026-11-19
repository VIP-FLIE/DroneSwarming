# Copyright 2026 DroneSwarming maintainers.
"""Expose pure mission translation without creating flight command clients."""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from swarm_interfaces.msg import GeographicPosition
from swarm_interfaces.srv import PlanFlight

from .compiler import PlanningLimits, StartingPosition, compile_plan
from .geography import GeographicOrigin


class MissionPlanner(Node):
    """Compile a vehicle's selected fleet assignment from a supplied start snapshot."""

    def __init__(self, namespace: str | None = None) -> None:
        """Create a planning service with immutable geometry and flight bounds."""
        super().__init__('mission_planner', namespace=namespace)
        descriptor = ParameterDescriptor(read_only=True)
        self.limits = PlanningLimits(
            float(self.declare_parameter('maximum_altitude_m', 30.0, descriptor).value),
            float(self.declare_parameter('maximum_horizontal_distance_m', 100.0, descriptor).value),
            float(self.declare_parameter('max_orbit_radius_m', 20.0, descriptor).value),
            float(self.declare_parameter('max_orbit_speed_mps', 3.0, descriptor).value),
            float(self.declare_parameter('max_orbit_center_distance_m', 100.0, descriptor).value),
        )
        self.planning_service = self.create_service(
            PlanFlight, 'mission_planner/plan_flight', self.plan_flight
        )

    def plan_flight(
        self, request: PlanFlight.Request, response: PlanFlight.Response
    ) -> PlanFlight.Response:
        """Return fixed geographic instructions or an unsuccessful diagnostic."""
        try:
            state = request.starting_state
            if (
                state.header.frame_id != 'local_enu'
                or not state.position_valid
                or not state.global_position_valid
            ):
                raise ValueError(
                    'planning requires valid local and geographic position in the start snapshot'
                )
            reference: GeographicOrigin | None = None
            if state.global_reference_valid:
                if state.local_frame_revision == 0:
                    raise ValueError('geographic reference has no local-frame revision')
                reference = GeographicOrigin(
                    state.reference_latitude_deg,
                    state.reference_longitude_deg,
                    state.reference_altitude_amsl_m,
                )
            start = StartingPosition(
                state.position.x,
                state.position.y,
                state.position.z,
                reference,
                state.yaw_rad if state.yaw_valid else None,
                GeographicOrigin(
                    state.geographic_position.latitude_deg,
                    state.geographic_position.longitude_deg,
                    state.geographic_position.altitude_amsl_m,
                ),
            )
            instructions = compile_plan(request.plan_json, request.vehicle_id, start, self.limits)
            response.takeoff_altitude_m = instructions.takeoff_altitude_m
            response.position_tolerance_m = instructions.position_tolerance_m
            response.waypoint_positions = [
                GeographicPosition(
                    latitude_deg=position.latitude_deg,
                    longitude_deg=position.longitude_deg,
                    altitude_amsl_m=position.altitude_amsl_m,
                )
                for position in instructions.waypoint_positions
            ]
            if instructions.orbit_center_geographic is not None:
                center: GeographicOrigin = instructions.orbit_center_geographic
                response.orbit_after_waypoints = True
                response.orbit_center_geographic = GeographicPosition(
                    latitude_deg=center.latitude_deg,
                    longitude_deg=center.longitude_deg,
                    altitude_amsl_m=center.altitude_amsl_m,
                )
                response.orbit_radius_m = instructions.orbit_radius_m
                response.orbit_tangential_speed_mps = instructions.orbit_speed_mps
                response.orbit_revolutions = 1.0
                response.orbit_timeout_sec = instructions.orbit_timeout_sec
            response.success = True
            response.message = 'Selected assignment compiled into fixed geographic destinations'
        except (ValueError, OverflowError, RecursionError) as error:
            response.success = False
            response.message = str(error)
        return response


def main() -> None:
    """Run the planning-only service in its ROS namespace."""
    rclpy.init()
    node: MissionPlanner | None = None
    try:
        node = MissionPlanner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
