# Copyright 2026 DroneSwarming maintainers.
"""Exercise real planners and executors with namespaced simulated action backends."""

import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from swarm_interfaces.action import ExecuteFlightPlan, Land, NavigateTo, Takeoff
from swarm_interfaces.msg import VehicleState
from swarm_interfaces.srv import PlanFlight
from geometry_msgs.msg import Point

from mission_planner.geography import GeographicOrigin, enu_to_geographic
from mission_planner.node import MissionPlanner
from mission_planner.flight_plan import JsonValue


class VehicleBackend(Node):
    """Record child operations and publish independent local frame references."""

    def __init__(self, namespace: str, reference: GeographicOrigin) -> None:
        super().__init__('vehicle_backend', namespace=namespace)
        self.state = VehicleState()
        self.state.header.frame_id = 'local_enu'
        self.state.position_valid = True
        self.state.yaw_valid = True
        self.state.landed = True
        self.state.global_reference_valid = True
        self.state.local_frame_revision = 1
        self.state.reference_latitude_deg = reference.latitude_deg
        self.state.reference_longitude_deg = reference.longitude_deg
        self.state.reference_altitude_amsl_m = reference.altitude_amsl_m
        self.navigation_targets: list[tuple[float, float, float]] = []
        self.should_hold_navigation = False
        self.navigation_started = threading.Event()
        self.navigation_cancellations = 0
        self.takeoffs = 0
        self.landings = 0
        self.lock = threading.Lock()
        self.publisher = self.create_publisher(VehicleState, 'vehicle_state', 10)
        self.timer = self.create_timer(0.02, self.publish_state)
        group = ReentrantCallbackGroup()
        self.servers: list[ActionServer] = [
            ActionServer(
                self, Takeoff, 'vehicle_control/takeoff', self.takeoff, callback_group=group
            ),
            ActionServer(
                self,
                NavigateTo,
                'vehicle_control/navigate_to',
                self.navigate,
                cancel_callback=lambda handle: CancelResponse.ACCEPT,
                callback_group=group,
            ),
            ActionServer(self, Land, 'vehicle_control/land', self.land, callback_group=group),
        ]

    def publish_state(self) -> None:
        with self.lock:
            self.state.header.stamp = self.get_clock().now().to_msg()
            self.state.setpoint_publication_count += 1
            self.publisher.publish(self.state)

    def takeoff(self, handle: ServerGoalHandle) -> Takeoff.Result:
        with self.lock:
            self.takeoffs += 1
            self.state.armed = True
            self.state.landed = False
            self.state.offboard = True
            self.state.position.z += handle.request.altitude_m
        time.sleep(0.06)
        handle.succeed()
        return Takeoff.Result(success=True, message='Simulated takeoff complete')

    def navigate(self, handle: ServerGoalHandle) -> NavigateTo.Result:
        with self.lock:
            target = handle.request.target_position
            self.navigation_targets.append((target.x, target.y, target.z))
            self.navigation_started.set()
        while self.should_hold_navigation and rclpy.ok():
            if handle.is_cancel_requested:
                with self.lock:
                    self.navigation_cancellations += 1
                handle.canceled()
                return NavigateTo.Result(success=False, message='Simulated hold established')
            time.sleep(0.01)
        with self.lock:
            self.state.position = handle.request.target_position
        time.sleep(0.06)
        handle.succeed()
        return NavigateTo.Result(success=True, final_distance_m=0.0, final_yaw_error_rad=math.nan)

    def land(self, handle: ServerGoalHandle) -> Land.Result:
        with self.lock:
            self.landings += 1
            self.state.armed = False
            self.state.offboard = False
            self.state.landed = True
            self.state.position.z = 0.0
        handle.succeed()
        return Land.Result(success=True, landed=True, message='Simulated landing complete')


class PlannerExecutionFlow(unittest.TestCase):
    """Check the JSON ingress, pure planner handoff, and per-vehicle execution boundary."""

    def setUp(self) -> None:
        rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.peer = Node('planner_flow_client')
        self.executor.add_node(self.peer)
        reference = GeographicOrigin(47.0, 8.0, 500.0)
        references: list[GeographicOrigin] = [reference, enu_to_geographic(30, 0, 0, reference)]
        self.backends: list[VehicleBackend] = []
        self.planners: list[MissionPlanner] = []
        self.clients: list[ActionClient] = []
        self.processes: list[subprocess.Popen[bytes]] = []
        self.logs = tempfile.TemporaryFile(mode='w+')
        executable = (
            Path(get_package_prefix('mission_execution'))
            / 'lib/mission_execution/mission_execution_node'
        )
        for index, origin in enumerate(references, start=1):
            namespace = f'/planner_flow/uav_{index}'
            backend = VehicleBackend(namespace, origin)
            planner = MissionPlanner(namespace=namespace)
            self.backends.append(backend)
            self.planners.append(planner)
            self.executor.add_node(backend)
            self.executor.add_node(planner)
            self.processes.append(
                subprocess.Popen(
                    [
                        str(executable),
                        '--ros-args',
                        '-r',
                        f'__ns:={namespace}',
                        '-p',
                        f'vehicle_id:=uav_{index}',
                        '-p',
                        'action_server_timeout_sec:=0.5',
                    ],
                    stdout=self.logs,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
            self.clients.append(
                ActionClient(
                    self.peer,
                    ExecuteFlightPlan,
                    f'{namespace}/mission_execution/execute_flight_plan',
                )
            )
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()
        for client in self.clients:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
        time.sleep(0.1)

    def tearDown(self) -> None:
        for backend in self.backends:
            backend.should_hold_navigation = False
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.0)
        self.executor.shutdown(timeout_sec=5.0)
        self.spin_thread.join(timeout=5.0)
        for client in self.clients:
            client.destroy()
        for backend in self.backends:
            for server in backend.servers:
                server.destroy()
            backend.destroy_node()
        for planner in self.planners:
            planner.destroy_node()
        self.peer.destroy_node()
        rclpy.shutdown()
        self.logs.seek(0)
        print(self.logs.read())
        self.logs.close()

    def wait_future(self, future: Future) -> None:
        deadline: float = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done(), 'ROS operation did not complete')

    def start_plan(self, encoded: str, index: int = 0) -> ClientGoalHandle:
        response: Future = self.clients[index].send_goal_async(
            ExecuteFlightPlan.Goal(plan_json=encoded)
        )
        self.wait_future(response)
        handle = response.result()
        self.assertTrue(handle.accepted)
        return handle

    def test_two_drones_select_different_relative_assignments(self) -> None:
        missions: list[dict[str, JsonValue]] = [
            {
                'vehicle_ids': [f'uav_{index}'],
                'frame_id': 'relative_enu',
                'takeoff_altitude_m': 3,
                'waypoint_acceptance_radius_m': 0.5,
                'path': {'type': 'waypoints', 'points': [[east_m, 0], [east_m, 2]]},
            }
            for index, east_m in ((1, 3), (2, -4))
        ]
        encoded: str = json.dumps({'version': 2, 'missions': missions})
        futures: list[Future] = [
            self.start_plan(encoded, index).get_result_async() for index in range(2)
        ]
        for future in futures:
            self.wait_future(future)
            self.assertEqual(future.result().status, GoalStatus.STATUS_SUCCEEDED)
            self.assertEqual(future.result().result.completed_waypoints, 2)
        self.assertEqual(self.backends[0].navigation_targets, [(3, 0, 3), (3, 2, 3)])
        self.assertEqual(self.backends[1].navigation_targets, [(-4, 0, 3), (-4, 2, 3)])
        self.assertEqual([backend.landings for backend in self.backends], [1, 1])

    def test_two_local_frames_resolve_one_absolute_gps_destination(self) -> None:
        origin = GeographicOrigin(47, 8, 500)
        destination = enu_to_geographic(10, 5, 3, origin)
        encoded: str = json.dumps(
            {
                'version': 2,
                'missions': [
                    {
                        'vehicle_ids': ['uav_1', 'uav_2'],
                        'frame_id': 'wgs84',
                        'takeoff_altitude_m': 3,
                        'waypoint_acceptance_radius_m': 0.5,
                        'altitude_amsl_m': 503,
                        'path': {
                            'type': 'waypoints',
                            'points': [[destination.latitude_deg, destination.longitude_deg]],
                        },
                    }
                ],
            }
        )
        for index in range(2):
            future = self.start_plan(encoded, index).get_result_async()
            self.wait_future(future)
            self.assertEqual(future.result().status, GoalStatus.STATUS_SUCCEEDED)
        self.assertAlmostEqual(self.backends[0].navigation_targets[0][0], 10.0, places=6)
        self.assertAlmostEqual(self.backends[1].navigation_targets[0][0], -20.0, places=4)
        self.assertAlmostEqual(self.backends[0].navigation_targets[0][1], 5.0, places=6)

    def test_invalid_json_or_assignment_never_reaches_vehicle_control(self) -> None:
        for encoded in ('{"version":2,"version":2}', '{"version":2,"missions":[]}'):
            result = self.start_plan(encoded).get_result_async()
            self.wait_future(result)
            self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(self.backends[0].takeoffs, 0)
        self.assertEqual(self.backends[0].navigation_targets, [])

    def relative_plan(self) -> str:
        return json.dumps(
            {
                'version': 2,
                'missions': [
                    {
                        'vehicle_ids': ['uav_1'],
                        'frame_id': 'relative_enu',
                        'takeoff_altitude_m': 3,
                        'waypoint_acceptance_radius_m': 0.5,
                        'path': {'type': 'waypoints', 'points': [[3, 0], [3, 2]]},
                    }
                ],
            }
        )

    def test_parent_cancellation_drains_active_navigation(self) -> None:
        backend = self.backends[0]
        backend.should_hold_navigation = True
        handle = self.start_plan(self.relative_plan())
        self.assertTrue(backend.navigation_started.wait(timeout=5.0))
        cancellation = handle.cancel_goal_async()
        self.wait_future(cancellation)
        result = handle.get_result_async()
        self.wait_future(result)
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(backend.navigation_cancellations, 1)
        self.assertEqual(len(backend.navigation_targets), 1)
        self.assertEqual(backend.landings, 0)

    def test_coordinate_frame_reset_cancels_and_aborts_the_plan(self) -> None:
        backend = self.backends[0]
        backend.should_hold_navigation = True
        handle = self.start_plan(self.relative_plan())
        self.assertTrue(backend.navigation_started.wait(timeout=5.0))
        with backend.lock:
            backend.state.local_frame_revision += 1
        result = handle.get_result_async()
        self.wait_future(result)
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertIn('coordinate frame changed', result.result().result.message)
        self.assertEqual(backend.navigation_cancellations, 1)
        self.assertEqual(len(backend.navigation_targets), 1)

    def test_explicit_landing_preempts_the_json_route(self) -> None:
        backend = self.backends[0]
        backend.should_hold_navigation = True
        handle = self.start_plan(self.relative_plan())
        self.assertTrue(backend.navigation_started.wait(timeout=5.0))
        client = ActionClient(self.peer, Land, '/planner_flow/uav_1/mission_execution/land')
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            landing = client.send_goal_async(Land.Goal())
            self.wait_future(landing)
            result = landing.result().get_result_async()
            self.wait_future(result)
            self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)
            parent_result = handle.get_result_async()
            self.wait_future(parent_result)
            self.assertEqual(parent_result.result().status, GoalStatus.STATUS_ABORTED)
            self.assertEqual(backend.navigation_cancellations, 1)
            self.assertEqual(backend.landings, 1)
            self.assertEqual(len(backend.navigation_targets), 1)
        finally:
            client.destroy()

    def test_unavailable_planner_cannot_start_takeoff(self) -> None:
        self.planners[0].destroy_service(self.planners[0].planning_service)
        result = self.start_plan(self.relative_plan()).get_result_async()
        self.wait_future(result)
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertIn('planner', result.result().result.message)
        self.assertEqual(self.backends[0].takeoffs, 0)

    def test_invalid_planner_response_cannot_start_takeoff(self) -> None:

        def invalid_instructions(
            request: PlanFlight.Request, response: PlanFlight.Response
        ) -> PlanFlight.Response:
            response.success = True
            response.takeoff_altitude_m = 3.0
            response.position_tolerance_m = 0.5
            response.waypoint_offsets = [Point(x=10000.0, y=0.0, z=3.0)]
            return response

        self.planners[0].planning_service.callback = invalid_instructions
        result = self.start_plan(self.relative_plan()).get_result_async()
        self.wait_future(result)
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertIn('out-of-bounds', result.result().result.message)
        self.assertEqual(self.backends[0].takeoffs, 0)

    def test_landing_during_planning_ignores_a_late_translation(self) -> None:
        planning_started = threading.Event()

        def delayed_instructions(
            request: PlanFlight.Request, response: PlanFlight.Response
        ) -> PlanFlight.Response:
            planning_started.set()
            time.sleep(1.0)
            response.success = True
            response.takeoff_altitude_m = 3.0
            response.position_tolerance_m = 0.5
            response.waypoint_offsets = [Point(x=3.0, y=0.0, z=3.0)]
            return response

        self.planners[0].planning_service.callback = delayed_instructions
        client = ActionClient(self.peer, Land, '/planner_flow/uav_1/mission_execution/land')
        try:
            handle = self.start_plan(self.relative_plan())
            self.assertTrue(planning_started.wait(timeout=5.0))
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            landing = client.send_goal_async(Land.Goal())
            self.wait_future(landing)
            landing_result = landing.result().get_result_async()
            self.wait_future(landing_result)
            parent_result = handle.get_result_async()
            self.wait_future(parent_result)
            self.assertEqual(parent_result.result().status, GoalStatus.STATUS_ABORTED)
            self.assertEqual(landing_result.result().status, GoalStatus.STATUS_SUCCEEDED)
            time.sleep(1.1)
            self.assertEqual(self.backends[0].takeoffs, 0)
            self.assertEqual(self.backends[0].landings, 1)
        finally:
            client.destroy()


if __name__ == '__main__':
    unittest.main()
