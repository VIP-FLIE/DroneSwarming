# Copyright 2026 DroneSwarming maintainers.
"""Inspect the actual namespaced launch without enabling flight commands."""

import os
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.action.graph import (
    get_action_client_names_and_types_by_node,
    get_action_server_names_and_types_by_node,
)
from rclpy.node import Node


class CommandEndpoints(unittest.TestCase):
    """Verify public ownership, internal clients, and unique launch endpoints."""

    def test_namespaced_command_boundary(self) -> None:
        """Launch all three nodes and compare their actual ROS action graph."""
        namespace = '/command_boundary'
        operations: dict[str, str] = {
            'takeoff': 'Takeoff',
            'navigate_to': 'NavigateTo',
            'follow_trajectory': 'FollowTrajectory',
            'orbit_point': 'OrbitPoint',
            'land': 'Land',
        }
        expected_internal: dict[str, list[str]] = {
            f'{namespace}/vehicle_control/{name}': [f'swarm_interfaces/action/{action}']
            for name, action in operations.items()
        }
        expected_public: dict[str, list[str]] = {
            f'{namespace}/mission_execution/{name}': [f'swarm_interfaces/action/{action}']
            for name, action in operations.items()
        }
        expected_public[f'{namespace}/mission_execution/execute_flight_plan'] = [
            'swarm_interfaces/action/ExecuteFlightPlan'
        ]
        expected_public[f'{namespace}/execute_waypoint_mission'] = [
            'swarm_interfaces/action/ExecuteWaypointMission'
        ]
        with tempfile.TemporaryFile(mode='w+') as launch_log:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'drone_bringup',
                    'single_vehicle.launch.py',
                    f'namespace:={namespace}',
                    'px4_topic_prefix:=/command_boundary/fmu',
                ],
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init()
            node = Node('command_boundary_inspector')
            try:
                deadline: float = time.monotonic() + 15.0
                public: dict[str, list[str]] = {}
                internal: dict[str, list[str]] = {}
                clients: dict[str, list[str]] = {}
                while time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), 'Application launch exited')
                    rclpy.spin_once(node, timeout_sec=0.05)
                    graph: list[tuple[str, str]] = node.get_node_names_and_namespaces()
                    if any(
                        (name, namespace) not in graph
                        for name in ('vehicle_control', 'mission_execution', 'mission_planner')
                    ):
                        continue
                    public = dict(
                        get_action_server_names_and_types_by_node(
                            node, 'mission_execution', namespace
                        )
                    )
                    internal = dict(
                        get_action_server_names_and_types_by_node(
                            node, 'vehicle_control', namespace
                        )
                    )
                    clients = dict(
                        get_action_client_names_and_types_by_node(
                            node, 'mission_execution', namespace
                        )
                    )
                    if (
                        public == expected_public
                        and internal == expected_internal
                        and clients == expected_internal
                    ):
                        break
                self.assertEqual(public, expected_public)
                self.assertEqual(internal, expected_internal)
                self.assertEqual(clients, expected_internal)
                self.assertEqual(
                    get_action_client_names_and_types_by_node(node, 'vehicle_control', namespace),
                    [],
                )
                graph = node.get_node_names_and_namespaces()
                self.assertEqual(graph.count(('vehicle_control', namespace)), 1)
                self.assertEqual(graph.count(('mission_execution', namespace)), 1)
                self.assertEqual(graph.count(('mission_planner', namespace)), 1)
                self.assertEqual(
                    get_action_client_names_and_types_by_node(node, 'mission_planner', namespace),
                    [],
                )
            finally:
                node.destroy_node()
                rclpy.shutdown()
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.0)
                launch_log.seek(0)
                print(launch_log.read())


if __name__ == '__main__':
    unittest.main()
