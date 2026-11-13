# Portions Copyright (c) 2026, vijaymanbajracharya. See LICENSE.sar-drone.
"""Read a JSON file and submit its content to the selected mission executor."""

import argparse
import signal
from pathlib import Path
from types import FrameType
from typing import cast

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from swarm_interfaces.action import ExecuteFlightPlan

from .flight_plan import MAXIMUM_PLAN_JSON_LENGTH

SERVER_WAIT_SEC: float = 5.0
CALLBACK_POLL_SEC: float = 0.1


def submit_plan(plan_json: str, namespace: str) -> int:
    """Forward JSON unchanged and wait for the mission's success, failure, or cancellation."""
    cancel_requested = False

    def request_cancel(signal_number: int, frame: FrameType | None) -> None:
        """Defer SIGINT into an action cancellation that can drain its active child."""
        nonlocal cancel_requested
        cancel_requested = True

    previous_handler = signal.signal(signal.SIGINT, request_cancel)
    node: Node | None = None
    client: ActionClient | None = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node('flight_plan_launcher', namespace=namespace)
        client = ActionClient(node, ExecuteFlightPlan, 'mission_execution/execute_flight_plan')
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            raise RuntimeError('mission execution is unavailable')
        if cancel_requested:
            raise RuntimeError('flight plan canceled before submission')
        goal = ExecuteFlightPlan.Goal()
        goal.plan_json = plan_json

        def show_feedback(message: ExecuteFlightPlan.Impl.FeedbackMessage) -> None:
            """Display execution progress from the command-owning node."""
            feedback: ExecuteFlightPlan.Feedback = message.feedback
            print(
                f'{feedback.status}: completed waypoints {feedback.completed_waypoints}', flush=True
            )

        goal_future: Future = client.send_goal_async(goal, feedback_callback=show_feedback)
        while rclpy.ok() and not goal_future.done():
            rclpy.spin_once(node, timeout_sec=CALLBACK_POLL_SEC)
            if not client.server_is_ready():
                raise RuntimeError('mission executor disappeared; goal acceptance is unknown')
        # rclpy futures expose a dynamically typed ROS boundary.
        handle: ClientGoalHandle | None = cast(ClientGoalHandle | None, goal_future.result())
        if handle is None or not handle.accepted:
            raise RuntimeError('mission execution rejected the flight plan')
        result_future: Future = handle.get_result_async()
        cancel_sent = False
        while rclpy.ok() and not result_future.done():
            if cancel_requested and not cancel_sent:
                handle.cancel_goal_async()
                cancel_sent = True
            rclpy.spin_once(node, timeout_sec=CALLBACK_POLL_SEC)
            if not client.server_is_ready():
                raise RuntimeError('mission executor disappeared; flight outcome is unknown')
        response: ExecuteFlightPlan.Impl.GetResultService.Response | None = result_future.result()
        if response is None:
            raise RuntimeError('ROS stopped without a terminal mission result')
        print(response.result.message)
        return (
            0 if response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success else 1
        )
    except (RuntimeError, ValueError) as error:
        print(f'Flight plan submission failed: {error}')
        return 1
    finally:
        if client is not None:
            client.destroy()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_handler)


def main() -> int:
    """Load a bounded UTF-8 file and address the selected application namespace."""
    parser = argparse.ArgumentParser(
        description='Submit JSON flight intent through mission execution.'
    )
    parser.add_argument('plan_file', type=Path)
    parser.add_argument(
        '--namespace', default='/', help='application namespace, for example /uav_1'
    )
    arguments: argparse.Namespace = parser.parse_args()
    try:
        with arguments.plan_file.open('rb') as plan_file:
            encoded: bytes = plan_file.read(MAXIMUM_PLAN_JSON_LENGTH + 1)
        if len(encoded) > MAXIMUM_PLAN_JSON_LENGTH:
            raise ValueError('flight plan JSON exceeds 65536 bytes')
        plan_json: str = encoded.decode('utf-8')
    except (OSError, ValueError) as error:
        parser.error(f'could not read flight plan: {error}')
    return submit_plan(plan_json, arguments.namespace)
