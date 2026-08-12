#!/usr/bin/env python3
"""Run the standalone UGV waypoint mission."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from baseline.mission.world_poses import world_target_in_enu_odom


def log(path: Path, event: str, **details) -> None:
    record = {"timestamp": datetime.now().astimezone().isoformat(),
              "component": "mission", "event": event, **details}
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, separators=(",", ":")) + "\n")


def pose_message(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = "odom"
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def wait_for_navigation_server(navigator: BasicNavigator, events: Path) -> None:
    """Wait until Nav2 is active and its navigation action is available.

    Under a heavy Gazebo + RTAB-Map load, a lifecycle get-state request can time
    out once and leave BasicNavigator.waitUntilNav2Active() waiting forever. Use
    short, repeatable requests so one delayed response cannot wedge the mission.
    """
    attempts = 0
    state_client = navigator.create_client(GetState, "/bt_navigator/get_state")
    request = GetState.Request()
    while True:
        attempts += 1
        if state_client.wait_for_service(timeout_sec=1.0):
            future = state_client.call_async(request)
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(navigator, timeout_sec=0.1)
            if future.done() and future.result() is not None:
                if future.result().current_state.label == "active":
                    break
            else:
                future.cancel()
        if attempts == 1 or attempts % 5 == 0:
            log(events, "nav2_lifecycle_wait", attempts=attempts)
        time.sleep(0.2)

    attempts = 0
    while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        attempts += 1
        if attempts == 1 or attempts % 10 == 0:
            log(events, "nav2_action_wait", seconds=attempts)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--targets", nargs="+", required=True)
    args = parser.parse_args()
    targets = [(name, world_target_in_enu_odom(args.world, name)) for name in args.targets]

    rclpy.init()
    navigator = BasicNavigator()
    try:
        log(args.events, "waiting_for_nav2", targets=args.targets)
        wait_for_navigation_server(navigator, args.events)
        log(args.events, "nav2_action_ready")
        for name, target in targets:
            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                accepted = navigator.goToPose(
                    pose_message(navigator, target.x, target.y, target.yaw)
                )
                if accepted is False:
                    log(args.events, "goal_rejected", target=name, attempt=attempt)
                    result = TaskResult.FAILED
                else:
                    log(
                        args.events,
                        "goal_sent",
                        target=name,
                        attempt=attempt,
                        frame="odom",
                        pose={"x": target.x, "y": target.y, "yaw": target.yaw},
                    )
                    while not navigator.isTaskComplete():
                        time.sleep(0.2)
                    result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    break
                if attempt == max_attempts:
                    log(
                        args.events,
                        "navigation_failed",
                        target=name,
                        attempts=attempt,
                        result=str(result),
                    )
                    return 3
                log(
                    args.events,
                    "navigation_retry",
                    target=name,
                    failed_attempt=attempt,
                    result=str(result),
                )
                # Let the rolling costmap receive another segmented LiDAR
                # update before requesting a fresh global path.
                time.sleep(2.0)
            log(args.events, "waypoint_reached", target=name)
        log(args.events, "mission_completed", targets=args.targets)
        return 0
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
