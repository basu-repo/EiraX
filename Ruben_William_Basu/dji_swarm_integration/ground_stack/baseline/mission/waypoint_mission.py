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
import yaml
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

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


def wait_for_navigation_server(
    navigator: BasicNavigator,
    events: Path,
    *,
    reason: str = "startup",
) -> None:
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
            log(events, "nav2_lifecycle_wait", attempts=attempts, reason=reason)
        time.sleep(0.2)

    attempts = 0
    while not navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        attempts += 1
        if attempts == 1 or attempts % 10 == 0:
            log(events, "nav2_action_wait", seconds=attempts, reason=reason)


def recover_navigation(
    navigator: BasicNavigator,
    events: Path,
    *,
    target: str,
    failed_attempt: int,
    reason: str,
    reverse_config: dict,
    allow_reverse: bool,
    lidar_received_monotonic: list[float | None],
) -> None:
    """Wait for a fully active stack and rebuild costmaps before retrying."""
    print(
        f"[NAV2 RECOVERY] {target} attempt {failed_attempt} failed ({reason}); "
        "waiting for the previous goal and lifecycle recovery.",
        flush=True,
    )
    log(
        events,
        "nav2_recovery_started",
        target=target,
        failed_attempt=failed_attempt,
        reason=reason,
    )
    # isTaskComplete() has already observed the terminal result. Waiting for
    # lifecycle state + action availability prevents inactive-server rejects
    # from consuming the next real navigation attempt.
    wait_for_navigation_server(navigator, events, reason="mission_retry")
    if allow_reverse and reverse_config["enabled"]:
        freshness = float(reverse_config["lidar_freshness_sec"])
        deadline = time.monotonic() + float(
            reverse_config["lidar_wait_timeout_sec"]
        )
        while time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            received = lidar_received_monotonic[0]
            if received is not None and time.monotonic() - received <= freshness:
                break
        else:
            log(events, "reverse_recovery_skipped", target=target, reason="stale_lidar")
            print(
                "[REVERSE RECOVERY] Skipped: no fresh rear-capable LiDAR; "
                "clearing costmaps and retrying forward.",
                flush=True,
            )
        if (
            lidar_received_monotonic[0] is not None
            and time.monotonic() - lidar_received_monotonic[0] <= freshness
        ):
            distance = float(reverse_config["distance_m"])
            speed = float(reverse_config["speed_mps"])
            allowance = int(reverse_config["time_allowance_sec"])
            print(
                f"[REVERSE RECOVERY] Fresh LiDAR verified; requesting at most "
                f"{distance:.1f} m at {speed:.2f} m/s.",
                flush=True,
            )
            log(
                events,
                "reverse_recovery_started",
                target=target,
                distance_m=distance,
                speed_mps=speed,
                time_allowance_sec=allowance,
            )
            accepted = navigator.backup(
                backup_dist=distance,
                backup_speed=speed,
                time_allowance=allowance,
            )
            if accepted is not False:
                while not navigator.isTaskComplete():
                    time.sleep(0.1)
                backup_result = navigator.getResult()
            else:
                backup_result = TaskResult.FAILED
            log(
                events,
                "reverse_recovery_finished",
                target=target,
                result=str(backup_result),
            )
            print(
                f"[REVERSE RECOVERY] Finished with {backup_result}; "
                "returning to forward-only planning.",
                flush=True,
            )
    print(
        "[NAV2 RECOVERY] Controller, planner and navigator are active; "
        "clearing local and global costmaps.",
        flush=True,
    )
    navigator.clearAllCostmaps()
    log(events, "nav2_costmaps_cleared", target=target)
    # Allow segmented LiDAR and aerial observations to repopulate the cleared
    # maps before requesting a new global path.
    time.sleep(3.0)
    wait_for_navigation_server(navigator, events, reason="post_costmap_clear")
    log(events, "nav2_recovery_ready", target=target)
    print(
        f"[NAV2 RECOVERY] Nav2 and fresh costmaps are ready; retrying {target}.",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--targets", nargs="+", required=True)
    parser.add_argument("--reverse-recovery-config", type=Path)
    args = parser.parse_args()
    targets = [(name, world_target_in_enu_odom(args.world, name)) for name in args.targets]

    rclpy.init()
    navigator = BasicNavigator()
    lidar_received_monotonic: list[float | None] = [None]

    def lidar_received(_message: PointCloud2) -> None:
        lidar_received_monotonic[0] = time.monotonic()

    navigator.create_subscription(
        PointCloud2,
        "/husky/lidar3d/points",
        lidar_received,
        qos_profile_sensor_data,
    )
    reverse_config = {
        "enabled": False,
        "distance_m": 1.5,
        "speed_mps": 0.15,
        "time_allowance_sec": 15,
        "lidar_freshness_sec": 1.0,
        "lidar_wait_timeout_sec": 10.0,
    }
    if args.reverse_recovery_config is not None:
        loaded = yaml.safe_load(
            args.reverse_recovery_config.read_text(encoding="utf-8")
        )
        reverse_config.update(loaded.get("reverse_recovery", {}))
    try:
        log(args.events, "waiting_for_nav2", targets=args.targets)
        wait_for_navigation_server(navigator, args.events)
        log(args.events, "nav2_action_ready")
        for name, target in targets:
            max_attempts = 3
            attempt = 1
            rejected_while_inactive = 0
            while attempt <= max_attempts:
                accepted = navigator.goToPose(
                    pose_message(navigator, target.x, target.y, target.yaw)
                )
                if accepted is False:
                    rejected_while_inactive += 1
                    log(
                        args.events,
                        "goal_rejected_while_unavailable",
                        target=name,
                        intended_attempt=attempt,
                        readiness_rejections=rejected_while_inactive,
                    )
                    print(
                        f"[NAV2 RECOVERY] {name} goal server was unavailable; "
                        f"attempt {attempt} has not been consumed.",
                        flush=True,
                    )
                    wait_for_navigation_server(
                        navigator, args.events, reason="goal_rejected"
                    )
                    time.sleep(1.0)
                    continue
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
                recover_navigation(
                    navigator,
                    args.events,
                    target=name,
                    failed_attempt=attempt,
                    reason=str(result),
                    reverse_config=reverse_config,
                    allow_reverse=attempt == 1,
                    lidar_received_monotonic=lidar_received_monotonic,
                )
                attempt += 1
            log(args.events, "waypoint_reached", target=name)
        log(args.events, "mission_completed", targets=args.targets)
        return 0
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
