#!/usr/bin/env python3
"""DJI adaptation of the proven UGV_UAV progressive scout-and-escort pattern."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Joy

from robots.dji_m100.missions.failover_protocol import read as read_swarm_state


def progressive_lead_point(
    ugv_x: float, ugv_y: float, target_x: float, target_y: float, lead_m: float
) -> tuple[float, float]:
    dx, dy = target_x - ugv_x, target_y - ugv_y
    remaining = math.hypot(dx, dy)
    if remaining <= 0.05:
        return target_x, target_y
    distance = min(lead_m, remaining)
    return ugv_x + dx * distance / remaining, ugv_y + dy * distance / remaining


class DjiFollower(Node):
    def __init__(
        self,
        name: str,
        ugv_start: tuple[float, float, float],
    ) -> None:
        super().__init__("dji_husky_follower")
        self.name = name
        self.ugv_start = ugv_start
        # Seed the mission with the configured Husky spawn. Live ground truth
        # replaces this as soon as it arrives, but it must never block DJI
        # takeoff. This preserves the working one_dji_check flight contract.
        self.ugv: tuple[float, float, float] | None = ugv_start
        self.uav: tuple[float, float, float] | None = None
        self.camera_ready = False
        self.publisher = self.create_publisher(
            Joy, f"/{name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw", 10
        )
        self.ugv_pose_subscription = self.create_subscription(
            PoseStamped,
            "/dji_swarm/husky_pose",
            self._on_ugv_pose,
            qos_profile_sensor_data,
        )
        # Match the subscription used by the already validated DJI movement
        # test. This is the native Halmstad DJI simulator pose topic.
        self.uav_pose_subscription = self.create_subscription(
            PoseStamped,
            f"/{name}/pose",
            self._on_uav_pose,
            10,
        )
        self.camera_subscription = self.create_subscription(
            Image,
            f"/{name}/camera0/image_raw",
            self._image,
            qos_profile_sensor_data,
        )

    def _on_ugv_pose(self, message: PoseStamped) -> None:
        # Dedicated DJI-side Gazebo bridge: already absolute Baylands ENU.
        p = message.pose.position
        self.ugv = float(p.x), float(p.y), float(p.z)

    def _on_uav_pose(self, message: PoseStamped) -> None:
        p = message.pose.position
        self.uav = float(p.x), float(p.y), float(p.z)

    def _image(self, _message: Image) -> None:
        self.camera_ready = True

    def command(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        message = Joy()
        message.axes = [float(x), float(y), float(z), float(yaw)]
        self.publisher.publish(message)


def wait_for_dji_pose(node: DjiFollower, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    last_report = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.uav is not None:
            return True
        now = time.monotonic()
        if now - last_report >= 5.0:
            print(
                "[DJI INPUT STATUS] "
                f"UGV_pose={'live' if node.ugv is not None else 'seeded'} | "
                f"DJI_pose={'ready' if node.uav is not None else 'waiting'} | "
                f"camera={'ready' if node.camera_ready else 'waiting'}",
                flush=True,
            )
            last_report = now
    return False


def move_dji(
    node: DjiFollower,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    speed: float,
    yaw: float = 0.0,
) -> None:
    """Use the same 20 Hz absolute-ENU interpolation as one_dji_check."""
    distance = math.dist(start, end)
    duration = max(1.0, distance / max(0.1, speed))
    steps = max(1, int(duration * 20.0))
    for index in range(1, steps + 1):
        ratio = index / steps
        pose = tuple(a + (b - a) * ratio for a, b in zip(start, end))
        node.command(*pose, yaw)
        rclpy.spin_once(node, timeout_sec=0.05)


def step_toward(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    maximum_step: float,
) -> tuple[float, float, float]:
    dx, dy, dz = (b - a for a, b in zip(current, target))
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance <= maximum_step or distance <= 1e-6:
        return target
    scale = maximum_step / distance
    return current[0] + dx * scale, current[1] + dy * scale, current[2] + dz * scale


def camera_yaw_to_target(
    aircraft: tuple[float, float, float],
    target: tuple[float, float, float],
) -> float:
    """World yaw that points the fixed camera optical axis at a target."""
    return math.atan2(target[1] - aircraft[1], target[0] - aircraft[0])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="dji0")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--coordination-dir", type=Path, required=True)
    parser.add_argument("--role-state", type=Path)
    parser.add_argument("--lateral-offset", type=float, default=0.0)
    parser.add_argument("--altitude", type=float, default=15.0)
    parser.add_argument("--lead-distance", type=float, default=20.0)
    parser.add_argument("--follow-only", action="store_true")
    parser.add_argument("--follow-distance", type=float, default=15.0)
    parser.add_argument("--survey-settle-sec", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument(
        "--ground-z",
        type=float,
        default=1.5,
        help="Absolute Baylands Z height at which the DJI landing stops.",
    )
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--start-z", type=float, required=True)
    parser.add_argument("--ugv-start-x", type=float, required=True)
    parser.add_argument("--ugv-start-y", type=float, required=True)
    parser.add_argument("--ugv-start-z", type=float, required=True)
    parser.add_argument("--target", action="append", required=True)
    args = parser.parse_args()

    targets: list[tuple[str, float, float]] = []
    for value in args.target:
        name, x, y = value.split(",", maxsplit=2)
        targets.append((name, float(x), float(y)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.coordination_dir.mkdir(parents=True, exist_ok=True)
    args.stop_file.unlink(missing_ok=True)
    args.ready_file.unlink(missing_ok=True)
    for index in range(1, len(targets) + 1):
        (args.coordination_dir / f"survey_{index:02d}_ready").unlink(missing_ok=True)
        (args.coordination_dir / f"leg_{index:02d}_complete").unlink(missing_ok=True)

    rclpy.init()
    node = DjiFollower(
        args.name,
        (args.ugv_start_x, args.ugv_start_y, args.ugv_start_z),
    )
    try:
        print("[DJI WAITING] Click Play; waiting for the DJI pose interface.", flush=True)
        if not wait_for_dji_pose(node, 300.0):
            print("[DJI FAILED] DJI pose interface did not start.", flush=True)
            return 2
        print("[DJI READY] DJI pose and absolute-ENU command interface are active.", flush=True)
        if node.camera_ready:
            print("[DJI CAMERA] Camera stream is active.", flush=True)
        else:
            print(
                "[DJI CAMERA WARNING] Camera is not ready yet; movement will "
                "continue because camera health is independent of flight control.",
                flush=True,
            )
        assert node.uav is not None and node.ugv is not None
        command = (args.start_x, args.start_y, args.start_z)
        takeoff = (args.start_x, args.start_y, node.ugv[2] + args.altitude)
        print(
            f"[DJI TAKEOFF] Holding spawn X/Y and climbing to {takeoff[2]:.1f} m.",
            flush=True,
        )
        # Turn toward the Husky from the actual spawn side while climbing. The
        # bearing is computed from geometry, so no particular spawn yaw or
        # start location is assumed.
        takeoff_yaw = camera_yaw_to_target(command, node.ugv)
        move_dji(
            node, command, takeoff, speed=min(args.speed, 1.5), yaw=takeoff_yaw
        )
        command = takeoff
        if args.follow_only:
            print(
                "[DJI AIRBORNE] Takeoff complete; pure Husky-follow mode is active.",
                flush=True,
            )
            args.ready_file.touch()
        else:
            print("[DJI AIRBORNE] Takeoff complete; scout logic is now active.", flush=True)
        leg = 0
        phase = "follow" if args.follow_only else "survey"
        last_role: str | None = None
        settle_started: float | None = None
        start = time.monotonic()
        last_report = -10.0
        period = 0.05

        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "elapsed_sec", "phase", "leg", "target", "command_x", "command_y",
                "command_z", "uav_x", "uav_y", "uav_z", "ugv_x", "ugv_y",
                "horizontal_error_m",
            ])
            while rclpy.ok() and not args.stop_file.exists():
                cycle = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.0)
                if node.ugv is None or node.uav is None:
                    time.sleep(period)
                    continue
                target_name, target_x, target_y = targets[leg]
                role = "follower" if args.follow_only else "scout"
                active_scout = None if args.follow_only else args.name
                if args.role_state and args.role_state.exists():
                    state = read_swarm_state(args.role_state)
                    role = state.states.get(args.name, "follower")
                    active_scout = state.active_scout
                if role != last_role:
                    print(
                        f"[DJI ROLE] {args.name.upper()} -> {role.upper()} | "
                        f"leader/scout={active_scout.upper() if active_scout else 'UGV NAV2'}",
                        flush=True,
                    )
                    last_role = role

                if role == "returning":
                    # The saved spawn Z is the known-safe ground contact height
                    # for this aircraft. A fixed world Z can be underground on
                    # Baylands' uneven terrain.
                    desired = (args.start_x, args.start_y, max(args.ground_z, args.start_z))
                    command = step_toward(command, desired, args.speed * period)
                    node.command(*command, 0.0)
                    if math.dist(command, desired) <= 0.05:
                        print(f"[DJI LANDED] {args.name.upper()} returned to base.", flush=True)
                        while rclpy.ok() and not args.stop_file.exists():
                            node.command(*desired, 0.0)
                            rclpy.spin_once(node, timeout_sec=0.05)
                        return 0
                    time.sleep(max(0.0, period - (time.monotonic() - cycle)))
                    continue
                if role == "disconnected":
                    node.command(*command, 0.0)
                    time.sleep(max(0.0, period - (time.monotonic() - cycle)))
                    continue

                direction_x, direction_y = target_x - node.ugv[0], target_y - node.ugv[1]
                direction_length = max(0.001, math.hypot(direction_x, direction_y))
                left_x, left_y = -direction_y / direction_length, direction_x / direction_length
                if args.follow_only:
                    lead_x = node.ugv[0] - direction_x / direction_length * args.follow_distance
                    lead_y = node.ugv[1] - direction_y / direction_length * args.follow_distance
                elif role == "scout":
                    lead_x, lead_y = progressive_lead_point(
                        node.ugv[0], node.ugv[1], target_x, target_y, args.lead_distance
                    )
                else:
                    lead_x = node.ugv[0] + left_x * args.lateral_offset
                    lead_y = node.ugv[1] + left_y * args.lateral_offset
                desired = (lead_x, lead_y, node.ugv[2] + args.altitude)
                command = step_toward(command, desired, args.speed * period)
                if args.follow_only:
                    # Recalculate every cycle: the fixed camera points at the
                    # live Husky pose regardless of either vehicle's start.
                    yaw = camera_yaw_to_target(command, node.ugv)
                else:
                    yaw = math.atan2(target_y - node.ugv[1], target_x - node.ugv[0])
                node.command(*command, yaw)
                horizontal_error = math.hypot(node.uav[0] - lead_x, node.uav[1] - lead_y)
                altitude_error = abs(node.uav[2] - desired[2])

                if phase == "survey" and role == "scout":
                    if horizontal_error <= 1.5 and altitude_error <= 1.0:
                        if settle_started is None:
                            settle_started = time.monotonic()
                            print(
                                f"[DJI SURVEY] {target_name} lead established; "
                                f"holding {args.survey_settle_sec:.0f} s.", flush=True
                            )
                        elif time.monotonic() - settle_started >= args.survey_settle_sec:
                            (args.coordination_dir / f"survey_{leg + 1:02d}_ready").touch()
                            if not args.ready_file.exists():
                                args.ready_file.touch()
                            phase = "escort"
                            print(
                                f"[DJI SURVEY READY] {target_name}; UGV may move.",
                                flush=True,
                            )
                    else:
                        settle_started = None
                elif (args.coordination_dir / f"leg_{leg + 1:02d}_complete").exists():
                    leg += 1
                    if leg >= len(targets):
                        phase = "follow"
                        leg = len(targets) - 1
                        print("[DJI FOLLOW] All mission legs completed.", flush=True)
                    else:
                        phase = "survey"
                        settle_started = None
                        print(f"[DJI SURVEY] Starting {targets[leg][0]}.", flush=True)

                elapsed = time.monotonic() - start
                writer.writerow([
                    f"{elapsed:.3f}", phase, leg + 1, target_name,
                    *(f"{value:.3f}" for value in command),
                    *(f"{value:.3f}" for value in node.uav),
                    f"{node.ugv[0]:.3f}", f"{node.ugv[1]:.3f}",
                    f"{horizontal_error:.3f}",
                ])
                stream.flush()
                if elapsed - last_report >= 5.0:
                    separation = math.hypot(
                        node.uav[0] - node.ugv[0], node.uav[1] - node.ugv[1]
                    )
                    print(
                        f"[DJI {phase.upper()}] target={target_name} "
                        f"lead_error={horizontal_error:.1f} m "
                        f"UGV_separation={separation:.1f} m", flush=True
                    )
                    last_report = elapsed
                time.sleep(max(0.0, period - (time.monotonic() - cycle)))

            assert node.ugv is not None and node.uav is not None
            dx, dy = node.uav[0] - node.ugv[0], node.uav[1] - node.ugv[1]
            distance = math.hypot(dx, dy)
            if distance < 0.1:
                dx, dy, distance = 1.0, 0.0, 1.0
            local_landing_z = max(args.ground_z, node.ugv[2] + 0.45)
            landing = (
                node.ugv[0] + 4.0 * dx / distance,
                node.ugv[1] + 4.0 * dy / distance,
                local_landing_z,
            )
            print(
                f"[DJI LANDING] Moving 4 m clear of the UGV and descending "
                f"to terrain-safe z={local_landing_z:.2f} m "
                f"(minimum z={args.ground_z:.2f} m).",
                flush=True,
            )
            deadline = time.monotonic() + 90.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.0)
                command = step_toward(command, landing, 0.05)
                node.command(*command, 0.0)
                if math.dist(command, landing) <= 0.05:
                    print("[DJI LANDED] Landing sequence complete.", flush=True)
                    return 0
                time.sleep(0.05)
            print("[DJI FAILED] Landing timed out.", flush=True)
            return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
