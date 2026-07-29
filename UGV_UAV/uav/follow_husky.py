"""Follow the Husky GNSS position from above with a PX4-controlled UAV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys
import threading
import time

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "px4_runtime/python"))
    from pymavlink import mavutil
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from uav.x500_mission import command_and_wait, send_local_setpoint, wait_for_position


EARTH_RADIUS_M = 6_378_137.0


def progressive_lead_point(
    ugv_north: float,
    ugv_east: float,
    target_north: float,
    target_east: float,
    lead_distance: float,
) -> tuple[float, float]:
    """Return a point no farther than lead_distance ahead of the UGV."""
    delta_north = target_north - ugv_north
    delta_east = target_east - ugv_east
    remaining = math.hypot(delta_north, delta_east)
    if remaining <= 0.05:
        return target_north, target_east
    lead = min(lead_distance, remaining)
    return (
        ugv_north + delta_north * lead / remaining,
        ugv_east + delta_east * lead / remaining,
    )


class HuskyGnss(Node):
    def __init__(self) -> None:
        super().__init__("uav_husky_gnss_follower")
        self.fix: tuple[float, float] | None = None
        self.link_active = True
        self.obstacle_points: list[tuple[float, float]] = []
        self.ground_clearance_m: float | None = None
        self.px4_odom = self.create_publisher(Odometry, "/uav/px4_odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            NavSatFix,
            "/communication/uav/rx/ugv_gps",
            self._fix,
            10,
        )
        self.create_subscription(
            PointCloud2, "/x500/lidar3d/points", self._points, 5
        )
        self.create_subscription(
            Bool, "/communication/link/status", self._link_status, 10
        )
        self.planner = ActionClient(self, ComputePathToPose, "/uav_nav/compute_path_to_pose")

    def _fix(self, message: NavSatFix) -> None:
        if math.isfinite(message.latitude) and math.isfinite(message.longitude):
            self.fix = message.latitude, message.longitude

    def _link_status(self, message: Bool) -> None:
        self.link_active = bool(message.data)

    def _points(self, message: PointCloud2) -> None:
        points: list[tuple[float, float]] = []
        downward_ranges: list[float] = []
        # Retain only returns intersecting the UAV flight corridor. Ground
        # returns are far below this band and cannot trigger a false stop.
        for index, point in enumerate(
            point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
        ):
            if index % 4:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            radius = math.hypot(x, y)
            if radius <= 1.5 and z < -0.1:
                downward_ranges.append(-z)
            if -2.5 <= z <= 2.5 and 1.0 <= radius <= 8.0:
                points.append((x, y))
        self.obstacle_points = points
        self.ground_clearance_m = (
            statistics.median(downward_ranges)
            if downward_ranges
            else None
        )

    def publish_px4_pose(self, north: float, east: float, down: float) -> None:
        """Publish PX4 North-East-Down position as ROS East-North-Up."""
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "uav_px4_odom"
        odom.child_frame_id = "uav_base_link"
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = -down
        odom.pose.pose.orientation.w = 1.0
        self.px4_odom.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "uav_px4_odom"
        transform.child_frame_id = "uav_base_link"
        transform.transform.translation.x = east
        transform.transform.translation.y = north
        transform.transform.translation.z = -down
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def corridor_is_clear(
        self,
        delta_north: float,
        delta_east: float,
        emergency_radius: float = 2.0,
        forward_distance: float = 5.0,
        half_width: float = 1.8,
    ) -> bool:
        distance = math.hypot(delta_north, delta_east)
        if distance < 0.05:
            return True
        # Sensor/ROS cloud is East-North; PX4 target is North-East.
        unit_x = delta_east / distance
        unit_y = delta_north / distance
        for x, y in self.obstacle_points:
            radius = math.hypot(x, y)
            if radius < emergency_radius:
                return False
            forward = x * unit_x + y * unit_y
            lateral = abs(-x * unit_y + y * unit_x)
            if 0.0 < forward < forward_distance and lateral < half_width:
                return False
        return True


def relative_ne(latitude: float, longitude: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    north = math.radians(latitude - origin_lat) * EARTH_RADIUS_M
    east = (
        math.radians(longitude - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin_lat))
    )
    return north, east


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--altitude", type=float, default=15.0)
    parser.add_argument("--port", type=int, default=14540)
    parser.add_argument("--lookahead", type=float, default=3.0)
    parser.add_argument("--lead-distance", type=float, default=20.0)
    parser.add_argument("--max-link-range", type=float, default=50.0)
    parser.add_argument("--link-warning-range", type=float, default=40.0)
    parser.add_argument(
        "--survey-settle-sec",
        type=float,
        default=12.0,
        help="Hold over each target while its final observations enter the UGV costmap",
    )
    parser.add_argument(
        "--survey-target",
        action="append",
        default=[],
        metavar="NAME,NORTH,EAST",
        help="Scout this PX4-local target before the matching UGV leg starts",
    )
    parser.add_argument(
        "--coordination-dir",
        type=Path,
        help="Directory containing per-leg survey_ready and leg_complete files",
    )
    args = parser.parse_args()
    if not (
        0.0 < args.lead_distance < args.link_warning_range
        < args.max_link_range
    ):
        parser.error(
            "require 0 < lead distance < warning range < maximum link range"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.stop_file.unlink(missing_ok=True)
    args.ready_file.unlink(missing_ok=True)
    survey_targets: list[tuple[str, float, float]] = []
    for item in args.survey_target:
        name, north, east = item.split(",", maxsplit=2)
        survey_targets.append((name, float(north), float(east)))
    if survey_targets and args.coordination_dir is None:
        parser.error("--coordination-dir is required with --survey-target")
    if args.coordination_dir is not None:
        args.coordination_dir.mkdir(parents=True, exist_ok=True)
        for index in range(len(survey_targets)):
            (args.coordination_dir / f"survey_{index + 1:02d}_ready").unlink(
                missing_ok=True
            )
            (args.coordination_dir / f"leg_{index + 1:02d}_complete").unlink(
                missing_ok=True
            )

    rclpy.init()
    node = HuskyGnss()
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{args.port}", source_system=251)
    if connection.wait_heartbeat(timeout=30) is None:
        raise TimeoutError("No cooperative UAV PX4 heartbeat")
    print("[UAV] Connected to PX4")

    heartbeat_stop = threading.Event()
    setpoint_stop = threading.Event()
    active_target = [0.0, 0.0, -args.altitude]
    cruise_down = -args.altitude

    def heartbeats() -> None:
        while not heartbeat_stop.is_set():
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            heartbeat_stop.wait(0.8)

    def setpoints() -> None:
        while not setpoint_stop.is_set():
            send_local_setpoint(connection, *active_target)
            setpoint_stop.wait(0.1)

    heartbeat_thread = threading.Thread(target=heartbeats, daemon=True)
    setpoint_thread = threading.Thread(target=setpoints, daemon=True)
    heartbeat_thread.start()
    try:
        uav_lat, uav_lon, _ = wait_for_position(connection)
        fix_deadline = time.monotonic() + 60.0
        while node.fix is None and time.monotonic() < fix_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.fix is None:
            raise TimeoutError("No Husky GNSS fix received")

        setpoint_thread.start()
        time.sleep(2.0)
        arm_deadline = time.monotonic() + 45.0
        while True:
            try:
                command_and_wait(
                    connection,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                    timeout=5.0,
                )
                break
            except RuntimeError:
                if time.monotonic() >= arm_deadline:
                    raise RuntimeError("Cooperative UAV preflight did not become ready")
                time.sleep(2.0)
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6 << 16,
        )
        mode_deadline = time.monotonic() + 10.0
        while time.monotonic() < mode_deadline:
            heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat and ((int(heartbeat.custom_mode) >> 16) & 0xFF) == 6:
                break
        else:
            raise TimeoutError("Cooperative UAV did not enter offboard mode")
        if survey_targets:
            print(
                f"[UAV] Armed; taking off to scout {len(survey_targets)} "
                f"UGV mission legs at {args.altitude:.1f} m"
            )
        else:
            print(f"[UAV] Armed; following Husky from {args.altitude:.1f} m above ground")

        start = time.monotonic()
        last_report = -10.0
        last_plan = -10.0
        ready = False
        latest_local = None
        leg_index = 0
        phase = "survey" if survey_targets else "follow"
        survey_arrival_started: float | None = None

        def planned_step(goal_north: float, goal_east: float, current_north: float, current_east: float):
            """Return a short collision-free Nav2 look-ahead point, or None."""
            if not node.planner.wait_for_server(timeout_sec=0.2):
                return None
            # Nav2 now uses the stable PX4 local frame. RTAB-Map's independent
            # LiDAR odometry may reset without moving the flight-control frame.
            current_x = current_east
            current_y = current_north
            delta_north = goal_north - current_north
            delta_east = goal_east - current_east
            goal_distance = math.hypot(delta_north, delta_east)
            if goal_distance > 100.0:
                scale = 100.0 / goal_distance
                delta_north *= scale
                delta_east *= scale
            # ROS / Gazebo are East-North-Up (X=East, Y=North), while PX4
            # local position is North-East-Down (X=North, Y=East).
            goal_x = current_x + delta_east
            goal_y = current_y + delta_north
            request = ComputePathToPose.Goal()
            request.goal = PoseStamped()
            request.goal.header.frame_id = "uav_px4_odom"
            request.goal.header.stamp = node.get_clock().now().to_msg()
            request.goal.pose.position.x = goal_x
            request.goal.pose.position.y = goal_y
            request.goal.pose.orientation.w = 1.0
            request.planner_id = "GridBased"
            request.use_start = False
            sent = node.planner.send_goal_async(request)
            rclpy.spin_until_future_complete(node, sent, timeout_sec=2.0)
            handle = sent.result() if sent.done() else None
            if handle is None or not handle.accepted:
                return None
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=3.0)
            wrapped = result_future.result() if result_future.done() else None
            if wrapped is None or not wrapped.result.path.poses:
                return None
            poses = wrapped.result.path.poses
            for pose in poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                if math.hypot(x - current_x, y - current_y) >= args.lookahead:
                    return current_north + (y - current_y), current_east + (x - current_x)
            final = poses[-1].pose.position
            return (
                current_north + (final.y - current_y),
                current_east + (final.x - current_x),
            )

        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_sec", "phase", "target_north_m", "target_east_m",
                    "uav_north_m", "uav_east_m", "uav_down_m", "follow_error_m",
                    "husky_latitude_deg", "husky_longitude_deg",
                    "husky_north_m", "husky_east_m", "overhead_error_m",
                ]
            )
            while not args.stop_file.exists():
                rclpy.spin_once(node, timeout_sec=0.01)
                if node.fix is not None:
                    north, east = relative_ne(node.fix[0], node.fix[1], uav_lat, uav_lon)
                message = connection.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.1)
                if message is None:
                    continue
                latest_local = message
                node.publish_px4_pose(message.x, message.y, message.z)
                elapsed = time.monotonic() - start
                if survey_targets and phase == "escort":
                    complete_file = (
                        args.coordination_dir
                        / f"leg_{leg_index + 1:02d}_complete"
                    )
                    if complete_file.exists():
                        leg_index += 1
                        if leg_index < len(survey_targets):
                            phase = "survey"
                            survey_arrival_started = None
                            print(
                                f"[UAV SURVEY] Starting leg {leg_index + 1}: "
                                f"{survey_targets[leg_index][0]}"
                            )
                        else:
                            phase = "follow"
                            print("[UAV SURVEY] All requested mission legs surveyed")

                if survey_targets and phase in ("survey", "escort"):
                    _, target_north, target_east = survey_targets[leg_index]
                    command_north, command_east = progressive_lead_point(
                        north,
                        east,
                        target_north,
                        target_east,
                        args.lead_distance,
                    )
                else:
                    command_north, command_east = north, east

                separation = math.hypot(message.x - north, message.y - east)
                if not node.link_active or separation >= args.link_warning_range:
                    # Use the last received UGV position as the recovery point.
                    # This command is local and remains available even after
                    # the communication channel stops relaying vehicle data.
                    command_north, command_east = north, east

                if elapsed - last_plan >= 1.0:
                    step = planned_step(
                        command_north, command_east, message.x, message.y
                    )
                    if step is None:
                        # No safe path means hover, never fall back to a direct
                        # line through an unknown or occupied costmap region.
                        active_target[:] = [message.x, message.y, cruise_down]
                        print("[UAV NAV2] No safe path; holding position")
                    elif node.corridor_is_clear(
                        step[0] - message.x, step[1] - message.y
                    ):
                        candidate_north, candidate_east = step
                        candidate_separation = math.hypot(
                            candidate_north - north, candidate_east - east
                        )
                        if candidate_separation > args.link_warning_range:
                            scale = (
                                args.link_warning_range / candidate_separation
                            )
                            candidate_north = north + (
                                candidate_north - north
                            ) * scale
                            candidate_east = east + (
                                candidate_east - east
                            ) * scale
                        active_target[:] = [
                            candidate_north,
                            candidate_east,
                            cruise_down,
                        ]
                    else:
                        # Escape a canopy in the vertical dimension instead of
                        # remaining trapped behind a two-dimensional obstacle.
                        cruise_down = max(-30.0, min(-args.altitude, message.z - 4.0))
                        active_target[:] = [message.x, message.y, cruise_down]
                        print(
                            "[UAV SAFETY] LiDAR obstacle in flight corridor; "
                            f"holding X/Y and climbing to {-cruise_down:.1f} m"
                        )
                    last_plan = elapsed
                if survey_targets and phase == "survey":
                    survey_error = math.hypot(
                        message.x - command_north, message.y - command_east
                    )
                    altitude_error = abs(message.z - cruise_down)
                    if survey_error <= 4.0 and altitude_error <= 1.5:
                        if survey_arrival_started is None:
                            survey_arrival_started = time.monotonic()
                            print(
                                f"[UAV SURVEY] {survey_targets[leg_index][0]} "
                                f"progressive lead established; holding "
                                f"{args.survey_settle_sec:.0f} s for costmap "
                                "integration"
                            )
                        elif (
                            time.monotonic() - survey_arrival_started
                            >= args.survey_settle_sec
                        ):
                            ready_path = (
                                args.coordination_dir
                                / f"survey_{leg_index + 1:02d}_ready"
                            )
                            ready_path.touch()
                            print(
                                f"[UAV SURVEY READY] {survey_targets[leg_index][0]} "
                                "progressive corridor is available; UGV may "
                                "move while mapping continues"
                            )
                            phase = "escort"
                    else:
                        survey_arrival_started = None
                error = math.sqrt(
                    (message.x - active_target[0]) ** 2
                    + (message.y - active_target[1]) ** 2
                    + (message.z - active_target[2]) ** 2
                )
                overhead_error = math.sqrt(
                    (message.x - north) ** 2
                    + (message.y - east) ** 2
                    + (message.z - cruise_down) ** 2
                )
                writer.writerow(
                    [
                        f"{elapsed:.3f}", phase, f"{active_target[0]:.3f}",
                        f"{active_target[1]:.3f}", f"{message.x:.3f}", f"{message.y:.3f}",
                        f"{message.z:.3f}", f"{error:.3f}",
                        f"{node.fix[0]:.8f}" if node.fix else "",
                        f"{node.fix[1]:.8f}" if node.fix else "",
                        f"{north:.3f}", f"{east:.3f}", f"{overhead_error:.3f}",
                    ]
                )
                stream.flush()
                takeoff_ready = abs(message.z - cruise_down) <= 1.0
                overhead_ready = overhead_error <= 2.0
                if not ready and takeoff_ready and (
                    bool(survey_targets) or overhead_ready
                ):
                    args.ready_file.touch()
                    ready = True
                    if survey_targets:
                        print("[UAV READY] Takeoff complete; aerial survey is active")
                        print(
                            f"[UAV SURVEY] Starting leg 1: "
                            f"{survey_targets[0][0]}"
                        )
                    else:
                        print("[UAV READY] Overhead follow position reached")
                if elapsed - last_report >= 5.0:
                    if survey_targets and phase == "survey":
                        print(
                            f"[UAV SURVEY] {survey_targets[leg_index][0]} "
                            f"remaining={survey_error:.1f} m "
                            f"(Nav2 step error={error:.2f} m)"
                        )
                    else:
                        print(
                            f"[UAV FOLLOW] overhead error={overhead_error:.2f} m "
                            f"(Nav2 step error={error:.2f} m)"
                        )
                    last_report = elapsed

            # The UAV has been escorting the Husky and is already in the goal
            # area when the final UGV leg completes. Land near the goal instead
            # of spending several minutes flying back to the launch pad.
            if latest_local is None:
                raise RuntimeError("No UAV local position available for landing")
            # Keep enough horizontal clearance from the Husky for the x500
            # landing gear and propellers. The previous "land where it is"
            # behavior could begin less than one metre from the UGV and cause
            # a vehicle collision during descent.
            separation_north = latest_local.x - north
            separation_east = latest_local.y - east
            separation = math.hypot(separation_north, separation_east)
            landing_clearance = 4.0
            if separation < 0.1:
                separation_north, separation_east, separation = 0.0, 1.0, 1.0
            landing_north = north + separation_north * landing_clearance / separation
            landing_east = east + separation_east * landing_clearance / separation
            active_target[:] = [landing_north, landing_east, cruise_down]
            print(
                "[UAV LANDING APPROACH] Moving to a clear point "
                f"{landing_clearance:.1f} m from the Husky"
            )
            approach_deadline = time.monotonic() + 45.0
            while time.monotonic() < approach_deadline:
                message = connection.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True, timeout=1
                )
                if message is None:
                    continue
                latest_local = message
                node.publish_px4_pose(message.x, message.y, message.z)
                if math.hypot(
                    message.x - landing_north,
                    message.y - landing_east,
                ) <= 0.75:
                    break
            else:
                raise TimeoutError("UAV did not reach its clear landing point")
            goal_vicinity_error = math.hypot(
                latest_local.x - north,
                latest_local.y - east,
            )
            print(
                "[UAV LANDING] UGV motion finished; landing near its final position "
                f"({goal_vicinity_error:.1f} m from the Husky)"
            )
            command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)
            landing_deadline = time.monotonic() + 120.0
            disarm_requested = False
            while time.monotonic() < landing_deadline:
                rclpy.spin_once(node, timeout_sec=0.0)
                message = connection.recv_match(
                    type=[
                        "HEARTBEAT",
                        "LOCAL_POSITION_NED",
                        "EXTENDED_SYS_STATE",
                    ],
                    blocking=True,
                    timeout=1,
                )
                if message is None:
                    continue
                if message.get_type() == "LOCAL_POSITION_NED":
                    node.publish_px4_pose(message.x, message.y, message.z)
                    landing_horizontal_error = math.hypot(
                        message.x - active_target[0],
                        message.y - active_target[1],
                    )
                    elapsed = time.monotonic() - start
                    writer.writerow(
                        [
                            f"{elapsed:.3f}", "landing",
                            f"{active_target[0]:.3f}", f"{active_target[1]:.3f}",
                            f"{message.x:.3f}", f"{message.y:.3f}",
                            f"{message.z:.3f}", f"{landing_horizontal_error:.3f}",
                            f"{node.fix[0]:.8f}" if node.fix else "",
                            f"{node.fix[1]:.8f}" if node.fix else "",
                            f"{north:.3f}", f"{east:.3f}",
                            f"{goal_vicinity_error:.3f}",
                        ]
                    )
                    stream.flush()
                    if (
                        not disarm_requested
                        and node.ground_clearance_m is not None
                        and node.ground_clearance_m <= 0.45
                    ):
                        print(
                            "[UAV TOUCHDOWN] 3D LiDAR ground clearance "
                            f"{node.ground_clearance_m:.2f} m; requesting disarm"
                        )
                        try:
                            command_and_wait(
                                connection,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0.0,
                                timeout=3.0,
                            )
                            disarm_requested = True
                        except (RuntimeError, TimeoutError):
                            # PX4 may reject disarm just before its land
                            # detector changes state. Retry on a later scan.
                            pass
                elif (
                    message.get_type() == "EXTENDED_SYS_STATE"
                    and message.landed_state
                    == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                    and not disarm_requested
                ):
                    try:
                        command_and_wait(
                            connection,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0.0,
                            timeout=3.0,
                        )
                        disarm_requested = True
                    except (RuntimeError, TimeoutError):
                        pass
                elif message.get_type() == "HEARTBEAT" and not (
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ):
                    print(
                        "[UAV LANDED] UAV disarmed near the UGV final position "
                        f"({goal_vicinity_error:.1f} m from the Husky)"
                    )
                    return 0
            return 2
    finally:
        setpoint_stop.set()
        heartbeat_stop.set()
        if setpoint_thread.is_alive():
            setpoint_thread.join(timeout=2)
        heartbeat_thread.join(timeout=2)
        connection.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
