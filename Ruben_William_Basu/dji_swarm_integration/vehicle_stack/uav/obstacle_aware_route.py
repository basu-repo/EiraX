"""Fly a fixed-altitude UAV route using live Nav2 obstacle-aware plans."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "px4_runtime/python"))
    from pymavlink import mavutil

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.action import ActionClient
from rclpy.node import Node

from uav.gazebo_pose import GazeboPoseReader
from uav.x500_mission import command_and_wait, send_local_setpoint, wait_for_position


class AerialPlanner(Node):
    """Expose the aerial LiDAR pose, Nav2 planner and visible planned path."""

    def __init__(self) -> None:
        super().__init__("uav_obstacle_aware_route")
        self.lidar_pose: tuple[float, float, float] | None = None
        self.create_subscription(Odometry, "/uav/lidar_odom", self._odom, 20)
        self.client = ActionClient(
            self, ComputePathToPose, "/uav_nav/compute_path_to_pose"
        )
        self.path_publisher = self.create_publisher(RosPath, "/uav/planned_path", 10)
        self.last_plan_mode = "direct"

    def _odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.lidar_pose = position.x, position.y, position.z

    def plan_step(
        self,
        target_north: float,
        target_east: float,
        current_north: float,
        current_east: float,
        lookahead_m: float,
    ) -> tuple[float, float, float] | None:
        """Return North, East and full path length for a safe short setpoint."""
        if self.lidar_pose is None or not self.client.wait_for_server(timeout_sec=0.2):
            return None

        lidar_x, lidar_y, _ = self.lidar_pose
        delta_north = target_north - current_north
        delta_east = target_east - current_east
        target_distance = math.hypot(delta_north, delta_east)
        # Receding-horizon planning: only ask the rolling sensor costmap for a
        # nearby observed subgoal. The final mission target remains unchanged.
        # This avoids treating unseen space near a distant waypoint as blocked.
        if target_distance > 30.0:
            scale = 30.0 / target_distance
            delta_north *= scale
            delta_east *= scale

        def request_path(east: float, north: float) -> RosPath | None:
            # Gazebo / ROS use East-North-Up: X is East and Y is North.
            # PX4 local position uses North-East-Down: X is North and Y is East.
            goal = ComputePathToPose.Goal()
            goal.goal = PoseStamped()
            goal.goal.header.frame_id = "uav_lidar_odom"
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.goal.pose.position.x = lidar_x + east
            goal.goal.pose.position.y = lidar_y + north
            goal.goal.pose.orientation.w = 1.0
            goal.planner_id = "GridBased"
            goal.use_start = False
            sent = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, sent, timeout_sec=1.0)
            handle = sent.result() if sent.done() else None
            if handle is None or not handle.accepted:
                return None
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=2.0)
            wrapped = result_future.result() if result_future.done() else None
            if wrapped is None or not wrapped.result.path.poses:
                return None
            return wrapped.result.path

        path = request_path(delta_east, delta_north)
        self.last_plan_mode = "direct"
        if path is None:
            # Short-range manoeuvre mode. Search left and right for a nearby
            # free subgoal, move only a short distance, then re-aim at the
            # actual mission target on the next planning cycle.
            heading = math.atan2(delta_north, delta_east)
            detour_distance = min(15.0, max(8.0, target_distance * 0.5))
            for offset in (
                math.radians(45),
                -math.radians(45),
                math.radians(90),
                -math.radians(90),
            ):
                angle = heading + offset
                path = request_path(
                    detour_distance * math.cos(angle),
                    detour_distance * math.sin(angle),
                )
                if path is not None:
                    self.last_plan_mode = "short_range_detour"
                    break
        if path is None:
            return None

        path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path)

        path_length = 0.0
        previous_x, previous_y = lidar_x, lidar_y
        selected_x = path.poses[-1].pose.position.x
        selected_y = path.poses[-1].pose.position.y
        selected = False
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            path_length += math.hypot(x - previous_x, y - previous_y)
            previous_x, previous_y = x, y
            if not selected and math.hypot(x - lidar_x, y - lidar_y) >= lookahead_m:
                selected_x, selected_y = x, y
                selected = True

        # Convert the short ROS displacement back to PX4 North-East.
        step_east = selected_x - lidar_x
        step_north = selected_y - lidar_y
        return (
            current_north + step_north,
            current_east + step_east,
            path_length,
        )


def fly_route(
    port: int,
    output_directory: Path,
    targets: list[tuple[str, float, float]],
    altitude_m: float,
    speed_mps: float = 2.0,
    reach_radius_m: float = 2.0,
    lookahead_m: float = 2.0,
    model_name: str = "x500_mapping_0",
) -> int:
    """Take off, visit every target using Nav2 plans, then land at the goal."""
    output_directory.mkdir(parents=True, exist_ok=True)
    connection = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{port}", source_system=250
    )
    if connection.wait_heartbeat(timeout=30) is None:
        raise TimeoutError(f"No PX4 heartbeat received on UDP port {port}")
    print("[OK] Connected to PX4 x500")

    rclpy.init()
    planner = AerialPlanner()
    pose_reader = GazeboPoseReader("baylands_editable", model_name=model_name)
    heartbeat_stop = threading.Event()
    setpoint_stop = threading.Event()
    active_target = [0.0, 0.0, -altitude_m]

    def heartbeats() -> None:
        while not heartbeat_stop.is_set():
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            heartbeat_stop.wait(0.8)

    def setpoints() -> None:
        while not setpoint_stop.is_set():
            send_local_setpoint(connection, *active_target)
            setpoint_stop.wait(0.1)

    heartbeat_thread = threading.Thread(target=heartbeats, daemon=True)
    setpoint_thread = threading.Thread(target=setpoints, daemon=True)
    heartbeat_thread.start()
    setpoint_thread.start()

    csv_path = output_directory / "uav_route_trajectory.csv"
    summary_path = output_directory / "uav_route_summary.json"
    reached: list[dict[str, float | str]] = []
    localization_errors: list[float] = []
    no_path_events = 0
    mission_completed = False
    landed = False

    try:
        wait_for_position(connection)
        ground_origin = pose_reader.wait()
        for parameter_name in ("MPC_XY_VEL_MAX", "MPC_XY_CRUISE"):
            connection.mav.param_set_send(
                connection.target_system,
                connection.target_component,
                parameter_name.encode("ascii"),
                speed_mps,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        print(f"[SPEED] Horizontal flight limited to {speed_mps:.1f} m/s")

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
                    raise RuntimeError("PX4 preflight checks did not become ready")
                print("[WAITING] PX4 preflight checks are not ready.")
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
            raise TimeoutError("PX4 did not enter offboard mode")

        print(f"[TAKEOFF] Climbing vertically to {altitude_m:.1f} m")
        takeoff_deadline = time.monotonic() + 120.0
        current = None
        while time.monotonic() < takeoff_deadline:
            rclpy.spin_once(planner, timeout_sec=0.01)
            current = connection.recv_match(
                type="LOCAL_POSITION_NED", blocking=True, timeout=0.2
            )
            if current and abs(current.z + altitude_m) <= 1.0:
                break
        if current is None or abs(current.z + altitude_m) > 1.0:
            raise TimeoutError("UAV did not reach its safe flight altitude")
        print("[AIRBORNE] Safe flight altitude reached")

        route_names = " -> ".join(name for name, _, _ in targets)
        print(f"[MISSION] spawn -> {route_names} -> land")
        start = time.monotonic()
        last_report = start - 10.0
        last_plan = start - 10.0
        last_path_time = start
        target_index = 0
        latest_path_length = float("nan")

        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_sec",
                    "target",
                    "target_north_m",
                    "target_east_m",
                    "planned_path_length_m",
                    "setpoint_north_m",
                    "setpoint_east_m",
                    "px4_north_m",
                    "px4_east_m",
                    "px4_down_m",
                    "target_distance_m",
                    "gazebo_north_m",
                    "gazebo_east_m",
                    "gazebo_down_m",
                    "localization_error_m",
                    "planner_state",
                ]
            )
            while target_index < len(targets):
                rclpy.spin_once(planner, timeout_sec=0.01)
                message = connection.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True, timeout=0.1
                )
                if message is None:
                    continue
                now = time.monotonic()
                name, target_north, target_east = targets[target_index]
                target_distance = math.hypot(
                    message.x - target_north, message.y - target_east
                )
                if target_distance <= reach_radius_m and abs(message.z + altitude_m) <= 1.5:
                    reached.append(
                        {
                            "target": name,
                            "elapsed_sec": now - start,
                            "reach_error_m": target_distance,
                        }
                    )
                    print(f"[REACHED] {name} within {target_distance:.2f} m")
                    target_index += 1
                    last_path_time = now
                    if target_index >= len(targets):
                        mission_completed = True
                        break
                    continue

                planner_state = "following_path"
                if now - last_plan >= 0.5:
                    step = planner.plan_step(
                        target_north,
                        target_east,
                        message.x,
                        message.y,
                        lookahead_m,
                    )
                    if step is None:
                        active_target[:] = [message.x, message.y, -altitude_m]
                        no_path_events += 1
                        planner_state = "holding_no_safe_path"
                        print(f"[HOLD] No safe path to {name}; maintaining position")
                    else:
                        active_target[:] = [step[0], step[1], -altitude_m]
                        latest_path_length = step[2]
                        last_path_time = now
                        planner_state = planner.last_plan_mode
                    last_plan = now

                ground = pose_reader.at(message.time_boot_ms / 1000.0)
                localization_error = float("nan")
                gazebo_north = gazebo_east = gazebo_down = float("nan")
                if ground is not None:
                    gazebo_north = ground[1] - ground_origin[1]
                    gazebo_east = ground[0] - ground_origin[0]
                    gazebo_down = -(ground[2] - ground_origin[2])
                    localization_error = math.sqrt(
                        (message.x - gazebo_north) ** 2
                        + (message.y - gazebo_east) ** 2
                        + (message.z - gazebo_down) ** 2
                    )
                    localization_errors.append(localization_error)

                writer.writerow(
                    [
                        f"{now - start:.3f}",
                        name,
                        f"{target_north:.3f}",
                        f"{target_east:.3f}",
                        f"{latest_path_length:.3f}",
                        f"{active_target[0]:.3f}",
                        f"{active_target[1]:.3f}",
                        f"{message.x:.3f}",
                        f"{message.y:.3f}",
                        f"{message.z:.3f}",
                        f"{target_distance:.3f}",
                        f"{gazebo_north:.3f}",
                        f"{gazebo_east:.3f}",
                        f"{gazebo_down:.3f}",
                        f"{localization_error:.3f}",
                        planner_state,
                    ]
                )
                stream.flush()

                if now - last_report >= 5.0:
                    print(
                        f"[FLIGHT] {name}: distance={target_distance:.1f} m, "
                        f"planned path={latest_path_length:.1f} m"
                    )
                    last_report = now
                if now - last_path_time > 120.0:
                    raise TimeoutError(
                        f"No safe path to {name} for 120 seconds; refusing direct flight"
                    )

        print("[LANDING] Goal reached; descending vertically")
        setpoint_stop.set()
        setpoint_thread.join(timeout=2.0)
        command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)
        landing_deadline = time.monotonic() + 120.0
        while time.monotonic() < landing_deadline:
            heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat and not (
                heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ):
                landed = True
                print("[LANDED] UAV landed and disarmed at the goal")
                break

        summary = {
            "mission_completed": mission_completed,
            "landed_and_disarmed": landed,
            "route": [name for name, _, _ in targets],
            "targets_reached": reached,
            "no_safe_path_holds": no_path_events,
            "speed_limit_mps": speed_mps,
            "flight_altitude_m": altitude_m,
            "reach_radius_m": reach_radius_m,
            "localization_samples": len(localization_errors),
            "localization_mean_error_m": (
                statistics.fmean(localization_errors) if localization_errors else None
            ),
            "localization_rmse_m": (
                math.sqrt(statistics.fmean(value * value for value in localization_errors))
                if localization_errors
                else None
            ),
            "localization_max_error_m": (
                max(localization_errors) if localization_errors else None
            ),
            "ground_truth_used_for_control": False,
        }
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        return 0 if mission_completed and landed else 2
    finally:
        setpoint_stop.set()
        heartbeat_stop.set()
        if setpoint_thread.is_alive():
            setpoint_thread.join(timeout=2)
        heartbeat_thread.join(timeout=2)
        pose_reader.close()
        connection.close()
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
