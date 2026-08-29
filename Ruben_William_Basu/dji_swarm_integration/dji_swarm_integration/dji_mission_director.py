"""Three-DJI corridor mission, deterministic failover, return and landing."""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .dji_protocol import DjiCommand, encode_command
from .failover_protocol import elect_successor, initial_state, reconnect_as_follower


UAVS = ("dji0", "dji1", "dji2")


class DjiMissionDirector(Node):
    """Command only DJI poses and a predefined corridor; never consume UGV pose."""

    def __init__(self):
        super().__init__("dji_mission_director")
        self.scenario = str(self.declare_parameter("scenario", "normal").value).strip()
        if self.scenario not in {"normal", "permanent_failure", "reconnect"}:
            raise ValueError("scenario must be normal, permanent_failure, or reconnect")
        self.command_hz = float(self.declare_parameter("command_hz", 2.0).value)
        self.cruise_altitudes = tuple(
            float(v) for v in self.declare_parameter("cruise_altitudes", [16.0, 19.0, 22.0]).value
        )
        self.max_command_step_m = float(self.declare_parameter("max_command_step_m", 5.0).value)
        self.arrival_tolerance_m = float(self.declare_parameter("arrival_tolerance_m", 2.0).value)
        self.failure_interval_s = float(self.declare_parameter("failure_interval_s", 90.0).value)
        self.reconnect_after_s = float(self.declare_parameter("reconnect_after_s", 30.0).value)
        self.start_delay_s = float(self.declare_parameter("start_delay_s", 8.0).value)

        self.bays = {
            "dji0": (218.766, -279.249, 3.5217),
            "dji1": (218.766, -275.249, 3.5217),
            "dji2": (218.766, -271.249, 3.5217),
        }
        self.route = [
            (133.3040008545, -279.4549865723),
            (82.8774032593, -241.2203369141),
            (24.6462001801, -314.0588073730),
            (-29.5465202332, -286.4321289062),
        ]
        self.route_names = ("waypoint_1", "waypoint_2", "waypoint_3", "goal")
        self.poses: dict[str, PoseStamped] = {}
        self.pose_recv_ns: dict[str, int] = {}
        self._command_publishers = {
            uav: self.create_publisher(String, f"/coord/swarm/{uav}/dji_command", 10)
            for uav in UAVS
        }
        for uav in UAVS:
            self.create_subscription(
                PoseStamped, f"/{uav}/pose", lambda msg, name=uav: self._pose(name, msg), 10
            )
        self.role_pub = self.create_publisher(String, "/coord/swarm/role_state", 10)
        self.event_pub = self.create_publisher(String, "/coord/swarm/mission_events", 20)
        self.roles = initial_state()
        self.sequence = {uav: 0 for uav in UAVS}
        self.route_index = 0
        self.returning: set[str] = set()
        self.landed: set[str] = set()
        self.failed: set[str] = set()
        self.start_ns = self.get_clock().now().nanoseconds
        self.failure_stage = 0
        self.reconnected = False
        self.mission_complete = False
        self.last_role_json = ""
        self.create_timer(1.0 / max(0.5, self.command_hz), self._tick)
        self._event("MISSION", f"DJI scenario={self.scenario}; scout=DJI0")

    def _pose(self, uav: str, message: PoseStamped) -> None:
        self.poses[uav] = message
        self.pose_recv_ns[uav] = self.get_clock().now().nanoseconds

    def _event(self, kind: str, text: str) -> None:
        line = f"[{kind}] {text}"
        self.get_logger().info(line)
        message = String(); message.data = line
        self.event_pub.publish(message)

    def _publish_roles(self) -> None:
        payload = json.dumps(
            {
                "protocol": "eirax.swarm_role_state.v1",
                "term": self.roles.term,
                "active_scout": self.roles.active_scout,
                "states": self.roles.states,
                "reason": self.roles.reason,
            }, separators=(",", ":"), sort_keys=True,
        )
        if payload != self.last_role_json:
            scout = self.roles.active_scout.upper() if self.roles.active_scout else "UGV NAV2"
            followers = [u.upper() for u, role in self.roles.states.items() if role == "follower"]
            self._event("SWARM STATUS", f"Current leader/scout: {scout} | Followers: {', '.join(followers) or 'none'}")
            self.last_role_json = payload
        message = String(); message.data = payload
        self.role_pub.publish(message)

    def _scenario_transitions(self, elapsed_s: float) -> None:
        if self.scenario == "reconnect":
            if self.failure_stage == 0 and elapsed_s >= self.failure_interval_s:
                self.roles = elect_successor(self.roles, "dji0", permanent=False)
                self.failure_stage = 1
                self._event("DISCONNECTED", "DJI0 temporary communication failure; DJI1 is scout")
            elif self.failure_stage == 1 and elapsed_s >= self.failure_interval_s + self.reconnect_after_s:
                self.roles = reconnect_as_follower(self.roles, "dji0")
                self.failure_stage = 2
                self.reconnected = True
                self._event("RECOVERY", "DJI0 reconnected as follower; DJI1 remains scout")
            return
        if self.scenario != "permanent_failure":
            return
        while self.failure_stage < 3 and elapsed_s >= self.failure_interval_s * (self.failure_stage + 1):
            failed = UAVS[self.failure_stage]
            self.roles = elect_successor(self.roles, failed, permanent=True)
            self.failed.add(failed)
            self.returning.add(failed)
            self.failure_stage += 1
            successor = self.roles.active_scout.upper() if self.roles.active_scout else "UGV NAV2"
            self._event("DISCONNECTED", f"{failed.upper()} permanent communication failure")
            self._event("LEADER CHANGE", f"{successor} now owns scouting")
            self._event("RETURN TO BASE", f"{failed.upper()} returning to its own launch bay")
            if self.roles.active_scout is None:
                self._event("NAV2 FALLBACK", "All DJI links failed; UGV continues independently with Nav2")

    def _fresh_pose(self, uav: str, now_ns: int) -> PoseStamped | None:
        if now_ns - self.pose_recv_ns.get(uav, 0) > int(2.0e9):
            return None
        return self.poses.get(uav)

    def _bounded_target(self, pose: PoseStamped, target: tuple[float, float, float]):
        p = pose.pose.position
        dx, dy = target[0] - p.x, target[1] - p.y
        distance = math.hypot(dx, dy)
        if distance > self.max_command_step_m:
            scale = self.max_command_step_m / distance
            return p.x + dx * scale, p.y + dy * scale, target[2]
        return target

    def _send(self, uav: str, kind: str, target, yaw: float, reason: str) -> None:
        self.sequence[uav] += 1
        command = DjiCommand(
            uav_id=uav, sequence=self.sequence[uav], stamp_ns=self.get_clock().now().nanoseconds,
            kind=kind, x=target[0], y=target[1], z=target[2], yaw=yaw,
            pan_deg=0.0, tilt_deg=-45.0, reason=reason,
        )
        message = String(); message.data = encode_command(command)
        self._command_publishers[uav].publish(message)

    def _return_command(self, uav: str, pose: PoseStamped) -> None:
        bx, by, bz = self.bays[uav]
        p = pose.pose.position
        horizontal = math.hypot(bx - p.x, by - p.y)
        if horizontal > self.arrival_tolerance_m:
            target = self._bounded_target(pose, (bx, by, self.cruise_altitudes[UAVS.index(uav)]))
            self._send(uav, "return", target, math.atan2(by - p.y, bx - p.x), "return_to_bay")
        elif p.z > bz + 0.25:
            self._send(uav, "land", (bx, by, max(bz, p.z - 0.5)), 0.0, "vertical_landing")
        else:
            if uav not in self.landed:
                self.landed.add(uav)
                self._event("UAV LANDED VERIFIED", f"{uav.upper()} landed at its assigned bay")
            self._send(uav, "hold", (bx, by, bz), 0.0, "landed_hold")

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self.start_ns) * 1e-9
        self._scenario_transitions(elapsed_s)
        self._publish_roles()
        if elapsed_s < self.start_delay_s:
            return
        scout = self.roles.active_scout
        scout_pose = self._fresh_pose(scout, now_ns) if scout else None
        if scout and scout_pose is not None and not self.mission_complete:
            rx, ry = self.route[self.route_index]
            p = scout_pose.pose.position
            yaw = math.atan2(ry - p.y, rx - p.x)
            target = self._bounded_target(scout_pose, (rx, ry, self.cruise_altitudes[UAVS.index(scout)]))
            self._send(scout, "position", target, yaw, f"scout_{self.route_names[self.route_index]}")
            if math.hypot(rx - p.x, ry - p.y) <= self.arrival_tolerance_m:
                self._event("LEG COMPLETE", f"{scout.upper()} surveyed {self.route_names[self.route_index]}")
                self.route_index += 1
                if self.route_index >= len(self.route):
                    self.mission_complete = True
                    self.returning.update(UAVS)
                    self._event("MISSION FINISHED", "DJI survey corridor complete; all aircraft returning")

        # Followers use only the current scout aircraft pose, never UGV pose.
        if scout_pose is not None:
            sp = scout_pose.pose.position
            for uav in UAVS:
                if uav == scout or uav in self.returning or self.roles.role_of(uav) != "follower":
                    continue
                pose = self._fresh_pose(uav, now_ns)
                if pose is None:
                    continue
                side = -1.0 if uav == "dji0" else 1.0
                if uav == "dji1" and scout != "dji1": side = -1.0
                goal = (sp.x - 4.0, sp.y + side * 7.0, self.cruise_altitudes[UAVS.index(uav)])
                target = self._bounded_target(pose, goal)
                self._send(uav, "position", target, math.atan2(sp.y - pose.pose.position.y, sp.x - pose.pose.position.x), "follow_scout")

        for uav in tuple(self.returning):
            pose = self._fresh_pose(uav, now_ns)
            if pose is not None:
                self._return_command(uav, pose)


def main(args=None):
    rclpy.init(args=args)
    node = DjiMissionDirector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
