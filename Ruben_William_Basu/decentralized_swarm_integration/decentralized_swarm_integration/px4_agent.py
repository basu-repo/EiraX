"""One guarded MAVLink/PX4 offboard adapter per swarm UAV."""

from __future__ import annotations

import math
import queue
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .px4_protocol import Point3, ned_to_enu, validate_setpoint, yaw_ned_to_enu


class Px4Agent(Node):
    def __init__(self):
        super().__init__("px4_agent")
        try:
            from pymavlink import mavutil
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "pymavlink is required; install the system/ROS pymavlink package"
            ) from exc
        self.mavutil = mavutil
        self.uav_id = str(self.declare_parameter("uav_id", "uav").value).strip()
        self.connection_url = str(
            self.declare_parameter("connection_url", "udpin:0.0.0.0:14540").value
        ).strip()
        self.source_system = int(self.declare_parameter("source_system", 240).value)
        self.control_enabled = bool(self.declare_parameter("control_enabled", False).value)
        self.world_frame = str(self.declare_parameter("world_frame", "map").value).strip()
        self.map_origin = Point3(
            float(self.declare_parameter("map_origin_x", 0.0).value),
            float(self.declare_parameter("map_origin_y", 0.0).value),
            float(self.declare_parameter("map_origin_z", 0.0).value),
        )
        self.max_radius_m = float(self.declare_parameter("max_radius_m", 120.0).value)
        self.min_altitude_m = float(self.declare_parameter("min_altitude_m", 2.0).value)
        self.max_altitude_m = float(self.declare_parameter("max_altitude_m", 30.0).value)
        self.max_step_m = float(self.declare_parameter("max_step_m", 15.0).value)
        self.command_timeout_s = float(self.declare_parameter("command_timeout_s", 1.0).value)
        self.setpoint_rate_hz = float(self.declare_parameter("setpoint_rate_hz", 10.0).value)
        prefix = f"/{self.uav_id}/px4"
        setpoint_topic = str(
            self.declare_parameter("setpoint_topic", f"/coord/swarm/{self.uav_id}/setpoint").value
        )
        enable_topic = str(
            self.declare_parameter("enable_topic", f"/coord/swarm/{self.uav_id}/control_enable").value
        )
        self._odom_pub = self.create_publisher(Odometry, f"{prefix}/odom", 20)
        self._pose_pub = self.create_publisher(PoseStamped, f"{prefix}/pose", 20)
        self._status_pub = self.create_publisher(String, f"{prefix}/status", 10)
        self.create_subscription(PoseStamped, setpoint_topic, self._on_setpoint, 10)
        self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.create_service(SetBool, f"{prefix}/arm", self._on_arm_service)
        self.create_service(Trigger, f"{prefix}/land", self._on_land_service)

        self._connection = None
        self._current_enu: Point3 | None = None
        self._home_enu: Point3 | None = None
        self._current_yaw_enu = 0.0
        self._last_setpoint = None
        self._last_setpoint_time = 0.0
        self._last_heartbeat_time = 0.0
        self._armed = False
        self._offboard = False
        self._command_queue = queue.Queue()
        self._stop = threading.Event()
        self._io_thread = threading.Thread(target=self._io_loop, daemon=True)
        self._io_thread.start()
        self.get_logger().info(
            f"PX4 agent={self.uav_id} endpoint={self.connection_url} "
            f"control_enabled={self.control_enabled}"
        )

    def _on_enable(self, message: Bool):
        self.control_enabled = bool(message.data)
        if not self.control_enabled:
            self._last_setpoint = None
        self._publish_status("control_enabled" if self.control_enabled else "control_disabled")

    @staticmethod
    def _yaw_from_quaternion(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )

    def _on_setpoint(self, message: PoseStamped):
        if not self.control_enabled:
            self._publish_status("setpoint_rejected_control_disabled")
            return
        if self._home_enu is None:
            self._publish_status("setpoint_rejected_no_px4_position")
            return
        if str(message.header.frame_id).strip() != self.world_frame:
            self._publish_status(
                f"setpoint_rejected_frame:{message.header.frame_id or 'empty'}"
            )
            return
        position = message.pose.position
        try:
            safe = validate_setpoint(
                x_enu=float(position.x),
                y_enu=float(position.y),
                z_enu=float(position.z),
                yaw_enu=self._yaw_from_quaternion(message.pose.orientation),
                home_enu=self._home_enu,
                current_enu=self._current_enu,
                max_radius_m=self.max_radius_m,
                min_altitude_m=self.min_altitude_m,
                max_altitude_m=self.max_altitude_m,
                max_step_m=self.max_step_m,
            )
        except ValueError as exc:
            self._publish_status(f"setpoint_rejected:{exc}")
            return
        self._last_setpoint = safe
        self._last_setpoint_time = time.monotonic()

    def _on_arm_service(self, request, response):
        if not request.data:
            ok, reason = self._request_command("disarm")
        elif not self.control_enabled:
            ok, reason = False, "control is disabled"
        elif self._last_setpoint is None:
            ok, reason = False, "a valid setpoint is required before arming"
        else:
            # PX4 requires a setpoint stream before entering offboard mode.
            self._last_setpoint_time = time.monotonic()
            deadline = time.monotonic() + 1.2
            while time.monotonic() < deadline and self._last_setpoint is not None:
                self._last_setpoint_time = time.monotonic()
                time.sleep(0.05)
            self._last_setpoint_time = time.monotonic()
            ok, reason = self._request_command("arm_offboard")
        response.success = ok
        response.message = reason
        return response

    def _on_land_service(self, _request, response):
        ok, reason = self._request_command("land")
        response.success = ok
        response.message = reason
        return response

    def _request_command(self, command):
        event = threading.Event()
        result = {}
        self._command_queue.put((command, event, result))
        if not event.wait(8.0):
            return False, f"PX4 {command} request timed out"
        return bool(result.get("success")), str(result.get("message", command))

    def _io_loop(self):
        backoff = 0.5
        while not self._stop.is_set():
            try:
                self._connection = self.mavutil.mavlink_connection(
                    self.connection_url, source_system=self.source_system
                )
                if self._connection.wait_heartbeat(timeout=5.0) is None:
                    raise TimeoutError("heartbeat timeout")
                backoff = 0.5
                self._publish_status("connected")
                self._connected_loop()
            except Exception as exc:
                if not self._stop.is_set():
                    self._publish_status(f"disconnected:{type(exc).__name__}")
            finally:
                if self._connection is not None:
                    self._connection.close()
                    self._connection = None
            self._stop.wait(backoff)
            backoff = min(5.0, backoff * 2.0)

    def _connected_loop(self):
        next_setpoint = 0.0
        period = 1.0 / max(2.0, self.setpoint_rate_hz)
        while not self._stop.is_set() and self._connection is not None:
            now = time.monotonic()
            self._process_command()
            if (
                self.control_enabled
                and self._last_setpoint is not None
                and now - self._last_setpoint_time <= self.command_timeout_s
                and now >= next_setpoint
            ):
                self._send_setpoint(self._last_setpoint)
                next_setpoint = now + period
            message = self._connection.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT"],
                blocking=True,
                timeout=min(0.05, period),
            )
            if message is not None:
                self._handle_mavlink(message)

    def _process_command(self):
        try:
            command, event, result = self._command_queue.get_nowait()
        except queue.Empty:
            return
        try:
            if command == "arm_offboard":
                # PX4's supported sequence is: stream setpoints, request
                # OFFBOARD, confirm the mode, then arm. Arming first can be
                # rejected depending on the active preflight/mode checks.
                custom_mode = 6 << 16
                self._connection.mav.set_mode_send(
                    self._connection.target_system,
                    self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode,
                )
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    heartbeat = self._connection.recv_match(
                        type="HEARTBEAT", blocking=True, timeout=0.25
                    )
                    if heartbeat and ((int(heartbeat.custom_mode) >> 16) & 0xFF) == 6:
                        self._offboard = True
                        break
                else:
                    raise TimeoutError("PX4 did not confirm offboard mode")
                self._send_command(
                    self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0
                )
            elif command == "disarm":
                self._send_command(self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
            elif command == "land":
                self._send_command(self.mavutil.mavlink.MAV_CMD_NAV_LAND)
            result.update(success=True, message=f"PX4 {command} confirmed")
        except Exception as exc:
            result.update(success=False, message=str(exc))
        finally:
            event.set()

    def _send_command(self, command, first_parameter=0.0):
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            command,
            0,
            first_parameter,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            acknowledgement = self._connection.recv_match(
                type="COMMAND_ACK", blocking=True, timeout=0.25
            )
            if acknowledgement is None or int(acknowledgement.command) != int(command):
                continue
            accepted = {
                self.mavutil.mavlink.MAV_RESULT_ACCEPTED,
                self.mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }
            if int(acknowledgement.result) not in accepted:
                raise RuntimeError(
                    f"PX4 rejected command {command}: result={acknowledgement.result}"
                )
            return
        raise TimeoutError(f"PX4 did not acknowledge command {command}")

    def _send_setpoint(self, setpoint):
        mask = (
            self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | self.mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self._connection.target_system,
            self._connection.target_component,
            self.mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            setpoint.north,
            setpoint.east,
            setpoint.down,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            setpoint.yaw_ned,
            0.0,
        )

    def _handle_mavlink(self, message):
        message_type = message.get_type()
        if message_type == "LOCAL_POSITION_NED":
            local_enu = ned_to_enu(float(message.x), float(message.y), float(message.z))
            self._current_enu = Point3(
                self.map_origin.x + local_enu.x,
                self.map_origin.y + local_enu.y,
                self.map_origin.z + local_enu.z,
            )
            if self._home_enu is None:
                self._home_enu = self._current_enu
            self._publish_odometry(message)
        elif message_type == "ATTITUDE":
            self._current_yaw_enu = yaw_ned_to_enu(float(message.yaw))
        elif message_type == "HEARTBEAT":
            self._last_heartbeat_time = time.monotonic()
            self._armed = bool(
                int(message.base_mode) & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self._offboard = ((int(message.custom_mode) >> 16) & 0xFF) == 6

    def _publish_odometry(self, local_position):
        if self._current_enu is None:
            return
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.world_frame
        message.child_frame_id = f"{self.uav_id}/base_link"
        message.pose.pose.position.x = self._current_enu.x
        message.pose.pose.position.y = self._current_enu.y
        message.pose.pose.position.z = self._current_enu.z
        half = self._current_yaw_enu / 2.0
        message.pose.pose.orientation.z = math.sin(half)
        message.pose.pose.orientation.w = math.cos(half)
        velocity = ned_to_enu(float(local_position.vx), float(local_position.vy), float(local_position.vz))
        message.twist.twist.linear.x = velocity.x
        message.twist.twist.linear.y = velocity.y
        message.twist.twist.linear.z = velocity.z
        self._odom_pub.publish(message)
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self._pose_pub.publish(pose)

    def _publish_status(self, state):
        message = String()
        message.data = (
            f"uav_id={self.uav_id} state={state} armed={str(self._armed).lower()} "
            f"offboard={str(self._offboard).lower()} control_enabled={str(self.control_enabled).lower()}"
        )
        self._status_pub.publish(message)

    def destroy_node(self):
        self._stop.set()
        self._io_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Px4Agent()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
