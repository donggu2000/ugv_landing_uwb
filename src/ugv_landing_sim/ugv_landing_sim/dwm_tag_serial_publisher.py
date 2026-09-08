import json
import math
import re
import socket
import threading
import time
from typing import Optional, Tuple

import rclpy
import serial
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


Vector3 = Tuple[float, float, float]


class DwmTagSerialPublisher(Node):
    def __init__(self):
        super().__init__("dwm_tag_serial_publisher")

        self.declare_parameter("frame_id", "bunker_base")
        self.declare_parameter("tag_port", "/dev/ttyACM3")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_s", 0.2)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("serial_stale_timeout_s", 3.0)
        self.declare_parameter("serial_mode", "tlv")
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("enter_shell", False)
        self.declare_parameter("location_command", "lec")
        self.declare_parameter("force_2d", True)
        self.declare_parameter("tag_z_m", 0.0)
        self.declare_parameter("xy_transform", "swap_invert_unit")
        self.declare_parameter("pose_topic", "/uwb/tag_pose")
        self.declare_parameter("marker_topic", "/uwb/tag_markers")
        self.declare_parameter("raw_topic", "/uwb/tag_raw")
        self.declare_parameter("udp_enabled", False)
        self.declare_parameter("udp_host", "10.99.40.20")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("tag_radius_m", 0.08)
        self.declare_parameter("label_z_offset_m", 0.16)
        self.declare_parameter("center_x_m", 0.5)
        self.declare_parameter("center_y_m", 0.5)
        self.declare_parameter("center_z_m", 0.0)
        self.declare_parameter("distance_label_z_offset_m", 0.12)

        self.frame_id = self.get_parameter("frame_id").value
        self.tag_port = self.get_parameter("tag_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.serial_timeout_s = float(self.get_parameter("serial_timeout_s").value)
        self.reconnect_delay_s = float(
            self.get_parameter("reconnect_delay_s").value
        )
        self.serial_stale_timeout_s = float(
            self.get_parameter("serial_stale_timeout_s").value
        )
        self.serial_mode = self.get_parameter("serial_mode").value
        self.poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)
        self.enter_shell = bool(self.get_parameter("enter_shell").value)
        self.location_command = self.get_parameter("location_command").value
        self.force_2d = bool(self.get_parameter("force_2d").value)
        self.tag_z_m = float(self.get_parameter("tag_z_m").value)
        self.xy_transform = self.get_parameter("xy_transform").value
        self.udp_enabled = bool(self.get_parameter("udp_enabled").value)
        self.udp_host = self.get_parameter("udp_host").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.tag_radius_m = float(self.get_parameter("tag_radius_m").value)
        self.label_z_offset_m = float(self.get_parameter("label_z_offset_m").value)
        self.center = (
            float(self.get_parameter("center_x_m").value),
            float(self.get_parameter("center_y_m").value),
            float(self.get_parameter("center_z_m").value),
        )
        self.distance_label_z_offset_m = float(
            self.get_parameter("distance_label_z_offset_m").value
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            10,
        )
        self.raw_pub = self.create_publisher(
            String,
            self.get_parameter("raw_topic").value,
            10,
        )

        self.serial_handle = None
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop_event = threading.Event()
        self.last_pose_log_time = 0.0
        self.last_missing_pose_log_time = 0.0
        self.reader_thread = threading.Thread(
            target=self._serial_connection_loop,
            daemon=True,
        )
        self.reader_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.serial_handle and self.serial_handle.is_open:
            self.serial_handle.close()
        self.udp_socket.close()
        super().destroy_node()

    def _open_serial(self) -> bool:
        try:
            self.serial_handle = serial.Serial(
                self.tag_port,
                self.baudrate,
                timeout=self.serial_timeout_s,
            )
        except serial.SerialException as exc:
            self.get_logger().error(f"Could not open DWM tag port {self.tag_port}: {exc}")
            return False
        self.get_logger().info(
            f"Reading DWM tag serial from {self.tag_port} at {self.baudrate} baud "
            f"in {self.serial_mode} mode"
        )
        return True

    def _serial_connection_loop(self):
        while not self.stop_event.is_set():
            if not self._open_serial():
                self.stop_event.wait(self.reconnect_delay_s)
                continue

            try:
                if self.serial_mode == "tlv":
                    self._tlv_poll_session()
                else:
                    self._shell_read_session()
            except (OSError, serial.SerialException) as exc:
                self.get_logger().error(f"DWM serial session failed: {exc}")
            finally:
                if self.serial_handle and self.serial_handle.is_open:
                    try:
                        self.serial_handle.close()
                    except (OSError, serial.SerialException):
                        pass

            if not self.stop_event.is_set():
                self.get_logger().warn(
                    f"DWM serial connection lost; retrying {self.tag_port} in "
                    f"{self.reconnect_delay_s:.1f}s"
                )
                self.stop_event.wait(self.reconnect_delay_s)

    def _shell_read_session(self):
        # This J-Link CDC console can stop responding after a serial break.
        # Enter the DWM shell gently, then start the continuous location stream.
        self.serial_handle.dtr = True
        self.serial_handle.rts = True
        self.serial_handle.reset_input_buffer()
        self.get_logger().info("DWM shell session initialized; sending enter sequence")

        if self.enter_shell:
            self.serial_handle.write(b"\r\r")
            self.get_logger().info("DWM shell enter sequence sent")

        if self.location_command:
            command = self.location_command.strip().encode("ascii") + b"\r"
            self.serial_handle.write(command)

        last_data_time = time.monotonic()
        line_buffer = ""
        self.get_logger().info("DWM serial chunk read loop started")
        while not self.stop_event.is_set():
            try:
                raw_chunk = self.serial_handle.read(512)
            except (OSError, serial.SerialException) as exc:
                self.get_logger().error(f"DWM serial read failed: {exc}")
                return

            if not raw_chunk:
                if time.monotonic() - last_data_time >= self.serial_stale_timeout_s:
                    self.get_logger().warn(
                        f"No DWM serial data for {self.serial_stale_timeout_s:.1f}s; "
                        "reinitializing shell"
                    )
                    return
                continue

            last_data_time = time.monotonic()
            line_buffer += raw_chunk.decode("utf-8", errors="ignore")
            while "\n" in line_buffer:
                line, line_buffer = line_buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                raw_msg = String()
                raw_msg.data = line
                self.raw_pub.publish(raw_msg)

                xyz = self._parse_position(line)
                if xyz is None:
                    now = time.monotonic()
                    if (
                        line.startswith("DIST,")
                        and now - self.last_missing_pose_log_time >= 5.0
                    ):
                        self.get_logger().warn(
                            "DWM reports anchor distances but no POS solution; "
                            "check anchor initiator/ranging and tag positioning mode"
                        )
                        self.last_missing_pose_log_time = now
                    self.get_logger().debug(f"Ignoring unparsed DWM line: {line}")
                    continue

                if self.force_2d:
                    xyz = (xyz[0], xyz[1], self.tag_z_m)
                xyz = self._transform_xy(xyz)
                self._publish_pose_and_marker(xyz)

    def _tlv_poll_session(self):
        poll_period_s = 1.0 / self.poll_rate_hz
        while not self.stop_event.is_set():
            start_time = time.monotonic()
            try:
                self.serial_handle.reset_input_buffer()
                self.serial_handle.write(bytes([0x0C, 0x00]))
                time.sleep(0.10)
                response = self.serial_handle.read(512)
            except (OSError, serial.SerialException) as exc:
                self.get_logger().error(f"DWM TLV poll failed: {exc}")
                return

            if response:
                raw_msg = String()
                raw_msg.data = response.hex(" ")
                self.raw_pub.publish(raw_msg)

                xyz = self._parse_tlv_location(response)
                if xyz is not None:
                    if self.force_2d:
                        xyz = (xyz[0], xyz[1], self.tag_z_m)
                    xyz = self._transform_xy(xyz)
                    self._publish_pose_and_marker(xyz)
                else:
                    self.get_logger().debug(
                        f"Ignoring unparsed TLV response: {response.hex(' ')}"
                    )

            elapsed_s = time.monotonic() - start_time
            time.sleep(max(0.0, poll_period_s - elapsed_s))

    def _parse_position(self, line: str) -> Optional[Vector3]:
        est_match = re.search(
            r"est\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)",
            line,
        )
        if est_match:
            return tuple(float(est_match.group(i)) for i in range(1, 4))

        pos_match = re.search(
            r"(?:^|[;,\s])POS\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)",
            line,
            re.IGNORECASE,
        )
        if pos_match:
            return tuple(float(pos_match.group(i)) for i in range(1, 4))

        bracket_match = re.search(
            r"tag\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)",
            line,
            re.IGNORECASE,
        )
        if bracket_match:
            return tuple(float(bracket_match.group(i)) for i in range(1, 4))

        return None

    def _parse_tlv_location(self, data: bytes) -> Optional[Vector3]:
        index = 0
        while index + 2 <= len(data):
            tlv_type = data[index]
            tlv_len = data[index + 1]
            value_start = index + 2
            value_end = value_start + tlv_len
            if value_end > len(data):
                break

            if tlv_type == 0x41 and tlv_len >= 13:
                value = data[value_start:value_end]
                x_mm = int.from_bytes(value[0:4], "little", signed=True)
                y_mm = int.from_bytes(value[4:8], "little", signed=True)
                z_mm = int.from_bytes(value[8:12], "little", signed=True)
                return (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)

            index = value_end

        return None

    def _transform_xy(self, xyz: Vector3) -> Vector3:
        x, y, z = xyz
        if self.xy_transform == "identity":
            return xyz
        if self.xy_transform == "swap_invert_unit":
            return (1.0 - y, 1.0 - x, z)
        if self.xy_transform == "invert_unit":
            return (1.0 - x, 1.0 - y, z)
        if self.xy_transform == "swap":
            return (y, x, z)
        self.get_logger().warn(
            f"Unknown xy_transform '{self.xy_transform}', using identity",
            throttle_duration_sec=2.0,
        )
        return xyz

    def _publish_pose_and_marker(self, xyz: Vector3):
        now = time.monotonic()
        if now - self.last_pose_log_time >= 5.0:
            self.get_logger().info(
                f"Publishing /uwb/tag_pose x={xyz[0]:.3f} "
                f"y={xyz[1]:.3f} z={xyz[2]:.3f}"
            )
            self.last_pose_log_time = now

        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self._send_udp_pose(xyz, stamp)

        markers = MarkerArray()
        markers.markers.append(self._tag_marker(xyz, stamp))
        markers.markers.append(self._label_marker(xyz, stamp))
        markers.markers.append(self._center_to_tag_line_marker(xyz, stamp))
        markers.markers.append(self._distance_label_marker(xyz, stamp))
        self.marker_pub.publish(markers)

    def _send_udp_pose(self, xyz: Vector3, stamp):
        if not self.udp_enabled:
            return
        payload = {
            "frame_id": self.frame_id,
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
        }
        try:
            self.udp_socket.sendto(
                json.dumps(payload).encode("utf-8"),
                (self.udp_host, self.udp_port),
            )
        except OSError as exc:
            self.get_logger().warn(
                f"UDP pose send failed to {self.udp_host}:{self.udp_port}: {exc}",
                throttle_duration_sec=2.0,
            )

    def _tag_marker(self, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_tag"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.tag_radius_m * 2.0
        marker.scale.y = self.tag_radius_m * 2.0
        marker.scale.z = self.tag_radius_m * 2.0
        marker.color.r = 0.1
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _label_marker(self, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_tag_label"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2] + self.label_z_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.10
        marker.color.r = 0.8
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (
            f"TAG x={xyz[0]:.2f} y={xyz[1]:.2f} "
            f"d={self._center_distance_m(xyz):.2f}m"
        )
        return marker

    def _center_to_tag_line_marker(self, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_center_to_tag"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        z = self.center[2] + 0.03
        center = Point(x=self.center[0], y=self.center[1], z=z)
        tag = Point(x=xyz[0], y=xyz[1], z=z)
        marker.points = [center, tag]
        return marker

    def _distance_label_marker(self, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_center_distance_label"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = (self.center[0] + xyz[0]) * 0.5
        marker.pose.position.y = (self.center[1] + xyz[1]) * 0.5
        marker.pose.position.z = self.center[2] + self.distance_label_z_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.11
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{self._center_distance_m(xyz):.2f} m"
        return marker

    def _center_distance_m(self, xyz: Vector3) -> float:
        return math.hypot(xyz[0] - self.center[0], xyz[1] - self.center[1])


def main(args=None):
    rclpy.init(args=args)
    node = DwmTagSerialPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
