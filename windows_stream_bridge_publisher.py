#!/usr/bin/env python3
"""Receive Windows TCP packets and publish ROS Image + PointStamped messages.

Packet protocol:
    Frame packet:   [4-byte big-endian length][JPEG payload]
    Control packet: [4-byte length with high-bit set][JSON payload]
"""

from __future__ import annotations

import json
import socket
import struct
import time

import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_image_codec import cv_frame_to_image_msg
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from opencv_ros2_bridge_interfaces.srv import GetObjectPoint
except ImportError:
    GetObjectPoint = None


class WindowsStreamBridgePublisher(Node):
    def __init__(self) -> None:
        super().__init__("windows_stream_bridge_publisher")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 5001)
        self.declare_parameter("publish_topic", "/camera/image_raw")
        self.declare_parameter("point_topic", "/camera/object_point")
        self.declare_parameter("point_service", "/camera/get_object_point")
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("bridge_control_topic", "/camera/bridge_control")
        self.declare_parameter("enable_publish_fps_log", True)
        self.declare_parameter("max_frame_bytes", 4_000_000)
        self.declare_parameter("max_control_bytes", 4096)
        self.declare_parameter("scale_px_per_meter", 100.0)
        self.declare_parameter("default_z_m", 0.0)

        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
        self.point_topic = self.get_parameter("point_topic").get_parameter_value().string_value
        self.point_service_name = (
            self.get_parameter("point_service").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.bridge_control_topic = (
            self.get_parameter("bridge_control_topic").get_parameter_value().string_value
        )
        self.enable_publish_fps_log = (
            self.get_parameter("enable_publish_fps_log").get_parameter_value().bool_value
        )
        self.max_frame_bytes = self.get_parameter("max_frame_bytes").get_parameter_value().integer_value
        self.max_control_bytes = (
            self.get_parameter("max_control_bytes").get_parameter_value().integer_value
        )
        self.scale_px_per_meter = (
            self.get_parameter("scale_px_per_meter").get_parameter_value().double_value
        )
        self.default_z_m = self.get_parameter("default_z_m").get_parameter_value().double_value

        if self.max_frame_bytes <= 1024:
            self.max_frame_bytes = 4_000_000
        if self.max_control_bytes <= 64:
            self.max_control_bytes = 4096
        if self.scale_px_per_meter <= 0.0:
            self.scale_px_per_meter = 100.0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Image, self.publish_topic, image_qos)
        self.point_publisher = self.create_publisher(PointStamped, self.point_topic, 10)
        self.bridge_control_subscription = self.create_subscription(
            String,
            self.bridge_control_topic,
            self.bridge_control_callback,
            10,
        )

        self._latest_point_lock = threading.Lock()
        self._latest_point: PointStamped | None = None
        self._latest_point_source: str = ""

        if GetObjectPoint is None:
            self.point_service = None
            self.get_logger().warning(
                "opencv_ros2_bridge_interfaces.srv.GetObjectPoint not found. "
                "Build the interface package and source install/setup.bash to enable the service."
            )
        else:
            self.point_service = self.create_service(
                GetObjectPoint,
                self.point_service_name,
                self._handle_get_object_point,
            )

        self.total_frames = 0
        self.window_frames = 0
        self.window_started = time.monotonic()
        self.last_point_log_time = 0.0

        self.get_logger().info(
            f"Windows stream bridge ready\n"
            f"  listen: {self.listen_host}:{self.listen_port}\n"
            f"  publish: {self.publish_topic}\n"
            f"  point: {self.point_topic}\n"
            f"  point_service: {self.point_service_name}"
            f"{' (disabled, interfaces not built)' if self.point_service is None else ''}\n"
            f"  frame_id: {self.frame_id}\n"
            f"  control: {self.bridge_control_topic}\n"
            f"  publish_fps_log: {self.enable_publish_fps_log}"
        )

    def bridge_control_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ("fps_log_off", "publish_fps_off", "publishing_fps_off"):
            self.enable_publish_fps_log = False
            self.get_logger().info("Publishing FPS log disabled by control command.")
            return

        if command in ("fps_log_on", "publish_fps_on", "publishing_fps_on"):
            self.enable_publish_fps_log = True
            self.get_logger().info("Publishing FPS log enabled by control command.")
            return

        if command in ("fps_log_toggle", "publish_fps_toggle", "publishing_fps_toggle"):
            self.enable_publish_fps_log = not self.enable_publish_fps_log
            self.get_logger().info(
                f"Publishing FPS log toggled to {self.enable_publish_fps_log}."
            )
            return

    def _recv_exact(self, conn: socket.socket, size: int) -> bytes | None:
        chunks: list[bytes] = []
        remaining = size

        while remaining > 0 and rclpy.ok():
            try:
                data = conn.recv(remaining)
            except socket.timeout:
                continue
            except OSError:
                return None

            if not data:
                return None

            chunks.append(data)
            remaining -= len(data)

        if remaining != 0:
            return None

        return b"".join(chunks)

    def _publish_frame(self, frame: np.ndarray) -> None:
        msg = cv_frame_to_image_msg(
            frame=frame,
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id,
            encoding="bgr8",
        )
        self.publisher.publish(msg)

        self.total_frames += 1
        self.window_frames += 1

        now = time.monotonic()
        elapsed = now - self.window_started
        if self.enable_publish_fps_log and elapsed >= 1.0:
            fps = self.window_frames / elapsed
            h, w = frame.shape[:2]
            self.get_logger().info(
                f"Publishing {w}x{h} on {self.publish_topic} fps={fps:.2f} total={self.total_frames}"
            )
            self.window_started = now
            self.window_frames = 0
        elif elapsed >= 1.0:
            self.window_started = now
            self.window_frames = 0

    def _publish_point_from_pixel(
        self,
        x_px: int,
        y_px: int,
        width: int,
        height: int,
        source: str = "click",
    ) -> None:
        if width <= 0 or height <= 0:
            return

        x_px = int(min(max(x_px, 0), width - 1))
        y_px = int(min(max(y_px, 0), height - 1))

        x_m = (float(x_px) - (width / 2.0)) / self.scale_px_per_meter
        y_m = ((height / 2.0) - float(y_px)) / self.scale_px_per_meter

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.frame_id
        point_msg.point.x = float(x_m)
        point_msg.point.y = float(y_m)
        point_msg.point.z = float(self.default_z_m)
        self.point_publisher.publish(point_msg)

        with self._latest_point_lock:
            self._latest_point = point_msg
            self._latest_point_source = source

        now = time.monotonic()
        if now - self.last_point_log_time >= 0.5:
            self.last_point_log_time = now
            self.get_logger().info(
                f"Published {source} point px=({x_px},{y_px}) -> "
                f"(X y z) ({x_m:.1f} {y_m:.1f} {self.default_z_m:.1f}) on {self.point_topic}"
            )

    def _handle_get_object_point(self, request, response):  # noqa: ANN001
        with self._latest_point_lock:
            cached = self._latest_point
            source = self._latest_point_source

        if cached is None:
            response.success = False
            response.message = "no point detected yet"
            response.source = ""
            response.point = PointStamped()
            response.point.header.frame_id = self.frame_id
            return response

        response.success = True
        response.message = "ok"
        response.source = source
        response.point = cached
        return response

    def _handle_control_payload(self, payload: bytes) -> None:
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Invalid control payload: {exc}")
            return

        msg_type = str(message.get("type", "")).strip().lower()
        if msg_type != "click_point_px":
            self.get_logger().warning(f"Unsupported control message type: {msg_type or '<empty>'}")
            return

        try:
            x_px = int(message["x"])
            y_px = int(message["y"])
            width = int(message["width"])
            height = int(message["height"])
            source = str(message.get("source", "click")).strip().lower() or "click"
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid click payload fields: {exc}")
            return

        self._publish_point_from_pixel(
            x_px=x_px,
            y_px=y_px,
            width=width,
            height=height,
            source=source,
        )

    def _handle_client(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        conn.settimeout(1.0)
        self.get_logger().info(f"Client connected: {addr[0]}:{addr[1]}")

        while rclpy.ok():
            header = self._recv_exact(conn, 4)
            if header is None:
                break

            raw_len = struct.unpack("!I", header)[0]
            is_control = bool(raw_len & 0x80000000)
            payload_len = raw_len & 0x7FFFFFFF
            if payload_len <= 0:
                self.get_logger().warning("Invalid packet length=0. Closing client.")
                break

            if is_control:
                if payload_len > self.max_control_bytes:
                    self.get_logger().warning(
                        f"Control packet too large={payload_len}, max={self.max_control_bytes}. "
                        "Closing client."
                    )
                    break
            else:
                if payload_len > self.max_frame_bytes:
                    self.get_logger().warning(
                        f"Invalid frame length={payload_len}, max={self.max_frame_bytes}. Closing client."
                    )
                    break

            payload = self._recv_exact(conn, payload_len)
            if payload is None:
                break

            if is_control:
                self._handle_control_payload(payload)
                continue

            encoded = np.frombuffer(payload, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning("Failed to decode JPEG frame from sender.")
                continue

            self._publish_frame(frame)

        self.get_logger().info(f"Client disconnected: {addr[0]}:{addr[1]}")

    def serve(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.listen_host, int(self.listen_port)))
        server.listen(1)
        server.settimeout(1.0)

        self.get_logger().info("Waiting for Windows sender connection...")

        try:
            while rclpy.ok():
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    self.get_logger().error(f"Accept failed: {exc}")
                    continue

                with conn:
                    self._handle_client(conn, addr)
        finally:
            try:
                server.close()
            except OSError:
                pass


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WindowsStreamBridgePublisher()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.serve()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by keyboard interrupt.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
