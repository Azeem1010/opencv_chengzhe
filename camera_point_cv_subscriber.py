#!/usr/bin/env python3
"""ROS 2 node that displays camera images and overlays PointStamped data."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_image_codec import image_msg_to_bgr8
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import tf2_geometry_msgs
    import tf2_ros
except Exception:  # noqa: BLE001
    tf2_geometry_msgs = None
    tf2_ros = None

# 嘗試匯入升級後的 GetObjectPoint.srv (current_position + target_position)
try:
    from opencv_ros2_bridge_interfaces.srv import GetObjectPoint
except ImportError:
    GetObjectPoint = None


class CameraPointCvSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("camera_point_cv_subscriber")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("point_topic", "/camera/object_point")
        self.declare_parameter("show_point_overlay", True)
        self.declare_parameter("enable_click_scan", True)
        self.declare_parameter("scale_px_per_meter", 100.0)
        self.declare_parameter("default_z_m", 0.0)
        self.declare_parameter("output_frame_mode", "camera")
        self.declare_parameter("arm_frame_id", "arm_base_link")
        self.declare_parameter("use_tf_for_arm_output", True)
        self.declare_parameter("xyz_output_topic", "/camera/object_xyz")
        self.declare_parameter("xyz_output_decimals", 1)
        self.declare_parameter("tx_topic", "/camera/tx")
        self.declare_parameter("stop_topic", "/camera/stop")
        self.declare_parameter("rx_topic", "/camera/rx")
        self.declare_parameter("bridge_control_topic", "/camera/bridge_control")
        self.declare_parameter("show_debug_metrics", True)
        self.declare_parameter("processing_mode", "none")
        self.declare_parameter("canny_low", 80)
        self.declare_parameter("canny_high", 160)
        self.declare_parameter("min_target_area", 350.0)
        self.declare_parameter("auto_publish_detected_point", False)
        self.declare_parameter("auto_publish_hz", 8.0)
        self.declare_parameter("render_hz", 60.0)

        # --- Pick-and-Place 模式新增參數 ---
        # 服務名稱：升級後回傳 (current_position, target_position) 的 ROS Service
        self.declare_parameter("pick_place_service", "/camera/get_object_point")
        # 黑色方塊二值化的亮度閾值 (V 通道 <= 此值才會被視為黑色)
        self.declare_parameter("black_v_max", 70)
        # 黑色方塊允許的飽和度上限 (避免把暗紅、暗藍誤判為黑)
        self.declare_parameter("black_s_max", 80)
        # 黑色方塊最小面積（像素²），用來濾掉雜點
        self.declare_parameter("black_min_area", 600.0)
        # 黑色方塊最大面積，用來避免抓到整片陰影/邊框
        self.declare_parameter("black_max_area", 60000.0)
        # 中央區域比例：方塊中心必須落在 (image_w * r, image_h * r) 的中央矩形內
        self.declare_parameter("center_region_ratio", 0.7)
        # 橘色 HSV 範圍 (OpenCV: H 0~179)
        self.declare_parameter("orange_h_min", 5)
        self.declare_parameter("orange_h_max", 22)
        self.declare_parameter("orange_s_min", 120)
        self.declare_parameter("orange_v_min", 120)
        # 橘色圓最小面積
        self.declare_parameter("orange_min_area", 200.0)
        # 圓形度門檻 (4πA/P² 接近 1 才算圓)
        self.declare_parameter("orange_min_circularity", 0.6)
        # 比對半徑 (像素)：若黑方塊中心半徑內有橘圓 -> 該方塊視為「已被佔據」
        # 若給負值，會自動使用「黑方塊邊長的一半」做為動態半徑
        self.declare_parameter("pair_match_radius_px", -1.0)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.point_topic = self.get_parameter("point_topic").get_parameter_value().string_value
        self.show_point_overlay = (
            self.get_parameter("show_point_overlay").get_parameter_value().bool_value
        )
        self.enable_click_scan = (
            self.get_parameter("enable_click_scan").get_parameter_value().bool_value
        )
        self.scale_px_per_meter = (
            self.get_parameter("scale_px_per_meter").get_parameter_value().double_value
        )
        self.default_z_m = self.get_parameter("default_z_m").get_parameter_value().double_value
        self.output_frame_mode = self._normalize_output_mode(
            self.get_parameter("output_frame_mode").get_parameter_value().string_value
        )
        self.arm_frame_id = self.get_parameter("arm_frame_id").get_parameter_value().string_value
        self.use_tf_for_arm_output = (
            self.get_parameter("use_tf_for_arm_output").get_parameter_value().bool_value
        )
        self.xyz_output_topic = (
            self.get_parameter("xyz_output_topic").get_parameter_value().string_value
        )
        self.xyz_output_decimals = (
            self.get_parameter("xyz_output_decimals").get_parameter_value().integer_value
        )
        self.tx_topic = self.get_parameter("tx_topic").get_parameter_value().string_value
        self.stop_topic = self.get_parameter("stop_topic").get_parameter_value().string_value
        self.rx_topic = self.get_parameter("rx_topic").get_parameter_value().string_value
        self.bridge_control_topic = (
            self.get_parameter("bridge_control_topic").get_parameter_value().string_value
        )
        self.show_debug_metrics = (
            self.get_parameter("show_debug_metrics").get_parameter_value().bool_value
        )
        self.processing_mode = (
            self.get_parameter("processing_mode").get_parameter_value().string_value.lower().strip()
        )
        self.canny_low = self.get_parameter("canny_low").get_parameter_value().integer_value
        self.canny_high = self.get_parameter("canny_high").get_parameter_value().integer_value
        self.min_target_area = (
            self.get_parameter("min_target_area").get_parameter_value().double_value
        )
        self.auto_publish_detected_point = (
            self.get_parameter("auto_publish_detected_point").get_parameter_value().bool_value
        )
        self.auto_publish_hz = (
            self.get_parameter("auto_publish_hz").get_parameter_value().double_value
        )
        self.render_hz = self.get_parameter("render_hz").get_parameter_value().double_value

        # --- Pick-and-Place 模式參數讀取 ---
        self.pick_place_service_name = (
            self.get_parameter("pick_place_service").get_parameter_value().string_value
        )
        self.black_v_max = int(self.get_parameter("black_v_max").get_parameter_value().integer_value)
        self.black_s_max = int(self.get_parameter("black_s_max").get_parameter_value().integer_value)
        self.black_min_area = float(
            self.get_parameter("black_min_area").get_parameter_value().double_value
        )
        self.black_max_area = float(
            self.get_parameter("black_max_area").get_parameter_value().double_value
        )
        self.center_region_ratio = float(
            self.get_parameter("center_region_ratio").get_parameter_value().double_value
        )
        self.orange_h_min = int(self.get_parameter("orange_h_min").get_parameter_value().integer_value)
        self.orange_h_max = int(self.get_parameter("orange_h_max").get_parameter_value().integer_value)
        self.orange_s_min = int(self.get_parameter("orange_s_min").get_parameter_value().integer_value)
        self.orange_v_min = int(self.get_parameter("orange_v_min").get_parameter_value().integer_value)
        self.orange_min_area = float(
            self.get_parameter("orange_min_area").get_parameter_value().double_value
        )
        self.orange_min_circularity = float(
            self.get_parameter("orange_min_circularity").get_parameter_value().double_value
        )
        self.pair_match_radius_px = float(
            self.get_parameter("pair_match_radius_px").get_parameter_value().double_value
        )

        self.window_name = "Camera Viewer (q:quit d:debug s:stop)"
        self.width = 960
        self.height = 720

        if self.scale_px_per_meter <= 0.0:
            self.scale_px_per_meter = 100.0
        if self.canny_low < 0:
            self.canny_low = 0
        if self.canny_high <= self.canny_low:
            self.canny_high = self.canny_low + 1
        if self.min_target_area <= 0.0:
            self.min_target_area = 350.0
        if self.auto_publish_hz <= 0.0:
            self.auto_publish_hz = 8.0
        if self.render_hz <= 0.0:
            self.render_hz = 60.0
        if self.xyz_output_decimals < 0:
            self.xyz_output_decimals = 0
        if self.xyz_output_decimals > 6:
            self.xyz_output_decimals = 6

        self.tf_buffer = None
        self.tf_listener = None
        self.last_tf_warn_time = self.get_clock().now()
        if self.output_frame_mode == "arm" and self.use_tf_for_arm_output:
            if tf2_ros is None or tf2_geometry_msgs is None:
                self.get_logger().warning(
                    "TF modules are unavailable. Output falls back to camera frame."
                )
            else:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_image: np.ndarray | None = None
        self.latest_image_msg: Image | None = None
        self.latest_point_msg: PointStamped | None = None
        self.last_clicked_pixel: tuple[int, int] | None = None
        self.last_detected_pixel: tuple[int, int] | None = None
        self.last_detected_area: float = 0.0

        # --- Pick-and-Place 最新偵測結果 (供 Service 取用) ---
        # 用 lock 保護：CV 處理在 image_callback 執行緒，service 在 rclpy 執行緒
        self._pp_lock = threading.Lock()
        self._pp_black_centers: list[tuple[int, int]] = []   # 4 個黑方塊中心
        self._pp_orange_centers: list[tuple[int, int]] = []  # 偵測到的橘色圓中心
        self._pp_empty_black_centers: list[tuple[int, int]] = []  # 空的黑方塊中心
        self._pp_current_pixel: tuple[int, int] | None = None     # 挑出的橘圓
        self._pp_target_pixel: tuple[int, int] | None = None      # 挑出的空黑方塊
        self._pp_match_radius_used: float = 0.0
        self._pp_last_message: str = "no detection yet"
        self.processing_mode_warned = False
        self.last_auto_publish_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.last_tx_message = "TX: N/A"
        self.last_rx_message = "RX: N/A"
        self.publish_fps_log_enabled = True

        now_mono = time.monotonic()
        self.image_hz_measured = 0.0
        self.point_hz_measured = 0.0
        self.render_fps_measured = 0.0
        self._image_window_started = now_mono
        self._point_window_started = now_mono
        self._render_window_started = now_mono
        self._image_count = 0
        self._point_count = 0
        self._render_count = 0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )
        self.point_subscription = self.create_subscription(
            PointStamped,
            self.point_topic,
            self.point_callback,
            10,
        )
        self.rx_subscription = self.create_subscription(
            String,
            self.rx_topic,
            self.rx_callback,
            10,
        )

        self.point_publisher = self.create_publisher(PointStamped, self.point_topic, 10)
        self.xyz_publisher = self.create_publisher(String, self.xyz_output_topic, 10)
        self.tx_publisher = self.create_publisher(String, self.tx_topic, 10)
        self.stop_publisher = self.create_publisher(String, self.stop_topic, 10)
        self.bridge_control_publisher = self.create_publisher(
            String,
            self.bridge_control_topic,
            10,
        )

        # --- 註冊升級後的 Pick-and-Place Service Server ---
        # Service 回傳兩組 2D 座標 (current_position, target_position) + success 旗標
        if GetObjectPoint is None:
            self.pick_place_service = None
            self.get_logger().warning(
                "opencv_ros2_bridge_interfaces.srv.GetObjectPoint not found; "
                "pick-and-place service disabled. Rebuild interfaces and source install/setup.bash."
            )
        else:
            self.pick_place_service = self.create_service(
                GetObjectPoint,
                self.pick_place_service_name,
                self._handle_get_object_point,
            )
            self.get_logger().info(
                f"Pick-and-place service ready at {self.pick_place_service_name}"
            )

        self.timer = self.create_timer(1.0 / self.render_hz, self.render_frame)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        cv2.setMouseCallback(self.window_name, self.on_mouse_event)

        self.get_logger().info(f"Subscribed image topic: {self.image_topic}")
        self.get_logger().info(f"Subscribed point topic: {self.point_topic}")
        self.get_logger().info(f"XYZ output topic: {self.xyz_output_topic}")
        self.get_logger().info(f"TX topic: {self.tx_topic}, STOP topic: {self.stop_topic}")
        self.get_logger().info(f"RX topic: {self.rx_topic}")
        self.get_logger().info(f"Bridge control topic: {self.bridge_control_topic}")
        self.get_logger().info(f"Render refresh rate: {self.render_hz:.2f} Hz")
        self.get_logger().info(f"Scale setting: {self.scale_px_per_meter:.1f} px/m (1:100cm)")
        self.get_logger().info(
            f"Output frame mode: {self.output_frame_mode} "
            f"(arm frame: {self.arm_frame_id})"
        )
        self.get_logger().info(f"Processing mode: {self.processing_mode}")
        if self.enable_click_scan:
            self.get_logger().info(
                "Click-scan enabled: left-click image to publish /camera/object_point."
            )
        if self.auto_publish_detected_point:
            self.get_logger().info(
                f"Auto point publish enabled at <= {self.auto_publish_hz:.2f} Hz"
            )
        self.get_logger().info("Press 'q' quit, 'd' debug, 'f' fps-log toggle, 's' STOP.")

    @staticmethod
    def _normalize_output_mode(raw_mode: str) -> str:
        mode = str(raw_mode).strip().lower()
        if mode in ("arm", "arm000", "robot", "robot_arm"):
            return "arm"
        return "camera"

    def _update_image_rate(self) -> None:
        self._image_count += 1
        now = time.monotonic()
        elapsed = now - self._image_window_started
        if elapsed >= 1.0:
            self.image_hz_measured = self._image_count / elapsed
            self._image_count = 0
            self._image_window_started = now

    def _update_point_rate(self) -> None:
        self._point_count += 1
        now = time.monotonic()
        elapsed = now - self._point_window_started
        if elapsed >= 1.0:
            self.point_hz_measured = self._point_count / elapsed
            self._point_count = 0
            self._point_window_started = now

    def _update_render_rate(self) -> None:
        self._render_count += 1
        now = time.monotonic()
        elapsed = now - self._render_window_started
        if elapsed >= 1.0:
            self.render_fps_measured = self._render_count / elapsed
            self._render_count = 0
            self._render_window_started = now

    def image_callback(self, msg: Image) -> None:
        self._update_image_rate()

        try:
            raw_frame = image_msg_to_bgr8(msg)
            processed_frame = self._process_frame(raw_frame)
            self.latest_image = processed_frame
            self.latest_image_msg = msg
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to convert image message: {exc}")

    def point_callback(self, msg: PointStamped) -> None:
        self._update_point_rate()

        point_for_output = self._transform_for_output(msg)
        self.latest_point_msg = point_for_output
        self._publish_xyz_output(point_for_output)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.last_log_time = now
            self.get_logger().info(
                f"Point frame={point_for_output.header.frame_id or 'unknown'} "
                f"{self._format_xyz(point_for_output)}"
            )

    def rx_callback(self, msg: String) -> None:
        self.last_rx_message = f"RX: {msg.data.strip() or '<empty>'}"
        self.get_logger().info(self.last_rx_message)

    @staticmethod
    def _debug_button_rect(frame_w: int) -> tuple[int, int, int, int]:
        x2 = max(140, frame_w - 12)
        x1 = max(10, x2 - 160)
        y1 = 10
        y2 = 44
        return x1, y1, x2, y2

    @staticmethod
    def _fps_log_button_rect(frame_w: int) -> tuple[int, int, int, int]:
        x2 = max(332, frame_w - 184)
        x1 = max(170, x2 - 190)
        y1 = 10
        y2 = 44
        return x1, y1, x2, y2

    def _publish_bridge_fps_log_state(self) -> None:
        command = "fps_log_on" if self.publish_fps_log_enabled else "fps_log_off"
        msg = String()
        msg.data = command
        self.bridge_control_publisher.publish(msg)

        self.last_tx_message = f"TX {command.upper()}"
        self.get_logger().info(
            f"Bridge publishing-fps log -> {'ON' if self.publish_fps_log_enabled else 'OFF'}"
        )

    def _toggle_bridge_fps_log_from_click(self, x: int, y: int) -> bool:
        frame_w = self.latest_image.shape[1] if self.latest_image is not None else self.width
        x1, y1, x2, y2 = self._fps_log_button_rect(frame_w)
        if x1 <= x <= x2 and y1 <= y <= y2:
            self.publish_fps_log_enabled = not self.publish_fps_log_enabled
            self._publish_bridge_fps_log_state()
            return True
        return False

    def _toggle_debug_from_click(self, x: int, y: int) -> bool:
        frame_w = self.latest_image.shape[1] if self.latest_image is not None else self.width
        x1, y1, x2, y2 = self._debug_button_rect(frame_w)
        if x1 <= x <= x2 and y1 <= y <= y2:
            self.show_debug_metrics = not self.show_debug_metrics
            state = "ON" if self.show_debug_metrics else "OFF"
            self.get_logger().info(f"Debug metrics toggled: {state}")
            return True
        return False

    def on_mouse_event(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        del flags, param

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self._toggle_bridge_fps_log_from_click(int(x), int(y)):
            return

        if self._toggle_debug_from_click(int(x), int(y)):
            return

        if not self.enable_click_scan:
            return

        if self.latest_image is None:
            return

        self.last_clicked_pixel = (int(x), int(y))
        self._publish_point_from_pixel(int(x), int(y), source="click")

    def _point_from_pixel(self, x: int, y: int) -> PointStamped:
        h, w = self.latest_image.shape[:2]
        x_m = (float(x) - (w / 2.0)) / self.scale_px_per_meter
        y_m = ((h / 2.0) - float(y)) / self.scale_px_per_meter

        frame_id = "camera_frame"
        if self.latest_image_msg is not None and self.latest_image_msg.header.frame_id:
            frame_id = self.latest_image_msg.header.frame_id

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = frame_id
        point_msg.point.x = float(x_m)
        point_msg.point.y = float(y_m)
        point_msg.point.z = float(self.default_z_m)
        return point_msg

    def _pixel_to_xy_meters(self, x_px: int, y_px: int) -> tuple[float, float]:
        """把影像 pixel 換成節點輸出座標系下的 (x, y) 公尺值。
        與 _point_from_pixel 使用相同公式：影像中心為原點、y 軸朝上。
        """
        if self.latest_image is None:
            return 0.0, 0.0
        h, w = self.latest_image.shape[:2]
        x_m = (float(x_px) - (w / 2.0)) / self.scale_px_per_meter
        y_m = ((h / 2.0) - float(y_px)) / self.scale_px_per_meter
        return x_m, y_m

    def _handle_get_object_point(self, request, response):  # noqa: ANN001
        """ROS Service 回呼：回傳 (current_position, target_position)。
        - 找不到橘色圓形 -> success=False
        - 4 個黑方塊都被佔據 -> success=False
        - 其餘情況 -> success=True 並回傳兩組 2D 座標 (z=0)
        """
        del request  # GetObjectPoint.srv 請求無欄位
        with self._pp_lock:
            current_pixel = self._pp_current_pixel
            target_pixel = self._pp_target_pixel
            message = self._pp_last_message
            n_orange = len(self._pp_orange_centers)
            n_empty = len(self._pp_empty_black_centers)

        response.current_position = Point()
        response.target_position = Point()

        if current_pixel is None or target_pixel is None:
            response.success = False
            if n_orange == 0:
                response.message = "no orange circle detected"
            elif n_empty == 0:
                response.message = "all black squares are occupied"
            else:
                response.message = message or "no valid pair"
            return response

        cx_m, cy_m = self._pixel_to_xy_meters(*current_pixel)
        tx_m, ty_m = self._pixel_to_xy_meters(*target_pixel)
        response.current_position.x = float(cx_m)
        response.current_position.y = float(cy_m)
        response.current_position.z = 0.0
        response.target_position.x = float(tx_m)
        response.target_position.y = float(ty_m)
        response.target_position.z = 0.0
        response.success = True
        response.message = "ok"
        return response

    def _transform_for_output(self, msg: PointStamped) -> PointStamped:
        if self.output_frame_mode != "arm":
            return msg

        if not self.use_tf_for_arm_output or self.tf_buffer is None or tf2_geometry_msgs is None:
            return msg

        src_frame = msg.header.frame_id or ""
        if not src_frame or src_frame == self.arm_frame_id:
            return msg

        try:
            transform = self.tf_buffer.lookup_transform(
                self.arm_frame_id,
                src_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return tf2_geometry_msgs.do_transform_point(msg, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time).nanoseconds > 1_000_000_000:
                self.last_tf_warn_time = now
                self.get_logger().warning(
                    f"TF transform failed ({src_frame} -> {self.arm_frame_id}), "
                    f"using source frame: {exc}"
                )
            return msg

    def _format_xyz(self, point_msg: PointStamped) -> str:
        value_fmt = f"{{:.{int(self.xyz_output_decimals)}f}}"
        p = point_msg.point
        return (
            f"(X y z) ({value_fmt.format(float(p.x))} "
            f"{value_fmt.format(float(p.y))} "
            f"{value_fmt.format(float(p.z))})"
        )

    def _publish_xyz_output(self, point_msg: PointStamped) -> None:
        msg = String()
        msg.data = self._format_xyz(point_msg)
        self.xyz_publisher.publish(msg)

    def _publish_tx_target(self, point_msg: PointStamped, source: str) -> None:
        tx_msg = String()
        tx_msg.data = (
            f"TX TARGET source={source} frame={point_msg.header.frame_id or 'unknown'} "
            f"{self._format_xyz(point_msg)}"
        )
        self.tx_publisher.publish(tx_msg)
        self.last_tx_message = tx_msg.data

    def _publish_stop_signal(self) -> None:
        stop_msg = String()
        stop_msg.data = "STOP"
        self.stop_publisher.publish(stop_msg)

        tx_msg = String()
        tx_msg.data = "TX STOP"
        self.tx_publisher.publish(tx_msg)
        self.last_tx_message = tx_msg.data

        self.get_logger().info(
            f"Published STOP on {self.stop_topic} and TX STOP on {self.tx_topic}"
        )

    def _publish_point_from_pixel(self, x: int, y: int, source: str) -> None:
        if self.latest_image is None:
            return

        if source == "auto":
            dt_ns = (self.get_clock().now() - self.last_auto_publish_time).nanoseconds
            min_interval_ns = int((1.0 / self.auto_publish_hz) * 1e9)
            if dt_ns < min_interval_ns:
                return

        point_msg = self._point_from_pixel(x, y)
        point_msg = self._transform_for_output(point_msg)
        self.point_publisher.publish(point_msg)
        self.latest_point_msg = point_msg

        self._publish_tx_target(point_msg, source=source)

        if source == "auto":
            self.last_auto_publish_time = self.get_clock().now()
            return

        self.get_logger().info(
            f"Clicked pixel=({x},{y}) frame={point_msg.header.frame_id or 'unknown'} "
            f"{self._format_xyz(point_msg)}"
        )

    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        mode = self.processing_mode

        if mode in ("", "none"):
            self.last_detected_pixel = None
            self.last_detected_area = 0.0
            return frame

        if mode == "gray":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.last_detected_pixel = None
            self.last_detected_area = 0.0
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if mode == "canny":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, int(self.canny_low), int(self.canny_high))
            self.last_detected_pixel = None
            self.last_detected_area = 0.0
            return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if mode in ("red-target", "red_track", "red"):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_red_1 = np.array([0, 100, 80], dtype=np.uint8)
            upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
            lower_red_2 = np.array([170, 100, 80], dtype=np.uint8)
            upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)

            mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
            mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
            mask = cv2.bitwise_or(mask_1, mask_2)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.last_detected_pixel = None
            self.last_detected_area = 0.0

            if contours:
                contour = max(contours, key=cv2.contourArea)
                area = float(cv2.contourArea(contour))
                if area >= self.min_target_area:
                    moments = cv2.moments(contour)
                    if moments["m00"] > 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                        self.last_detected_pixel = (cx, cy)
                        self.last_detected_area = area

                        cv2.drawContours(frame, [contour], -1, (0, 255, 255), 2)
                        cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                        cv2.putText(
                            frame,
                            f"target area={area:.0f}",
                            (max(12, cx - 80), max(20, cy - 16)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

                        if self.auto_publish_detected_point:
                            self._publish_point_from_pixel(cx, cy, source="auto")

            return frame

        # =========================================================
        # 新增模式：pick-and-place
        # 場景：畫面中央有 4 個黑色實心正方形排成 2x2 (田字)，
        #      畫面其他位置散佈 4 個橘色實心圓形。
        # 目標：找出「空的黑方塊」與「可夾取的橘圓」，
        #      並把結果存到 self._pp_* 供 ROS Service 取用。
        # =========================================================
        if mode in ("pick-and-place", "pick_and_place", "pnp"):
            return self._process_pick_and_place(frame)

        if not self.processing_mode_warned:
            self.processing_mode_warned = True
            self.get_logger().warning(
                f"Unknown processing_mode='{self.processing_mode}'. Falling back to raw image."
            )

        self.last_detected_pixel = None
        self.last_detected_area = 0.0
        return frame

    # =========================================================
    # Pick-and-Place 視覺處理核心
    # 步驟：
    #   1. 轉 HSV
    #   2. 用 V/S 通道做黑色遮罩 -> findContours -> 過濾為「中央區域的方形」
    #   3. 用 HSV 範圍做橘色遮罩 -> findContours -> 過濾為「夠圓的形狀」
    #   4. 比對每個黑方塊中心半徑內是否有橘圓，找出空方塊
    #   5. 從橘圓挑 current_position、從空方塊挑 target_position
    #   6. 在畫面上標註結果並寫回 self._pp_* (受 lock 保護)
    # =========================================================
    def _process_pick_and_place(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---- (1) 黑色方塊偵測 ----
        # 黑色 = 低亮度 (V 小) 且飽和度不高 (S 小)，避免暗色但有彩度的物體被誤判
        black_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 0], dtype=np.uint8),
            np.array([179, self.black_s_max, self.black_v_max], dtype=np.uint8),
        )
        # 形態學去雜訊 + 填補小洞，避免方塊邊緣斷裂
        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        black_candidates: list[dict] = []
        contours_b, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 中央區域矩形 (僅取中心落在此範圍內的方塊，避免抓到邊框/暗影)
        cx_min = int(w * (0.5 - self.center_region_ratio / 2.0))
        cx_max = int(w * (0.5 + self.center_region_ratio / 2.0))
        cy_min = int(h * (0.5 - self.center_region_ratio / 2.0))
        cy_max = int(h * (0.5 + self.center_region_ratio / 2.0))

        for cnt in contours_b:
            area = float(cv2.contourArea(cnt))
            if area < self.black_min_area or area > self.black_max_area:
                continue

            # 用最小外接矩形判斷「是否為正方形」
            rect = cv2.minAreaRect(cnt)
            (rcx, rcy), (rw, rh), _ = rect
            if rw <= 1 or rh <= 1:
                continue
            aspect = max(rw, rh) / min(rw, rh)
            if aspect > 1.35:  # 長寬比過大 -> 不是正方形 (例如桌邊、長條陰影)
                continue

            # 矩形填滿率：實際面積 / 外接矩形面積，正方形應該 > 0.75
            extent = area / float(rw * rh)
            if extent < 0.75:
                continue

            cx, cy = int(rcx), int(rcy)
            if not (cx_min <= cx <= cx_max and cy_min <= cy <= cy_max):
                continue  # 不在中央區域

            side = float((rw + rh) / 2.0)
            black_candidates.append({
                "center": (cx, cy),
                "side": side,
                "area": area,
                "contour": cnt,
            })

        # 中央田字最多 4 塊；若偵測到 >4，取「距畫面中心最近」的 4 個
        img_cx, img_cy = w // 2, h // 2
        black_candidates.sort(
            key=lambda b: (b["center"][0] - img_cx) ** 2 + (b["center"][1] - img_cy) ** 2
        )
        black_squares = black_candidates[:4]

        # ---- (2) 橘色圓形偵測 ----
        orange_mask = cv2.inRange(
            hsv,
            np.array([self.orange_h_min, self.orange_s_min, self.orange_v_min], dtype=np.uint8),
            np.array([self.orange_h_max, 255, 255], dtype=np.uint8),
        )
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)

        orange_circles: list[dict] = []
        contours_o, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_o:
            area = float(cv2.contourArea(cnt))
            if area < self.orange_min_area:
                continue
            perim = float(cv2.arcLength(cnt, True))
            if perim <= 0:
                continue
            circularity = 4.0 * np.pi * area / (perim * perim)
            if circularity < self.orange_min_circularity:
                continue  # 非圓形 -> 排除 (例如雜訊、橘色長物)

            (rcx, rcy), radius = cv2.minEnclosingCircle(cnt)
            orange_circles.append({
                "center": (int(rcx), int(rcy)),
                "radius": float(radius),
                "area": area,
                "contour": cnt,
            })

        # 場景最多 4 個橘圓；若雜點多，挑「面積最大」的前 4 個
        orange_circles.sort(key=lambda o: o["area"], reverse=True)
        orange_circles = orange_circles[:4]

        # ---- (3) 配對：哪些黑方塊是空的？ ----
        # 比對半徑：若使用者沒指定，動態用「方塊邊長一半」當半徑
        empty_squares: list[dict] = []
        match_radius_used = 0.0
        for sq in black_squares:
            if self.pair_match_radius_px > 0:
                match_radius = self.pair_match_radius_px
            else:
                match_radius = sq["side"] * 0.5
            match_radius_used = match_radius

            sx, sy = sq["center"]
            occupied = False
            for oc in orange_circles:
                ox, oy = oc["center"]
                if (ox - sx) ** 2 + (oy - sy) ** 2 <= match_radius ** 2:
                    occupied = True
                    break
            if not occupied:
                empty_squares.append(sq)

        # ---- (4) 挑選 current_position 與 target_position ----
        # 策略：
        #   current_position = 距畫面中心最遠的橘圓 (通常是散落在外面待夾取的物體)
        #   target_position  = 第一個空黑方塊 (順序由「距畫面中心由近到遠」決定)
        current_pixel: tuple[int, int] | None = None
        target_pixel: tuple[int, int] | None = None
        message = ""

        if not orange_circles:
            message = "no orange circle detected"
        elif not empty_squares:
            message = "all black squares are occupied"
        else:
            # current: 取距畫面中心最遠的橘圓 (代表它還沒被擺進田字中)
            current_pick = max(
                orange_circles,
                key=lambda o: (o["center"][0] - img_cx) ** 2 + (o["center"][1] - img_cy) ** 2,
            )
            current_pixel = current_pick["center"]
            # target: 取距畫面中心最近的空方塊
            target_pick = min(
                empty_squares,
                key=lambda s: (s["center"][0] - img_cx) ** 2 + (s["center"][1] - img_cy) ** 2,
            )
            target_pixel = target_pick["center"]
            message = "ok"

        # ---- (5) 把結果寫回節點狀態 (供 Service 讀取) ----
        with self._pp_lock:
            self._pp_black_centers = [b["center"] for b in black_squares]
            self._pp_orange_centers = [o["center"] for o in orange_circles]
            self._pp_empty_black_centers = [b["center"] for b in empty_squares]
            self._pp_current_pixel = current_pixel
            self._pp_target_pixel = target_pixel
            self._pp_match_radius_used = float(match_radius_used)
            self._pp_last_message = message

        # 也同步更新「單點偵測」的舊欄位，讓既有 overlay/auto publish 仍可用
        self.last_detected_pixel = current_pixel
        self.last_detected_area = 0.0

        # ---- (6) 視覺化標註 ----
        # 公尺換算公式與 _pixel_to_xy_meters / service 回傳值完全一致：
        #   x_m = (px - w/2) / scale ; y_m = (h/2 - py) / scale
        def _to_meters(px: int, py: int) -> tuple[float, float]:
            return (
                (float(px) - (w / 2.0)) / self.scale_px_per_meter,
                ((h / 2.0) - float(py)) / self.scale_px_per_meter,
            )

        # 黑方塊：白色輪廓；空的多加綠色圈、被佔據的多加紅色圈
        # 標註：編號 B# + 狀態 + 像素座標 + 公尺座標
        for idx, sq in enumerate(black_squares):
            cv2.drawContours(frame, [sq["contour"]], -1, (255, 255, 255), 2)
            sx, sy = sq["center"]
            is_empty = sq in empty_squares
            ring_color = (0, 255, 0) if is_empty else (0, 0, 255)
            cv2.circle(frame, (sx, sy), int(sq["side"] * 0.5), ring_color, 2)
            sx_m, sy_m = _to_meters(sx, sy)
            half = int(sq["side"] * 0.5)
            cv2.putText(
                frame, f"B{idx + 1} {'EMPTY' if is_empty else 'FULL'}",
                (sx - 30, sy - half - 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, ring_color, 2, cv2.LINE_AA,
            )
            cv2.putText(
                frame, f"px=({sx},{sy})", (sx - 30, sy - half - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA,
            )
            cv2.putText(
                frame, f"m=({sx_m:.2f},{sy_m:.2f})", (sx - 30, sy + half + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA,
            )

        # 橘圓：青色輪廓 + 中心點
        # 標註：編號 O# + 像素座標 + 公尺座標
        for idx, oc in enumerate(orange_circles):
            cv2.drawContours(frame, [oc["contour"]], -1, (255, 200, 0), 2)
            ox, oy = oc["center"]
            cv2.circle(frame, (ox, oy), 4, (255, 200, 0), -1)
            ox_m, oy_m = _to_meters(ox, oy)
            r = int(oc["radius"])
            cv2.putText(
                frame, f"O{idx + 1}", (ox - 12, oy - r - 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2, cv2.LINE_AA,
            )
            cv2.putText(
                frame, f"px=({ox},{oy})", (ox - 30, oy - r - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA,
            )
            cv2.putText(
                frame, f"m=({ox_m:.2f},{oy_m:.2f})", (ox - 30, oy + r + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA,
            )

        # 高亮選中的 current / target
        if current_pixel is not None:
            cv2.circle(frame, current_pixel, 14, (0, 165, 255), 3)
            cv2.putText(
                frame, "CURRENT", (current_pixel[0] + 16, current_pixel[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA,
            )
        if target_pixel is not None:
            cv2.circle(frame, target_pixel, 18, (0, 255, 255), 3)
            cv2.putText(
                frame, "TARGET", (target_pixel[0] + 18, target_pixel[1] + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
            )
        # 兩點間連線，方便檢查配對結果
        if current_pixel is not None and target_pixel is not None:
            cv2.arrowedLine(frame, current_pixel, target_pixel, (0, 255, 255), 2, tipLength=0.05)

        # 在畫面左下角顯示 pick-and-place 狀態
        status = (
            f"P&P black={len(black_squares)} orange={len(orange_circles)} "
            f"empty={len(empty_squares)} msg={message}"
        )
        cv2.putText(
            frame, status, (12, h - 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA,
        )

        return frame

    def _draw_waiting_canvas(self) -> np.ndarray:
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        cv2.putText(
            canvas,
            "Waiting for camera image...",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"image topic: {self.image_topic}",
            (30, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (190, 190, 190),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"point topic: {self.point_topic}",
            (30, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (190, 190, 190),
            2,
            cv2.LINE_AA,
        )

        return canvas

    def _draw_point_overlay(self, frame: np.ndarray) -> None:
        if not self.show_point_overlay:
            return

        h, w = frame.shape[:2]
        inset_w = min(300, max(220, w // 3))
        inset_h = min(240, max(180, h // 3))
        margin = 16

        x1 = w - inset_w - margin
        y1 = margin + 40
        x2 = x1 + inset_w
        y2 = y1 + inset_h

        if x1 < 0 or y2 > h:
            return

        roi = frame[y1:y2, x1:x2]
        overlay = roi.copy()
        overlay[:] = (30, 30, 30)
        cv2.addWeighted(overlay, 0.6, roi, 0.4, 0, roi)

        cx = x1 + inset_w // 2
        cy = y1 + inset_h // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (120, 120, 120), 1)
        cv2.line(frame, (x1, cy), (x2, cy), (80, 80, 80), 1)
        cv2.line(frame, (cx, y1), (cx, y2), (80, 80, 80), 1)
        cv2.putText(
            frame,
            "XY (top view)",
            (x1 + 10, y1 + 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )

        if self.latest_point_msg is None:
            cv2.putText(
                frame,
                "No PointStamped",
                (x1 + 10, y1 + inset_h - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )
            return

        p = self.latest_point_msg.point
        px = int(cx + p.x * self.scale_px_per_meter)
        py = int(cy - p.y * self.scale_px_per_meter)
        in_bounds = x1 <= px < x2 and y1 <= py < y2
        color = (0, 220, 0) if in_bounds else (0, 180, 255)

        draw_x = min(max(px, x1), x2 - 1)
        draw_y = min(max(py, y1), y2 - 1)
        cv2.circle(frame, (draw_x, draw_y), 6, color, -1)
        cv2.circle(frame, (draw_x, draw_y), 14, color, 2)

        cv2.putText(
            frame,
            self._format_xyz(self.latest_point_msg),
            (x1 + 10, y2 - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )

    def _draw_header_overlay(self, frame: np.ndarray) -> None:
        lines = [
            f"image topic: {self.image_topic}",
            f"point topic: {self.point_topic}",
            f"output mode: {self.output_frame_mode} (arm={self.arm_frame_id})",
            f"processing: {self.processing_mode}",
            "Press q=quit d=debug f=fps-log s=stop",
        ]

        if self.enable_click_scan:
            lines.append("Left-click image: publish object point")

        if self.latest_image_msg is not None:
            stamp = self.latest_image_msg.header.stamp
            stamp_sec = stamp.sec + stamp.nanosec / 1e9
            frame_id = self.latest_image_msg.header.frame_id or "unknown"
            lines.insert(2, f"image frame_id: {frame_id}  stamp: {stamp_sec:.6f}s")

        for idx, line in enumerate(lines):
            y = 62 + idx * 24
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def _draw_click_marker(self, frame: np.ndarray) -> None:
        if self.last_clicked_pixel is None:
            return

        x, y = self.last_clicked_pixel
        h, w = frame.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return

        color = (0, 220, 255)
        cv2.circle(frame, (x, y), 8, color, 2)
        cv2.line(frame, (x - 12, y), (x + 12, y), color, 1)
        cv2.line(frame, (x, y - 12), (x, y + 12), color, 1)

        label = f"px=({x},{y})"
        cv2.putText(
            frame,
            label,
            (x + 10, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_detected_marker(self, frame: np.ndarray) -> None:
        if self.last_detected_pixel is None:
            return

        x, y = self.last_detected_pixel
        h, w = frame.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return

        color = (0, 0, 255)
        cv2.circle(frame, (x, y), 12, color, 2)
        cv2.putText(
            frame,
            f"detected px=({x},{y})",
            (x + 10, min(h - 8, y + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_debug_button(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        del h
        x1, y1, x2, y2 = self._debug_button_rect(w)

        fill_color = (48, 140, 65) if self.show_debug_metrics else (80, 80, 80)
        cv2.rectangle(frame, (x1, y1), (x2, y2), fill_color, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (220, 220, 220), 1)

        label = "DEBUG ON" if self.show_debug_metrics else "DEBUG OFF"
        cv2.putText(
            frame,
            label,
            (x1 + 12, y1 + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_bridge_fps_button(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        del h
        x1, y1, x2, y2 = self._fps_log_button_rect(w)

        fill_color = (28, 108, 182) if self.publish_fps_log_enabled else (80, 80, 80)
        cv2.rectangle(frame, (x1, y1), (x2, y2), fill_color, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (220, 220, 220), 1)

        label = "PUB FPS ON" if self.publish_fps_log_enabled else "PUB FPS OFF"
        cv2.putText(
            frame,
            label,
            (x1 + 12, y1 + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_debug_overlay(self, frame: np.ndarray) -> None:
        if not self.show_debug_metrics:
            return

        point_frame = "unknown"
        point_xyz = "(X y z) (N/A N/A N/A)"
        if self.latest_point_msg is not None:
            point_frame = self.latest_point_msg.header.frame_id or "unknown"
            point_xyz = self._format_xyz(self.latest_point_msg)

        lines = [
            f"FPS/HZ image={self.image_hz_measured:.1f} point={self.point_hz_measured:.1f} render={self.render_fps_measured:.1f}",
            f"scale_px_per_meter={self.scale_px_per_meter:.1f}  ratio=1:100cm",
            f"bridge publishing-fps-log={'ON' if self.publish_fps_log_enabled else 'OFF'}",
            f"point frame={point_frame}  {point_xyz}",
            self.last_tx_message,
            self.last_rx_message,
        ]

        h, w = frame.shape[:2]
        box_h = 20 + len(lines) * 22
        y1 = max(0, h - box_h - 10)
        x1 = 10
        x2 = min(w - 10, 760)
        y2 = min(h - 10, y1 + box_h)

        roi = frame[y1:y2, x1:x2]
        if roi.size > 0:
            overlay = roi.copy()
            overlay[:] = (16, 16, 16)
            cv2.addWeighted(overlay, 0.55, roi, 0.45, 0, roi)

        for idx, line in enumerate(lines):
            y = y1 + 24 + idx * 22
            cv2.putText(
                frame,
                line,
                (x1 + 10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def render_frame(self) -> None:
        self._update_render_rate()

        if self.latest_image is None:
            frame = self._draw_waiting_canvas()
        else:
            frame = self.latest_image.copy()

        self._draw_bridge_fps_button(frame)
        self._draw_debug_button(frame)
        self._draw_header_overlay(frame)
        self._draw_point_overlay(frame)
        self._draw_click_marker(frame)
        self._draw_detected_marker(frame)
        self._draw_debug_overlay(frame)

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Quit requested from OpenCV window.")
            rclpy.shutdown()
        elif key == ord("d"):
            self.show_debug_metrics = not self.show_debug_metrics
            state = "ON" if self.show_debug_metrics else "OFF"
            self.get_logger().info(f"Debug metrics toggled: {state}")
        elif key == ord("f"):
            self.publish_fps_log_enabled = not self.publish_fps_log_enabled
            self._publish_bridge_fps_log_state()
        elif key == ord("s"):
            self._publish_stop_signal()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraPointCvSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by keyboard interrupt.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
