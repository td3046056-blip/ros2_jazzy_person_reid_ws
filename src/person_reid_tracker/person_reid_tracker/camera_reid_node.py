from __future__ import annotations

import json
import time
from typing import Any, Dict

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .camera_reid_core import CameraReIDCore, parse_camera_source
from .path_utils import resolve_path

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    CV_BRIDGE_AVAILABLE = True
except Exception:
    CvBridge = None
    Image = None
    CV_BRIDGE_AVAILABLE = False


class CameraReIDNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_reid_node")
        self._declare_parameters()
        params = self._read_parameters()

        self.get_logger().info("Loading YOLOv5 + DeepSORT/ReID models...")
        self.core = CameraReIDCore(params)
        self.get_logger().info("Models loaded")

        self.cap = cv2.VideoCapture(parse_camera_source(params["camera_source"]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(params["frame_width"]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(params["frame_height"]))
        if not self.cap.isOpened():
            self.get_logger().warning(f"Could not open camera source: {params['camera_source']}")

        self.target_pub = self.create_publisher(String, str(params["target_topic"]), 10)
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self.debug_img_pub = None
        if bool(params["publish_debug_image"]) and CV_BRIDGE_AVAILABLE:
            self.debug_img_pub = self.create_publisher(Image, str(params["debug_image_topic"]), 5)
        elif bool(params["publish_debug_image"]):
            self.get_logger().warning("cv_bridge is not available; debug image topic is disabled")

        self.show_window = bool(params["show_window"])
        self.window_name = "person_reid_tracker"
        self.frame_skip = max(0, int(params["frame_skip"]))
        self.frame_count = 0
        timer_period = max(0.001, 1.0 / max(1.0, float(params["processing_hz"])))
        self.timer = self.create_timer(timer_period, self._timer_cb)

    def _declare_parameters(self) -> None:
        defaults = {
            "camera_source": "0",
            "frame_width": 640,
            "frame_height": 480,
            "model_weights": "",
            "deepsort_ckpt": "",
            "device": "cpu",
            "use_cuda": False,
            "img_size": 640,
            "det_conf_thres": 0.5,
            "det_iou_thres": 0.5,
            "person_class_id": 0,
            "deepsort_max_age": 70,
            "deepsort_n_init": 3,
            "deepsort_max_dist": 0.2,
            "min_stable_frames": 3,
            "initial_select_mode": "largest_box",
            "reid_gallery_size": 30,
            "reid_threshold": 0.78,
            "reid_margin": 0.10,
            "max_lost_sec": 5.0,
            "spatial_gate_px": 220.0,
            "update_gallery_every_n_frames": 5,
            "verify_current_track": True,
            "verify_every_n_frames": 5,
            "current_min_reid": 0.42,
            "current_min_color": 0.18,
            "current_min_combined": 0.48,
            "verification_fail_limit": 2,
            "reid_match_mode": "max",
            "recover_min_combined": 0.66,
            "recover_min_reid_for_combined": 0.45,
            "recover_min_color": 0.20,
            "color_weight": 0.30,
            "reid_weight": 0.70,
            "min_bbox_area": 1200,
            "camera_fov_deg": 62.0,
            "draw_all_tracks": False,
            "publish_debug_image": True,
            "target_topic": "/person_reid/target",
            "debug_image_topic": "/person_reid/debug_image",
            "processing_hz": 15.0,
            "frame_skip": 0,
            "show_window": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> Dict[str, Any]:
        names = [p.name for p in self._parameters.values()]
        values = {name: self.get_parameter(name).value for name in names}
        values["model_weights"] = resolve_path(str(values.get("model_weights", "")), "yolov5n.pt")
        values["deepsort_ckpt"] = resolve_path(str(values.get("deepsort_ckpt", "")), "ckpt.t7")
        return values

    def _publish_payload(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.target_pub.publish(msg)

    def _timer_cb(self) -> None:
        if self.cap is None or not self.cap.isOpened():
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning("Failed to read camera frame")
            return

        self.frame_count += 1
        if self.frame_skip > 0 and (self.frame_count % (self.frame_skip + 1)) != 1:
            return

        try:
            debug, payload, _, _ = self.core.process_frame(frame)
        except Exception as exc:
            self.get_logger().error(f"Processing error: {exc}")
            return

        self._publish_payload(payload)

        if self.debug_img_pub is not None and self.bridge is not None:
            img_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera"
            self.debug_img_pub.publish(img_msg)

        if self.show_window:
            cv2.imshow(self.window_name, debug)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                self.core.reset_target()
                self.get_logger().info("Target reset by keyboard")
            elif key == ord("q"):
                rclpy.shutdown()

    def destroy_node(self) -> bool:
        if self.cap is not None:
            self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraReIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
