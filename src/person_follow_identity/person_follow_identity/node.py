from __future__ import annotations

import json
from typing import Any, Dict

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import IdentityFollowCore, parse_camera_source

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    CV_BRIDGE_AVAILABLE = True
except Exception:
    CvBridge = None
    Image = None
    CV_BRIDGE_AVAILABLE = False


class IdentityLockNode(Node):
    def __init__(self) -> None:
        super().__init__("identity_lock_node")
        self._declare_parameters()
        params = self._read_parameters()

        self.get_logger().info("Loading detector/tracker/identity lock...")
        self.core = IdentityFollowCore(params)
        self.get_logger().info(f"Models loaded; {self.core.device_note}")

        self.cap = cv2.VideoCapture(parse_camera_source(params["camera_source"]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(params["frame_width"]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(params["frame_height"]))
        if not self.cap.isOpened():
            self.get_logger().warning(f"Could not open camera source: {params['camera_source']}")

        self.target_pub = self.create_publisher(String, str(params["target_topic"]), 10)
        self.command_sub = self.create_subscription(String, str(params["command_topic"]), self._command_cb, 10)
        self.start_srv = self.create_service(Trigger, str(params["start_enroll_service"]), self._start_enroll_cb)
        self.finish_srv = self.create_service(Trigger, str(params["finish_enroll_service"]), self._finish_enroll_cb)
        self.reset_srv = self.create_service(Trigger, str(params["reset_service"]), self._reset_cb)

        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self.debug_img_pub = None
        if bool(params["publish_debug_image"]) and CV_BRIDGE_AVAILABLE:
            self.debug_img_pub = self.create_publisher(Image, str(params["debug_image_topic"]), 5)
        elif bool(params["publish_debug_image"]):
            self.get_logger().warning("cv_bridge is not available; debug image topic is disabled")

        self.show_window = bool(params["show_window"])
        self.window_name = "identity_lock"
        self.frame_skip = max(0, int(params["frame_skip"]))
        self.frame_count = 0
        timer_period = max(0.001, 1.0 / max(1.0, float(params["processing_hz"])))
        self.timer = self.create_timer(timer_period, self._timer_cb)

    def _declare_parameters(self) -> None:
        defaults: Dict[str, Any] = {
            "camera_source": "0",
            "frame_width": 640,
            "frame_height": 480,
            "model_weights": "",
            "deepsort_ckpt": "",
            "device": "auto",
            "use_cuda": True,
            "img_size": 640,
            "det_conf_thres": 0.55,
            "det_iou_thres": 0.50,
            "person_class_id": 0,
            "deepsort_max_age": 30,
            "deepsort_n_init": 4,
            "deepsort_max_dist": 0.18,
            "min_stable_frames": 4,
            "min_bbox_area": 1800,
            "initial_select_mode": "center_largest",
            "auto_enroll_on_start": False,
            "enroll_seconds": 30.0,
            "enroll_min_samples": 80,
            "enroll_sample_every_n_frames": 3,
            "positive_gallery_size": 160,
            "negative_gallery_size": 160,
            "color_gallery_size": 100,
            "reid_match_mode": "topk_mean",
            "reid_top_k": 5,
            "reid_threshold": 0.82,
            "current_min_reid": 0.60,
            "current_min_combined": 0.64,
            "current_min_color": 0.28,
            "recover_min_combined": 0.78,
            "recover_min_reid": 0.68,
            "recover_min_color": 0.25,
            "reid_margin": 0.16,
            "combined_margin": 0.12,
            "negative_margin": 0.18,
            "verification_fail_limit": 1,
            "verify_every_n_frames": 4,
            "update_gallery_every_n_frames": 4,
            "target_update_min_similarity": 0.62,
            "max_duplicate_similarity": 0.985,
            "negative_add_max_reid": 0.55,
            "negative_add_min_margin": 0.12,
            "spatial_gate_px": 220.0,
            "global_search_after_sec": 2.5,
            "global_search_min_reid": 0.90,
            "enable_global_recovery": False,
            "allow_single_candidate_global_recovery": False,
            "recovery_confirm_frames": 12,
            "recovery_single_candidate_min_reid": 0.94,
            "recovery_single_candidate_min_combined": 0.88,
            "max_lost_keep_id_sec": 2.0,
            "reid_weight": 0.70,
            "color_weight": 0.20,
            "gait_identity_weight": 0.10,
            "enable_gait": True,
            "gait_required_for_recovery": True,
            "gait_required_for_enrollment": False,
            "gait_sample_every_n_frames": 2,
            "gait_window": 45,
            "gait_min_frames": 18,
            "gait_gallery_size": 80,
            "gait_current_min_score": 0.50,
            "gait_recovery_min_score": 0.58,
            "gait_model_complexity": 1,
            "gait_min_detection_confidence": 0.45,
            "gait_min_tracking_confidence": 0.45,
            "gait_min_landmark_visibility": 0.25,
            "camera_fov_deg": 62.0,
            "draw_all_tracks": True,
            "publish_debug_image": True,
            "target_topic": "/person_reid/target",
            "debug_image_topic": "/person_reid/debug_image",
            "command_topic": "/person_reid/command",
            "start_enroll_service": "/person_reid/start_enroll",
            "finish_enroll_service": "/person_reid/finish_enroll",
            "reset_service": "/person_reid/reset",
            "processing_hz": 15.0,
            "frame_skip": 0,
            "show_window": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> Dict[str, Any]:
        names = [p.name for p in self._parameters.values()]
        return {name: self.get_parameter(name).value for name in names}

    def _command_cb(self, msg: String) -> None:
        cmd = str(msg.data).strip().lower()
        if cmd in {"e", "enroll", "start_enroll", "start"}:
            self.core.start_enrollment()
            self.get_logger().info("Enrollment started by command topic")
        elif cmd in {"f", "finish", "finish_enroll", "done"}:
            ready = self.core.finish_enrollment()
            self.get_logger().info(f"Enrollment finish requested; identity_ready={ready}")
        elif cmd in {"r", "reset"}:
            self.core.reset()
            self.get_logger().info("Target identity reset by command topic")

    def _start_enroll_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.core.start_enrollment()
        response.success = True
        response.message = "Enrollment started. Stand the target person near the camera center."
        return response

    def _finish_enroll_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ready = self.core.finish_enrollment()
        response.success = bool(ready)
        response.message = "Identity ready" if ready else "Not enough target samples yet"
        return response

    def _reset_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.core.reset()
        response.success = True
        response.message = "Target identity reset"
        return response

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
            if key == ord("e"):
                self.core.start_enrollment()
                self.get_logger().info("Enrollment started by keyboard")
            elif key == ord("f"):
                ready = self.core.finish_enrollment()
                self.get_logger().info(f"Enrollment finish requested; identity_ready={ready}")
            elif key == ord("r"):
                self.core.reset()
                self.get_logger().info("Target identity reset by keyboard")
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
    node = IdentityLockNode()
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
