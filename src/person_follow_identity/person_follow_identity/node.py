from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
import time
from typing import Any, Dict, Optional, Tuple

# BLAS cua numpy chi 1 luong. DeepSORT tinh khoang cach bang tich ma tran voi toi 100
# feature/track; OpenBLAS da luong cho cac phep nho nay roi quay cho ban, tranh CPU voi
# torch -> YOLO cham dan: do bang webcam 29 ms/khung luc dau -> 53-59 ms sau vai giay;
# gioi han 1 luong: on dinh 29 ms, 15 Hz. Phai dat TRUOC khi import numpy/cv2.
# KHONG dat MKL_NUM_THREADS/OMP_NUM_THREADS: torch doc hai bien do -> YOLO chay 1 luong (66 ms).
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
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


class LatestFrameGrabber:
    """Doc camera lien tuc trong luong rieng, chi giu khung MOI NHAT + thoi diem chup.

    Truoc day timer 8 Hz goi cap.read() truc tiep: camera chay 30 fps nen bo dem V4L2
    luon day va read() tra khung CU (tre them ~100+ ms), va thoi gian cho doc chan
    luon luong xu ly.

    exposure_mode="fixed_fps": camera o che do phoi sang THU CONG, luong nay tu chinh thoi
    gian phoi sang theo do sang anh nhung KHONG vuot exposure_max_ms. Do tren KINGSEN (30/09):
    phoi sang tu dong cua camera tu ha con nua fps (MJPG 12.5, YUYV 10 fps — co
    exposure_dynamic_framerate bi firmware bo qua), moi khung phoi ~80 ms -> nguoi di 1 m/s
    o 1.5 m nhoe ~28 px: ReID Rank-1 76.7% -> 59.2%, YOLO mat het nguoi o conf 0.55.
    Phoi sang <= 30 ms: 25 fps, nhoe ~11 px (Rank-1 73.9%).
    """

    def __init__(
        self,
        cap: cv2.VideoCapture,
        exposure_mode: str = "auto",
        exposure_ms: float = 20.0,
        exposure_min_ms: float = 1.0,
        exposure_max_ms: float = 30.0,
        exposure_step_ms: float = 10.0,
        brightness_target: float = 120.0,
        logger=None,
    ) -> None:
        self.cap = cap
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._stamp = 0.0
        self._seq = 0
        self.failures = 0
        self.logger = logger
        self.exposure_mode = str(exposure_mode).lower().strip()
        self.exposure_min_ms = max(0.1, float(exposure_min_ms))
        self.exposure_max_ms = max(self.exposure_min_ms, float(exposure_max_ms))
        self.exposure_step_ms = max(0.0, float(exposure_step_ms))
        self.brightness_target = float(brightness_target)
        self.exposure_ms = float(exposure_ms)
        self.brightness = float("nan")
        self._bsum = 0.0
        self._bn = 0
        self._last_adjust = time.time()
        self._last_change = 0.0
        self._warned_dark = False
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _quantize(self, ms: float) -> float:
        ms = min(self.exposure_max_ms, max(self.exposure_min_ms, ms))
        step = self.exposure_step_ms
        if step > 0 and ms >= step:
            # Boi so 10 ms: den dien 50 Hz nhap nhay 100 Hz, phoi sang tron chu ky thi khong soc/nhay sang
            ms = min(self.exposure_max_ms, max(step, round(ms / step) * step))
        return round(ms, 1)

    def _auto_exposure(self) -> None:
        if self._bn == 0:
            return
        b = self._bsum / self._bn
        self.brightness = b
        self._bsum, self._bn = 0.0, 0
        tgt = self.brightness_target
        # Dai rong + toi da 1 lan/5 s: KINGSEN hay tu tut 25 -> 12.5 fps vai chuc giay sau moi lan
        # doi phoi sang (do 30/09) nen chi doi khi anh lech nhieu.
        if 0.6 * tgt <= b <= 1.6 * tgt or time.time() - self._last_change < 5.0:
            return
        # Do sang ~ phoi sang^(1/gamma), gamma ~2 -> doi phoi sang theo binh phuong ti le
        ratio = min(2.0, max(0.5, (tgt / max(b, 1.0)) ** 2))
        new = self._quantize(self.exposure_ms * ratio)
        if new == self.exposure_ms:
            if b < 0.5 * tgt and new >= self.exposure_max_ms and not self._warned_dark and self.logger is not None:
                self.logger.warning(
                    f"Anh toi (do sang {b:.0f}/255) du da phoi sang toi da {self.exposure_max_ms:.0f} ms: "
                    "can them den, hoac dat camera_exposure_mode: auto (camera tu phoi sang lau hon nhung giam fps, nhoe hon)"
                )
                self._warned_dark = True
            return
        self.exposure_ms = new
        self._last_change = time.time()
        self.cap.set(cv2.CAP_PROP_EXPOSURE, new * 10.0)  # V4L2 exposure_time_absolute: don vi 100 us

    def _run(self) -> None:
        while self._running:
            ok, frame = self.cap.read()
            stamp = time.time()
            if not ok or frame is None:
                self.failures += 1
                time.sleep(0.02)
                continue
            with self._lock:
                self._frame, self._stamp, self._seq = frame, stamp, self._seq + 1
            # Do sang vung giua-duoi (noi co than nguoi); bo 1/4 tren: camera ngua len thi den
            # tran / cua so o mep tren keo phoi sang xuong lam nguoi bi toi
            h, w = frame.shape[:2]
            self._bsum += float(frame[h // 4::16, w // 8:w - w // 8:16].mean())
            self._bn += 1
            if stamp - self._last_adjust >= 0.5:
                self._last_adjust = stamp
                if self.exposure_mode == "fixed_fps":
                    self._auto_exposure()
                elif self._bn:
                    self.brightness = self._bsum / self._bn
                    self._bsum, self._bn = 0.0, 0

    def latest(self) -> Tuple[Optional[np.ndarray], float, int]:
        with self._lock:
            return self._frame, self._stamp, self._seq

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)


class IdentityLockNode(Node):
    def __init__(self) -> None:
        super().__init__("identity_lock_node")
        self._declare_parameters()
        params = self._read_parameters()

        self.get_logger().info("Loading detector/tracker/identity lock...")
        self.core = IdentityFollowCore(params)
        self.get_logger().info(f"Models loaded; {self.core.device_note}")

        source = parse_camera_source(params["camera_source"])
        # Dat dieu khien v4l2 TRUOC khi mo: do tren KINGSEN 30/09, len 25 fps sau ~8 s thay vi 17-24 s
        self._apply_v4l2_controls(source, str(params["camera_v4l2_controls"]))
        self.cap = cv2.VideoCapture(source)
        fourcc = str(params["camera_fourcc"]).strip()
        if fourcc:
            # Dat TRUOC kich thuoc. KINGSEN 640x480: MJPG toi 25 fps, YUYV chi 15-20 fps
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*(fourcc + "    ")[:4]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(params["frame_width"]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(params["frame_height"]))
        if float(params["camera_fps"]) > 0:
            self.cap.set(cv2.CAP_PROP_FPS, float(params["camera_fps"]))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.grabber: Optional[LatestFrameGrabber] = None
        if not self.cap.isOpened():
            self.get_logger().warning(f"Could not open camera source: {params['camera_source']}")
        else:
            mode = str(params["camera_exposure_mode"]).lower().strip()
            exp_ms = float(params["camera_exposure_ms"])
            if mode in {"fixed_fps", "manual"}:
                # V4L2: 1 = thu cong, 3 = tu dong (Aperture Priority)
                if not (self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) and self.cap.set(cv2.CAP_PROP_EXPOSURE, exp_ms * 10.0)):
                    self.get_logger().warning("Camera khong nhan phoi sang thu cong -> dung phoi sang tu dong")
                    mode = "auto"
            if mode == "auto":
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            got = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
            self.get_logger().info(
                f"camera: {got} {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
                f"{self.cap.get(cv2.CAP_PROP_FPS):.0f} fps, phoi sang {mode}" + (f" {exp_ms:.0f} ms" if mode != "auto" else "")
            )
            self.grabber = LatestFrameGrabber(
                self.cap,
                exposure_mode=mode,
                exposure_ms=exp_ms,
                exposure_min_ms=float(params["camera_exposure_min_ms"]),
                exposure_max_ms=float(params["camera_exposure_max_ms"]),
                exposure_step_ms=float(params["camera_exposure_step_ms"]),
                brightness_target=float(params["camera_brightness_target"]),
                logger=self.get_logger(),
            )
        self.last_seq = 0
        self.last_fail_log = 0.0
        self.stat_t0 = time.time()
        self.stat_n = 0
        self.stat_proc = 0.0
        self.stat_seq0 = 0

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
        self.stats_period = float(params["stats_period_sec"])
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
            # 09/10: file .onnx (ten ngan = tim trong model_assets) -> onnxruntime; de trong = PyTorch + ckpt.t7
            "detector_onnx": "",
            "reid_onnx": "",
            "ort_threads": 2,
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
            # --- Toi uu dam dong / camera goc rong (29/09) ---
            "rect_inference": True,
            "occlusion_ratio_max": 0.35,
            "occluded_fail_weight": 0.25,
            "learn_min_quality": 0.75,
            "eval_min_quality": 0.45,
            "switch_margin": 0.08,
            "strong_negative_margin": 0.04,
            "current_negative_margin": 0.0,
            "self_consistency_min": 0.72,
            "recover_strong_reid": 0.88,
            "view_bucket_min_samples": 5,
            "bucket_bootstrap_frames": 8,
            "suspect_penalty": 2.0,
            "edge_margin_px": 6,
            "occluded_loss_hold_sec": 2.5,
            "recovery_probation_sec": 3.0,
            "camera_angle_model": "pinhole",
            "camera_pitch_deg": 0.0,           # goc ngua len cua camera (do), duong = ngua len
            # 9 so (hang theo hang) tu scripts/calibrate_camera.py; fx = 0 nghia la chua hieu chinh
            "camera_matrix": [0.0] * 9,
            "dist_coeffs": [0.0] * 5,
            "camera_fisheye": False,
            "undistort_frame": False,
            "undistort_balance": 0.0,
            "stats_period_sec": 10.0,
            # --- Doc camera (30/09) ---
            "camera_fourcc": "MJPG",
            "camera_fps": 0.0,                 # 0 = de mac dinh cua camera
            "camera_exposure_mode": "auto",    # auto | fixed_fps | manual
            "camera_exposure_ms": 20.0,        # manual: gia tri co dinh; fixed_fps: gia tri bat dau
            "camera_exposure_min_ms": 1.0,
            "camera_exposure_max_ms": 30.0,    # 39 ms sat chu ky 40 ms (25 fps) -> KINGSEN tut 12.5 fps
            "camera_exposure_step_ms": 10.0,   # >= 10 ms thi lam tron boi so 10 ms (den 50 Hz); 0 = lien tuc
            "camera_brightness_target": 120.0,
            "camera_v4l2_controls": "",        # vd "power_line_frequency=1" (can v4l2-ctl)
        }
        # dynamic_typing: yaml/-p ghi 15 hay 15.0 deu duoc (ROS 2 phan biet INTEGER/DOUBLE)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(dynamic_typing=True))

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
        if self.grabber is None:
            return
        frame, stamp, seq = self.grabber.latest()
        if frame is None or seq == self.last_seq:
            # Chua co khung moi: khong xu ly lai khung cu (target_tracker coi moi tin la mot phep do)
            now = time.time()
            if self.grabber.failures > 0 and now - self.last_fail_log > 5.0:
                self.get_logger().warning(f"Failed to read camera frame ({self.grabber.failures} lan)")
                self.last_fail_log = now
            return
        self.last_seq = seq

        self.frame_count += 1
        if self.frame_skip > 0 and (self.frame_count % (self.frame_skip + 1)) != 1:
            return

        need_debug = self.show_window or self.debug_img_pub is not None
        t0 = time.time()
        try:
            debug, payload, _, _ = self.core.process_frame(frame, capture_ts=stamp, draw_debug=need_debug)
        except Exception as exc:
            self.get_logger().error(f"Processing error: {exc}")
            return

        self._publish_payload(payload)
        self._log_stats(time.time() - t0, payload)

        if debug is None:
            return
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

    def _apply_v4l2_controls(self, source: Any, controls: str) -> None:
        controls = controls.strip()
        if not controls:
            return
        dev = f"/dev/video{source}" if isinstance(source, int) else str(source)
        exe = shutil.which("v4l2-ctl")
        if exe is None:
            self.get_logger().warning("Khong co v4l2-ctl (sudo apt install v4l-utils) -> bo qua camera_v4l2_controls")
            return
        res = subprocess.run([exe, "-d", dev, "-c", controls], capture_output=True, text=True, timeout=5)
        if res.returncode != 0:
            self.get_logger().warning(f"v4l2-ctl -c {controls} loi: {res.stderr.strip()}")

    def _log_stats(self, proc_sec: float, payload: Dict[str, Any]) -> None:
        self.stat_n += 1
        self.stat_proc += proc_sec
        now = time.time()
        if self.stats_period <= 0 or now - self.stat_t0 < self.stats_period:
            return
        span = now - self.stat_t0
        # fps camera THUC gui ve (thieu sang thi nhieu webcam tu giam fps -> tre tang)
        cam_fps = (self.last_seq - self.stat_seq0) / span if self.stat_seq0 else float("nan")
        self.stat_seq0 = self.grabber.latest()[2] if self.grabber is not None else 0
        self.get_logger().info(
            f"camera: nhan {cam_fps:.1f} fps (phoi sang {self._exposure_text()}, do sang {self.grabber.brightness if self.grabber else float('nan'):.0f}), "
            f"xu ly {self.stat_n / span:.1f} Hz, TB {1000.0 * self.stat_proc / max(1, self.stat_n):.0f} ms/khung, "
            f"tre tu luc chup {payload.get('latency_ms')} ms, {payload.get('num_tracks')} nguoi, status {payload.get('status')}"
        )
        self.stat_t0, self.stat_n, self.stat_proc = now, 0, 0.0

    def _exposure_text(self) -> str:
        if self.grabber is None or self.grabber.exposure_mode == "auto":
            return "tu dong"
        return f"{self.grabber.exposure_ms:.0f} ms"

    def destroy_node(self) -> bool:
        if self.grabber is not None:
            self.grabber.stop()
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
