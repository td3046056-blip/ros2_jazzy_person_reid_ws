from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch

from person_reid_tracker.path_utils import default_model_path, resolve_path
from person_reid_tracker.types import TrackCandidate
from person_reid_tracker.yolo_deepsort_pipeline import YoloDeepSortPipeline

from .identity_lock_manager import IdentityResult, TargetIdentityManager
from .visualization import draw_identity_debug_frame


def parse_camera_source(source: Any) -> Any:
    if isinstance(source, int):
        return source
    text = str(source)
    if text.isdigit():
        return int(text)
    return text


def _model_file(value: Any) -> str:
    """'' -> '' (dung PyTorch); ten ngan (khong co '/') -> tim trong model_assets; duong dan -> giu nguyen."""
    v = str(value or "").strip()
    if not v:
        return ""
    return default_model_path(v) if "/" not in v else str(v)


def choose_device(device: str, use_cuda: bool) -> Tuple[str, bool, str]:
    requested = str(device).lower().strip()
    if requested in {"auto", "gpu"}:
        if torch.cuda.is_available():
            return "cuda", True, "torch cuda/rocm backend available"
        return "cpu", False, "torch cuda/rocm backend not available; using CPU"
    if requested == "cuda" and not torch.cuda.is_available():
        return "cpu", False, "requested cuda but torch cuda/rocm is not available; using CPU"
    if requested == "cuda":
        return "cuda", bool(use_cuda), "using torch cuda/rocm backend"
    return "cpu", False, "using CPU"


class IdentityFollowCore:
    def __init__(self, params: Dict[str, Any]) -> None:
        self.params = params
        self.frame_width = int(params.get("frame_width", 640))
        self.frame_height = int(params.get("frame_height", 480))
        self.camera_fov_deg = float(params.get("camera_fov_deg", 62.0))
        self.draw_all_tracks = bool(params.get("draw_all_tracks", True))

        device, use_cuda, device_note = choose_device(str(params.get("device", "auto")), bool(params.get("use_cuda", True)))
        self.device_note = device_note

        self.pipeline = YoloDeepSortPipeline(
            model_weights=resolve_path(str(params.get("model_weights", "")), "yolov5n.pt"),
            deepsort_ckpt=resolve_path(str(params.get("deepsort_ckpt", "")), "ckpt.t7"),
            device=device,
            use_cuda=use_cuda,
            img_size=int(params.get("img_size", 640)),
            conf_thres=float(params.get("det_conf_thres", 0.55)),
            iou_thres=float(params.get("det_iou_thres", 0.50)),
            person_class_id=int(params.get("person_class_id", 0)),
            deepsort_max_age=int(params.get("deepsort_max_age", 30)),
            deepsort_n_init=int(params.get("deepsort_n_init", 4)),
            deepsort_max_dist=float(params.get("deepsort_max_dist", 0.18)),
            rect_inference=bool(params.get("rect_inference", True)),
            detector_onnx=_model_file(params.get("detector_onnx", "")),
            reid_onnx=_model_file(params.get("reid_onnx", "")),
            ort_threads=int(params.get("ort_threads", 2)),
        )
        self._setup_camera_model(params)

        self.identity = TargetIdentityManager(
            min_stable_frames=int(params.get("min_stable_frames", 4)),
            min_bbox_area=int(params.get("min_bbox_area", 1800)),
            select_mode=str(params.get("initial_select_mode", "center_largest")),
            auto_enroll_on_start=bool(params.get("auto_enroll_on_start", False)),
            enroll_seconds=float(params.get("enroll_seconds", 30.0)),
            enroll_min_samples=int(params.get("enroll_min_samples", 80)),
            enroll_sample_every_n_frames=int(params.get("enroll_sample_every_n_frames", 3)),
            positive_gallery_size=int(params.get("positive_gallery_size", 160)),
            negative_gallery_size=int(params.get("negative_gallery_size", 160)),
            color_gallery_size=int(params.get("color_gallery_size", 100)),
            reid_match_mode=str(params.get("reid_match_mode", "topk_mean")),
            reid_top_k=int(params.get("reid_top_k", 5)),
            reid_threshold=float(params.get("reid_threshold", 0.82)),
            current_min_reid=float(params.get("current_min_reid", 0.60)),
            current_min_combined=float(params.get("current_min_combined", 0.64)),
            current_min_color=float(params.get("current_min_color", 0.28)),
            recover_min_combined=float(params.get("recover_min_combined", 0.78)),
            recover_min_reid=float(params.get("recover_min_reid", 0.68)),
            recover_min_color=float(params.get("recover_min_color", 0.25)),
            reid_margin=float(params.get("reid_margin", 0.16)),
            combined_margin=float(params.get("combined_margin", 0.12)),
            negative_margin=float(params.get("negative_margin", 0.18)),
            verification_fail_limit=int(params.get("verification_fail_limit", 1)),
            verify_every_n_frames=int(params.get("verify_every_n_frames", 4)),
            update_gallery_every_n_frames=int(params.get("update_gallery_every_n_frames", 4)),
            target_update_min_similarity=float(params.get("target_update_min_similarity", 0.62)),
            max_duplicate_similarity=float(params.get("max_duplicate_similarity", 0.985)),
            negative_add_max_reid=float(params.get("negative_add_max_reid", 0.55)),
            negative_add_min_margin=float(params.get("negative_add_min_margin", 0.12)),
            spatial_gate_px=float(params.get("spatial_gate_px", 220.0)),
            global_search_after_sec=float(params.get("global_search_after_sec", 2.5)),
            global_search_min_reid=float(params.get("global_search_min_reid", 0.90)),
            enable_global_recovery=bool(params.get("enable_global_recovery", False)),
            allow_single_candidate_global_recovery=bool(params.get("allow_single_candidate_global_recovery", False)),
            recovery_confirm_frames=int(params.get("recovery_confirm_frames", 12)),
            recovery_single_candidate_min_reid=float(params.get("recovery_single_candidate_min_reid", 0.94)),
            recovery_single_candidate_min_combined=float(params.get("recovery_single_candidate_min_combined", 0.88)),
            max_lost_keep_id_sec=float(params.get("max_lost_keep_id_sec", 2.0)),
            reid_weight=float(params.get("reid_weight", 0.70)),
            color_weight=float(params.get("color_weight", 0.20)),
            gait_identity_weight=float(params.get("gait_identity_weight", 0.10)),
            enable_gait=bool(params.get("enable_gait", True)),
            gait_required_for_recovery=bool(params.get("gait_required_for_recovery", True)),
            gait_required_for_enrollment=bool(params.get("gait_required_for_enrollment", False)),
            gait_sample_every_n_frames=int(params.get("gait_sample_every_n_frames", 2)),
            gait_window=int(params.get("gait_window", 45)),
            gait_min_frames=int(params.get("gait_min_frames", 18)),
            gait_gallery_size=int(params.get("gait_gallery_size", 80)),
            gait_current_min_score=float(params.get("gait_current_min_score", 0.50)),
            gait_recovery_min_score=float(params.get("gait_recovery_min_score", 0.58)),
            gait_model_complexity=int(params.get("gait_model_complexity", 1)),
            gait_min_detection_confidence=float(params.get("gait_min_detection_confidence", 0.45)),
            gait_min_tracking_confidence=float(params.get("gait_min_tracking_confidence", 0.45)),
            gait_min_landmark_visibility=float(params.get("gait_min_landmark_visibility", 0.25)),
            occlusion_ratio_max=float(params.get("occlusion_ratio_max", 0.35)),
            occluded_fail_weight=float(params.get("occluded_fail_weight", 0.25)),
            learn_min_quality=float(params.get("learn_min_quality", 0.75)),
            eval_min_quality=float(params.get("eval_min_quality", 0.45)),
            switch_margin=float(params.get("switch_margin", 0.08)),
            strong_negative_margin=float(params.get("strong_negative_margin", 0.04)),
            current_negative_margin=float(params.get("current_negative_margin", 0.0)),
            self_consistency_min=float(params.get("self_consistency_min", 0.72)),
            recover_strong_reid=float(params.get("recover_strong_reid", 0.88)),
            view_bucket_min_samples=int(params.get("view_bucket_min_samples", 5)),
            bucket_bootstrap_frames=int(params.get("bucket_bootstrap_frames", 8)),
            suspect_penalty=float(params.get("suspect_penalty", 2.0)),
            edge_margin_px=int(params.get("edge_margin_px", 6)),
            occluded_loss_hold_sec=float(params.get("occluded_loss_hold_sec", 2.5)),
            recovery_probation_sec=float(params.get("recovery_probation_sec", 3.0)),
        )

    def _setup_camera_model(self, params: Dict[str, Any]) -> None:
        """Mo hinh doi toa do anh -> goc ngang.

        linear     : goc = (cx - w/2)/(w/2) * FOV/2 — cach cu, sai voi camera goc rong
                     (FOV 62 do: sai toi 1.2 do giua nua khung; FOV 120 do: sai toi 11 do).
        pinhole    : goc = atan((cx - w/2)/fx), fx = (w/2)/tan(FOV/2). Dung cho ong kinh it meo.
        calibrated : dung camera_matrix + dist_coeffs (tu scripts/calibrate_camera.py), khu meo
                     diem tam bbox truoc khi tinh goc. camera_fisheye=true cho ong mat ca.
        undistort_frame=true (chi khi da hieu chinh): khu meo ca khung truoc YOLO/ReID — nguoi o
        mep anh goc rong het bi keo nghieng/phinh, ReID so khop tot hon; ton ~1-2 ms/khung.
        """
        self.angle_model = str(params.get("camera_angle_model", "pinhole")).lower().strip()
        # Camera ngua len (duong) de thay than nguoi o gan: goc ngang = atan2(x, cos p + y sin p)
        self.camera_pitch = math.radians(float(params.get("camera_pitch_deg", 0.0)))
        k = [float(v) for v in (params.get("camera_matrix") or [])]
        d = [float(v) for v in (params.get("dist_coeffs") or [])]
        self.fisheye = bool(params.get("camera_fisheye", False))
        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        if self.fisheye:
            # Mo hinh mat ca OpenCV can dung 4 he so (k1..k4); yaml mac dinh co 5 so 0
            d = (d + [0.0, 0.0, 0.0, 0.0])[:4]
        if len(k) == 9 and k[0] > 0.0 and k[4] > 0.0:
            self.K = np.asarray(k, dtype=np.float64).reshape(3, 3)
            self.D = np.asarray(d if d else [0.0] * (4 if self.fisheye else 5), dtype=np.float64).reshape(-1, 1)
        if self.angle_model == "calibrated" and self.K is None:
            self.device_note += "; camera_angle_model=calibrated nhung thieu camera_matrix -> dung pinhole"
            self.angle_model = "pinhole"
        self.undistort_maps = None
        self.K_rect: Optional[np.ndarray] = None
        if bool(params.get("undistort_frame", False)):
            if self.K is None:
                self.device_note += "; undistort_frame bi bo qua vi chua co camera_matrix"
            else:
                size = (self.frame_width, self.frame_height)
                if self.fisheye:
                    self.K_rect = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                        self.K, self.D, size, np.eye(3), balance=float(params.get("undistort_balance", 0.0)))
                    m1, m2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), self.K_rect, size, cv2.CV_16SC2)
                else:
                    self.K_rect, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, size, float(params.get("undistort_balance", 0.0)))
                    m1, m2 = cv2.initUndistortRectifyMap(self.K, self.D, None, self.K_rect, size, cv2.CV_16SC2)
                self.undistort_maps = (m1, m2, size)

    def start_enrollment(self) -> None:
        self.identity.start_enrollment()

    def finish_enrollment(self) -> bool:
        return self.identity.finish_enrollment()

    def reset(self) -> None:
        self.identity.reset()

    def _normalized_ray(self, cx: float, cy: float, w: int, h: int) -> Optional[Tuple[float, float]]:
        """Diem anh -> (xn, yn) cua tia trong khung camera (x phai, y xuong), cung mo hinh voi goc ngang.
        None = mo hinh 'linear' (khong co tia)."""
        if self.K_rect is not None and self.undistort_maps is not None:
            return (cx - self.K_rect[0, 2]) / self.K_rect[0, 0], (cy - self.K_rect[1, 2]) / self.K_rect[1, 1]
        if self.angle_model == "calibrated" and self.K is not None:
            pt = np.asarray([[[cx, cy]]], dtype=np.float64)
            und = cv2.fisheye.undistortPoints(pt, self.K, self.D) if self.fisheye else cv2.undistortPoints(pt, self.K, self.D)
            return float(und[0, 0, 0]), float(und[0, 0, 1])
        if self.angle_model == "linear":
            return None
        fx = (w * 0.5) / math.tan(math.radians(self.camera_fov_deg * 0.5))
        return (cx - (w * 0.5)) / max(1e-6, fx), (cy - (h * 0.5)) / max(1e-6, fx)

    def _bbox_elevations(self, target: Optional[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Dict[str, Any]:
        """Goc CAO (do, duong = tren duong chan troi, theo khung XE: da bu camera_pitch_deg) cua dinh va day
        bbox, tai cot giua bbox; kem co cham mep tren / duoi anh.

        target_tracker dung de: (1) biet CHAN nguoi co bi vat thap che khong (day bbox cao hon cho chan
        o khoang cach cua cum LiDAR) — truoc day tracker nhan mat truoc thung 30 cm la chan nguoi dung sau
        thung, xe dung "cach 0.74 m" khi nguoi con cach 1.84 m (07/10); (2) uoc luong khoang cach tu DINH
        DAU khi chan bi che (thay cho bbox_height_at_1m_px cua camera cu 62 do, sai voi ong mat ca)."""
        out: Dict[str, Any] = {"bbox_top_elev_deg": None, "bbox_bottom_elev_deg": None,
                               "bbox_top_cut": None, "bbox_bottom_cut": None}
        if target is None:
            return out
        h, w = frame_shape[:2]
        x1, y1, x2, y2 = [float(v) for v in target.bbox]
        cx = 0.5 * (x1 + x2)
        p = self.camera_pitch
        for key, yy in (("bbox_top_elev_deg", y1), ("bbox_bottom_elev_deg", y2)):
            ray = self._normalized_ray(cx, yy, w, h)
            if ray is None:
                return out
            xn, yn = ray
            # tia (xn, yn, 1), camera ngua len p: cao = atan2(sin p - yn cos p, |(cos p + yn sin p, xn)|)
            elev = math.atan2(math.sin(p) - yn * math.cos(p), math.hypot(math.cos(p) + yn * math.sin(p), xn))
            out[key] = round(math.degrees(elev), 2)
        out["bbox_top_cut"] = bool(y1 <= 2.0)
        out["bbox_bottom_cut"] = bool(y2 >= h - 3.0)
        return out

    def _angle_from_bbox(self, target: Optional[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Optional[float]:
        if target is None:
            return None
        h, w = frame_shape[:2]
        cx, cy = target.center
        if self.K_rect is not None and self.undistort_maps is not None:
            # Khung da khu meo -> anh pinhole voi ma tran K_rect
            xn = (cx - self.K_rect[0, 2]) / self.K_rect[0, 0]
            yn = (cy - self.K_rect[1, 2]) / self.K_rect[1, 1]
        elif self.angle_model == "calibrated" and self.K is not None:
            pt = np.asarray([[[cx, cy]]], dtype=np.float64)
            und = cv2.fisheye.undistortPoints(pt, self.K, self.D) if self.fisheye else cv2.undistortPoints(pt, self.K, self.D)
            xn, yn = float(und[0, 0, 0]), float(und[0, 0, 1])
        elif self.angle_model == "linear":
            normalized = (cx - (w * 0.5)) / max(1.0, w * 0.5)
            return normalized * (self.camera_fov_deg * 0.5)
        else:
            fx = (w * 0.5) / math.tan(math.radians(self.camera_fov_deg * 0.5))
            xn = (cx - (w * 0.5)) / max(1e-6, fx)
            yn = (cy - (h * 0.5)) / max(1e-6, fx)  # diem anh vuong
        # y anh huong xuong; camera ngua len p: tia (xn, yn, 1) -> ngang = atan2(xn, cos p + yn sin p)
        p = self.camera_pitch
        return math.degrees(math.atan2(xn, math.cos(p) + yn * math.sin(p)))

    def _make_payload(self, result: IdentityResult, tracks: List[TrackCandidate], frame: np.ndarray, capture_ts: Optional[float] = None) -> Dict[str, Any]:
        target = result.target
        angle = self._angle_from_bbox(target, frame.shape)
        payload: Dict[str, Any] = {
            "target_found": target is not None and result.status not in {"ENROLLING", "ENROLLING_WAITING_FOR_PERSON"},
            "status": result.status,
            "target_track_id": result.target_track_id,
            "recovered": bool(result.recovered),
            "verified": bool(result.verified),
            "identity_ready": bool(self.identity.identity_ready),
            "enrolled_samples": int(result.enrolled_samples),
            "negative_samples": int(result.negative_samples),
            "enrollment_progress": round(float(result.enrollment_progress), 3),
            "reid_similarity": round(float(result.reid_similarity), 4),
            "color_similarity": round(float(result.color_similarity), 4),
            "negative_similarity": round(float(result.negative_similarity), 4),
            "identity_margin": round(float(result.identity_margin), 4),
            "combined_similarity": round(float(result.combined_similarity), 4),
            "gait_similarity": round(float(result.gait_similarity), 4),
            "body_similarity": round(float(result.body_similarity), 4),
            "silhouette_similarity": round(float(result.silhouette_similarity), 4),
            "gait_ready": bool(result.gait_ready),
            "live_gait_ready": bool(result.live_gait_ready),
            "gait_target_samples": int(getattr(self.identity.gait, "target_sample_count", 0)),
            "second_similarity": round(float(result.second_similarity), 4),
            "lost_age_sec": round(float(result.lost_age_sec), 3),
            "reason": result.reason,
            "num_tracks": len(tracks),
            "frame_width": int(frame.shape[1]),
            "frame_height": int(frame.shape[0]),
            "camera_angle_deg": None if angle is None else round(float(angle), 3),
            "occlusion": round(float(result.occlusion), 3),
            "evidence": round(float(result.evidence), 2),
            "view_bucket": int(result.view_bucket),
            # ts = LUC CHUP khung (target_tracker dung de bu goc xe da quay trong luc xu ly);
            # truoc day la luc xu ly xong nen phan bu tre gan nhu bang 0.
            "ts": float(capture_ts) if capture_ts is not None else time.time(),
            "latency_ms": None if capture_ts is None else round((time.time() - float(capture_ts)) * 1000.0, 1),
        }
        if target is not None:
            cx, cy = target.center
            payload.update({
                "bbox": [int(v) for v in target.bbox],
                "bbox_center": [round(float(cx), 2), round(float(cy), 2)],
                "bbox_area": int(target.area),
                "det_conf": round(float(target.conf), 4),
            })
            payload.update(self._bbox_elevations(target, frame.shape))
        else:
            payload.update({"bbox": None, "bbox_center": None, "bbox_area": 0, "det_conf": 0.0})
        return payload

    def process_frame(
        self, frame: np.ndarray, capture_ts: Optional[float] = None, draw_debug: bool = True,
    ) -> Tuple[Optional[np.ndarray], Dict[str, Any], IdentityResult, List[TrackCandidate]]:
        if self.undistort_maps is not None:
            m1, m2, size = self.undistort_maps
            if (frame.shape[1], frame.shape[0]) != size:
                frame = cv2.resize(frame, size)
            frame = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
        tracks = self.pipeline.detect_and_track(frame)
        now = time.time() if capture_ts is None else float(capture_ts)
        result = self.identity.update(frame, tracks, self.pipeline.extract_features, now=now)
        debug = draw_identity_debug_frame(frame, tracks, result, draw_all_tracks=self.draw_all_tracks) if draw_debug else None
        payload = self._make_payload(result, tracks, frame, capture_ts)
        return debug, payload, result, tracks
