from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch

from person_reid_tracker.path_utils import resolve_path
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
        )

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
        )

    def start_enrollment(self) -> None:
        self.identity.start_enrollment()

    def finish_enrollment(self) -> bool:
        return self.identity.finish_enrollment()

    def reset(self) -> None:
        self.identity.reset()

    def _angle_from_bbox(self, target: Optional[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Optional[float]:
        if target is None:
            return None
        h, w = frame_shape[:2]
        cx, _ = target.center
        normalized = (cx - (w * 0.5)) / max(1.0, w * 0.5)
        return normalized * (self.camera_fov_deg * 0.5)

    def _make_payload(self, result: IdentityResult, tracks: List[TrackCandidate], frame: np.ndarray) -> Dict[str, Any]:
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
            "ts": time.time(),
        }
        if target is not None:
            cx, cy = target.center
            payload.update({
                "bbox": [int(v) for v in target.bbox],
                "bbox_center": [round(float(cx), 2), round(float(cy), 2)],
                "bbox_area": int(target.area),
                "det_conf": round(float(target.conf), 4),
            })
        else:
            payload.update({"bbox": None, "bbox_center": None, "bbox_area": 0, "det_conf": 0.0})
        return payload

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any], IdentityResult, List[TrackCandidate]]:
        tracks = self.pipeline.detect_and_track(frame)
        result = self.identity.update(frame, tracks, self.pipeline.extract_features, now=time.time())
        debug = draw_identity_debug_frame(frame, tracks, result, draw_all_tracks=self.draw_all_tracks)
        payload = self._make_payload(result, tracks, frame)
        return debug, payload, result, tracks
