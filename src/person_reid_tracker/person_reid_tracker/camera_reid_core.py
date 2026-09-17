from __future__ import annotations

import json
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .target_lock import TargetLockManager, TargetResult
from .types import TrackCandidate
from .visualization import draw_debug_frame
from .yolo_deepsort_pipeline import YoloDeepSortPipeline


def parse_camera_source(source: Any) -> Any:
    if isinstance(source, int):
        return source
    text = str(source)
    if text.isdigit():
        return int(text)
    return text


class CameraReIDCore:
    """Reusable core used by both ROS2 node and non-ROS demo script."""

    def __init__(self, params: Dict[str, Any]) -> None:
        self.params = params
        self.frame_width = int(params.get("frame_width", 640))
        self.frame_height = int(params.get("frame_height", 480))
        self.camera_fov_deg = float(params.get("camera_fov_deg", 62.0))
        self.draw_all_tracks = bool(params.get("draw_all_tracks", False))

        self.pipeline = YoloDeepSortPipeline(
            model_weights=str(params["model_weights"]),
            deepsort_ckpt=str(params["deepsort_ckpt"]),
            device=str(params.get("device", "cpu")),
            use_cuda=bool(params.get("use_cuda", False)),
            img_size=int(params.get("img_size", 640)),
            conf_thres=float(params.get("det_conf_thres", 0.5)),
            iou_thres=float(params.get("det_iou_thres", 0.5)),
            person_class_id=int(params.get("person_class_id", 0)),
            deepsort_max_age=int(params.get("deepsort_max_age", 70)),
            deepsort_n_init=int(params.get("deepsort_n_init", 3)),
            deepsort_max_dist=float(params.get("deepsort_max_dist", 0.2)),
        )
        self.target_manager = TargetLockManager(
            min_stable_frames=int(params.get("min_stable_frames", 3)),
            gallery_size=int(params.get("reid_gallery_size", 30)),
            reid_threshold=float(params.get("reid_threshold", 0.78)),
            reid_margin=float(params.get("reid_margin", 0.10)),
            max_lost_sec=float(params.get("max_lost_sec", 5.0)),
            spatial_gate_px=float(params.get("spatial_gate_px", 220.0)),
            select_mode=str(params.get("initial_select_mode", "largest_box")),
            update_gallery_every_n_frames=int(params.get("update_gallery_every_n_frames", 5)),
            verify_current_track=bool(params.get("verify_current_track", True)),
            verify_every_n_frames=int(params.get("verify_every_n_frames", 5)),
            current_min_reid=float(params.get("current_min_reid", 0.42)),
            current_min_color=float(params.get("current_min_color", 0.18)),
            current_min_combined=float(params.get("current_min_combined", 0.48)),
            verification_fail_limit=int(params.get("verification_fail_limit", 2)),
            reid_match_mode=str(params.get("reid_match_mode", "max")),
            recover_min_combined=float(params.get("recover_min_combined", 0.66)),
            recover_min_reid_for_combined=float(params.get("recover_min_reid_for_combined", 0.45)),
            recover_min_color=float(params.get("recover_min_color", 0.20)),
            color_weight=float(params.get("color_weight", 0.30)),
            reid_weight=float(params.get("reid_weight", 0.70)),
            min_bbox_area=int(params.get("min_bbox_area", 1200)),
        )

    def reset_target(self) -> None:
        self.target_manager.reset()

    def _angle_from_bbox(self, target: Optional[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Optional[float]:
        if target is None:
            return None
        h, w = frame_shape[:2]
        cx, _ = target.center
        normalized = (cx - (w * 0.5)) / max(1.0, w * 0.5)
        return normalized * (self.camera_fov_deg * 0.5)

    def _make_payload(self, result: TargetResult, tracks: List[TrackCandidate], frame: np.ndarray) -> Dict[str, Any]:
        target = result.target
        angle = self._angle_from_bbox(target, frame.shape)
        payload: Dict[str, Any] = {
            "target_found": target is not None,
            "status": result.status,
            "target_track_id": result.target_track_id,
            "recovered": result.recovered,
            "reid_similarity": round(float(result.reid_similarity), 4),
            "color_similarity": round(float(result.color_similarity), 4),
            "combined_similarity": round(float(result.combined_similarity), 4),
            "verified": bool(result.verified),
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
            payload.update({
                "bbox": None,
                "bbox_center": None,
                "bbox_area": 0,
                "det_conf": 0.0,
            })
        return payload

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any], TargetResult, List[TrackCandidate]]:
        tracks = self.pipeline.detect_and_track(frame)
        result = self.target_manager.update(frame, tracks, self.pipeline.extract_features, now=time.time())
        debug = draw_debug_frame(frame, tracks, result, draw_all_tracks=self.draw_all_tracks)
        payload = self._make_payload(result, tracks, frame)
        return debug, payload, result, tracks
