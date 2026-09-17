from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

from .reid_gallery import ColorGallery, ReIDGallery
from .types import BBox, TrackCandidate

FeatureExtractorFn = Callable[[np.ndarray, List[BBox]], List[Optional[np.ndarray]]]


@dataclass
class TargetResult:
    status: str
    target: Optional[TrackCandidate]
    target_track_id: Optional[int]
    reid_similarity: float = -1.0
    color_similarity: float = -1.0
    combined_similarity: float = -1.0
    recovered: bool = False
    verified: bool = False
    lost_age_sec: float = 0.0
    reason: str = ""


class TargetLockManager:
    """
    Keeps one stable target even when DeepSORT changes its numeric track ID.

    Important behavior in this version:
    - It does not blindly trust a DeepSORT numeric ID forever.
    - While the target ID is visible, it periodically verifies that the crop
      still looks like the saved target using ReID + color histogram.
    - If the current ID suddenly looks like another person, it returns
      TRACK_ID_SUSPECT/LOST instead of jumping the target box to that person.
    """

    def __init__(
        self,
        min_stable_frames: int = 3,
        gallery_size: int = 30,
        reid_threshold: float = 0.78,
        reid_margin: float = 0.10,
        max_lost_sec: float = 5.0,
        spatial_gate_px: float = 220.0,
        select_mode: str = "largest_box",
        update_gallery_every_n_frames: int = 5,
        # Continuous verification of the current DeepSORT ID.
        verify_current_track: bool = True,
        verify_every_n_frames: int = 5,
        current_min_reid: float = 0.42,
        current_min_color: float = 0.18,
        current_min_combined: float = 0.48,
        verification_fail_limit: int = 2,
        # Recovery scoring.
        reid_match_mode: str = "max",
        recover_min_combined: float = 0.66,
        recover_min_reid_for_combined: float = 0.45,
        recover_min_color: float = 0.20,
        color_weight: float = 0.30,
        reid_weight: float = 0.70,
        min_bbox_area: int = 1200,
    ) -> None:
        self.min_stable_frames = int(min_stable_frames)
        self.reid_threshold = float(reid_threshold)
        self.reid_margin = float(reid_margin)
        self.max_lost_sec = float(max_lost_sec)
        self.spatial_gate_px = float(spatial_gate_px)
        self.select_mode = select_mode
        self.update_gallery_every_n_frames = max(1, int(update_gallery_every_n_frames))

        self.verify_current_track = bool(verify_current_track)
        self.verify_every_n_frames = max(1, int(verify_every_n_frames))
        self.current_min_reid = float(current_min_reid)
        self.current_min_color = float(current_min_color)
        self.current_min_combined = float(current_min_combined)
        self.verification_fail_limit = max(1, int(verification_fail_limit))

        self.reid_match_mode = str(reid_match_mode)
        self.recover_min_combined = float(recover_min_combined)
        self.recover_min_reid_for_combined = float(recover_min_reid_for_combined)
        self.recover_min_color = float(recover_min_color)
        self.color_weight = float(color_weight)
        self.reid_weight = float(reid_weight)
        total = self.color_weight + self.reid_weight
        if total <= 1e-9:
            self.color_weight = 0.3
            self.reid_weight = 0.7
        else:
            self.color_weight /= total
            self.reid_weight /= total
        self.min_bbox_area = int(min_bbox_area)

        self.target_track_id: Optional[int] = None
        self.target_bbox: Optional[BBox] = None
        self.last_seen_time: Optional[float] = None
        self.last_seen_frame_index: int = 0
        self.frame_index: int = 0
        self.gallery = ReIDGallery(max_size=gallery_size)
        self.color_gallery = ColorGallery(max_size=max(10, gallery_size // 2))
        self.track_frames: Dict[int, int] = {}
        self.current_verify_failures: int = 0

    def reset(self) -> None:
        self.target_track_id = None
        self.target_bbox = None
        self.last_seen_time = None
        self.last_seen_frame_index = 0
        self.frame_index = 0
        self.gallery.clear()
        self.color_gallery.clear()
        self.track_frames.clear()
        self.current_verify_failures = 0

    def _update_track_counts(self, tracks: Iterable[TrackCandidate]) -> None:
        current_ids = {int(t.track_id) for t in tracks}
        for tid in current_ids:
            self.track_frames[tid] = self.track_frames.get(tid, 0) + 1
        for tid in list(self.track_frames.keys()):
            if tid not in current_ids and tid != self.target_track_id:
                self.track_frames[tid] = max(0, self.track_frames[tid] - 1)
                if self.track_frames[tid] == 0:
                    del self.track_frames[tid]

    def _stable_tracks(self, tracks: List[TrackCandidate]) -> List[TrackCandidate]:
        return [
            t for t in tracks
            if self.track_frames.get(int(t.track_id), 0) >= self.min_stable_frames
            and int(t.area) >= self.min_bbox_area
        ]

    def _select_initial_target(self, tracks: List[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Optional[TrackCandidate]:
        stable = self._stable_tracks(tracks)
        if not stable:
            return None
        if self.select_mode == "center_largest":
            h, w = frame_shape[:2]
            cx0 = w * 0.5

            def score(t: TrackCandidate) -> float:
                cx, _ = t.center
                center_penalty = abs(cx - cx0) / max(1.0, cx0)
                return float(t.area) * (1.0 - 0.35 * center_penalty)

            return max(stable, key=score)
        return max(stable, key=lambda t: t.area)

    @staticmethod
    def _bbox_center_distance(a: BBox, b: BBox) -> float:
        ax = (a[0] + a[2]) * 0.5
        ay = (a[1] + a[3]) * 0.5
        bx = (b[0] + b[2]) * 0.5
        by = (b[1] + b[3]) * 0.5
        return math.hypot(ax - bx, ay - by)

    @staticmethod
    def _color_histogram(frame: np.ndarray, bbox: BBox) -> Optional[np.ndarray]:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(w - 1, int(x1)))
        x2 = max(0, min(w - 1, int(x2)))
        y1 = max(0, min(h - 1, int(y1)))
        y2 = max(0, min(h - 1, int(y2)))
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)

        # Use the torso/center area to reduce background, face, and floor pixels.
        xx1 = x1 + int(0.18 * bw)
        xx2 = x2 - int(0.18 * bw)
        yy1 = y1 + int(0.18 * bh)
        yy2 = y1 + int(0.72 * bh)
        if xx2 <= xx1 + 4 or yy2 <= yy1 + 4:
            return None
        crop = frame[yy1:yy2, xx1:xx2]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [24, 16], [0, 180, 0, 256]).astype(np.float32)
        s = float(hist.sum())
        if s <= 1e-12:
            return None
        return (hist / s).reshape(-1)

    @staticmethod
    def _clip01(value: float) -> float:
        if value < 0:
            return 0.0
        if value > 1:
            return 1.0
        return float(value)

    def _combined_score(self, reid_sim: float, color_sim: float) -> float:
        r = self._clip01(reid_sim)
        c = self._clip01(color_sim)
        return self.reid_weight * r + self.color_weight * c

    def _target_scores(
        self,
        feature: Optional[np.ndarray],
        color_hist: Optional[np.ndarray],
    ) -> Tuple[float, float, float]:
        reid_sim = self.gallery.similarity(feature, mode=self.reid_match_mode) if feature is not None else -1.0
        color_sim = self.color_gallery.similarity(color_hist) if color_hist is not None else -1.0
        return reid_sim, color_sim, self._combined_score(reid_sim, color_sim)

    def _extract_one(
        self,
        frame: np.ndarray,
        target: TrackCandidate,
        extract_features: FeatureExtractorFn,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        features = extract_features(frame, [target.bbox])
        feature = features[0] if features else None
        color_hist = self._color_histogram(frame, target.bbox)
        return feature, color_hist

    def _add_target_signature(
        self,
        frame: np.ndarray,
        target: TrackCandidate,
        extract_features: FeatureExtractorFn,
        force: bool = False,
        feature: Optional[np.ndarray] = None,
        color_hist: Optional[np.ndarray] = None,
    ) -> None:
        if not force and (self.frame_index - self.last_seen_frame_index) % self.update_gallery_every_n_frames != 0:
            return
        if feature is None or color_hist is None:
            feature2, color2 = self._extract_one(frame, target, extract_features)
            feature = feature if feature is not None else feature2
            color_hist = color_hist if color_hist is not None else color2
        if feature is not None:
            target.feature = feature
            self.gallery.add(feature)
        if color_hist is not None:
            self.color_gallery.add(color_hist)

    def _verify_current_target(
        self,
        frame: np.ndarray,
        target: TrackCandidate,
        extract_features: FeatureExtractorFn,
    ) -> Tuple[bool, float, float, float, Optional[np.ndarray], Optional[np.ndarray], str]:
        if not self.verify_current_track:
            return True, -1.0, -1.0, -1.0, None, None, "verification_disabled"
        if not self.gallery.ready:
            return True, -1.0, -1.0, -1.0, None, None, "gallery_not_ready"
        if (self.frame_index % self.verify_every_n_frames) != 0:
            return True, -1.0, -1.0, -1.0, None, None, "not_verify_frame"

        feature, color_hist = self._extract_one(frame, target, extract_features)
        reid_sim, color_sim, combined = self._target_scores(feature, color_hist)
        ok = (
            combined >= self.current_min_combined
            or reid_sim >= max(self.current_min_reid, self.reid_threshold - 0.20)
            or (reid_sim >= self.current_min_reid and color_sim >= self.current_min_color)
        )
        reason = f"verify reid={reid_sim:.3f}, color={color_sim:.3f}, combined={combined:.3f}"
        return ok, reid_sim, color_sim, combined, feature, color_hist, reason

    def _score_candidates(
        self,
        frame: np.ndarray,
        candidates: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
    ) -> List[Tuple[float, float, float, TrackCandidate, Optional[np.ndarray], Optional[np.ndarray]]]:
        features = extract_features(frame, [t.bbox for t in candidates])
        scored: List[Tuple[float, float, float, TrackCandidate, Optional[np.ndarray], Optional[np.ndarray]]] = []
        for t, f in zip(candidates, features):
            color_hist = self._color_histogram(frame, t.bbox)
            reid_sim, color_sim, combined = self._target_scores(f, color_hist)
            scored.append((combined, reid_sim, color_sim, t, f, color_hist))
        scored.sort(key=lambda item: item[0], reverse=True)
        return scored

    def _try_reid_recover(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        lost_age: float,
        exclude_ids: Optional[set[int]] = None,
    ) -> TargetResult:
        if not self.gallery.ready or not tracks:
            return TargetResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="no_gallery_or_tracks")

        exclude_ids = exclude_ids or set()
        candidates = [t for t in (self._stable_tracks(tracks) or tracks) if int(t.track_id) not in exclude_ids and int(t.area) >= self.min_bbox_area]
        if not candidates:
            return TargetResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="no_candidate_after_exclude")

        gated: List[TrackCandidate] = []
        for t in candidates:
            if self.target_bbox is None or self.spatial_gate_px <= 0:
                gated.append(t)
                continue
            gate = self.spatial_gate_px * (1.0 + 0.35 * min(lost_age, 4.0))
            if self._bbox_center_distance(t.bbox, self.target_bbox) <= gate:
                gated.append(t)

        # After the camera has been away for a moment, allow global ReID search.
        if not gated and lost_age > 0.8:
            gated = candidates
        if not gated:
            return TargetResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="spatial_gate_rejected")

        scored = self._score_candidates(frame, gated, extract_features)
        if not scored:
            return TargetResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="no_scores")

        best_combined, best_reid, best_color, best_track, best_feature, best_color_hist = scored[0]
        second_combined = scored[1][0] if len(scored) > 1 else -1.0
        second_reid = scored[1][1] if len(scored) > 1 else -1.0
        combined_margin = best_combined - second_combined
        reid_margin = best_reid - second_reid

        accept_by_reid = best_reid >= self.reid_threshold and reid_margin >= self.reid_margin
        accept_by_combined = (
            best_combined >= self.recover_min_combined
            and combined_margin >= max(0.04, self.reid_margin * 0.6)
            and best_reid >= self.recover_min_reid_for_combined
            and best_color >= self.recover_min_color
        )
        very_confident = best_reid >= (self.reid_threshold + 0.10) and best_combined >= (self.recover_min_combined - 0.05)

        if accept_by_reid or accept_by_combined or very_confident:
            self.target_track_id = int(best_track.track_id)
            self.target_bbox = best_track.bbox
            self.last_seen_time = time.time()
            self.last_seen_frame_index = self.frame_index
            self.current_verify_failures = 0
            if best_feature is not None:
                best_track.feature = best_feature
                self.gallery.add(best_feature)
            if best_color_hist is not None:
                self.color_gallery.add(best_color_hist)
            return TargetResult(
                "REID_RECOVERED",
                best_track,
                self.target_track_id,
                reid_similarity=best_reid,
                color_similarity=best_color,
                combined_similarity=best_combined,
                recovered=True,
                verified=True,
                lost_age_sec=lost_age,
                reason=f"best={best_combined:.3f}, second={second_combined:.3f}, reid={best_reid:.3f}, color={best_color:.3f}",
            )

        return TargetResult(
            "LOST",
            None,
            self.target_track_id,
            reid_similarity=best_reid,
            color_similarity=best_color,
            combined_similarity=best_combined,
            lost_age_sec=lost_age,
            reason=f"best={best_combined:.3f}, second={second_combined:.3f}, reid={best_reid:.3f}, color={best_color:.3f}",
        )

    def update(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        now: Optional[float] = None,
    ) -> TargetResult:
        now = time.time() if now is None else float(now)
        self.frame_index += 1
        self._update_track_counts(tracks)
        track_by_id = {int(t.track_id): t for t in tracks}

        if self.target_track_id is None:
            selected = self._select_initial_target(tracks, frame.shape)
            if selected is None:
                return TargetResult("WAITING_FOR_STABLE_TARGET", None, None, reason="no_stable_track")
            self.target_track_id = int(selected.track_id)
            self.target_bbox = selected.bbox
            self.last_seen_time = now
            self.last_seen_frame_index = self.frame_index
            self.current_verify_failures = 0
            self._add_target_signature(frame, selected, extract_features, force=True)
            return TargetResult("TARGET_LOCKED", selected, self.target_track_id, verified=True, reason="initial_lock")

        if self.target_track_id in track_by_id:
            target = track_by_id[self.target_track_id]
            ok, reid_sim, color_sim, combined, feature, color_hist, reason = self._verify_current_target(frame, target, extract_features)
            if not ok:
                self.current_verify_failures += 1
                if self.current_verify_failures >= self.verification_fail_limit:
                    # Do not follow this box. It is likely an ID switch.
                    lost_age = 0.0 if self.last_seen_time is None else max(0.0, now - self.last_seen_time)
                    recovered = self._try_reid_recover(
                        frame,
                        tracks,
                        extract_features,
                        lost_age=lost_age,
                        exclude_ids={int(self.target_track_id)},
                    )
                    if recovered.target is not None:
                        return recovered
                    return TargetResult(
                        "TRACK_ID_SUSPECT",
                        None,
                        self.target_track_id,
                        reid_similarity=reid_sim,
                        color_similarity=color_sim,
                        combined_similarity=combined,
                        lost_age_sec=lost_age,
                        reason=reason + f", failures={self.current_verify_failures}",
                    )
                return TargetResult(
                    "TRACK_VERIFY_WARN",
                    target,
                    self.target_track_id,
                    reid_similarity=reid_sim,
                    color_similarity=color_sim,
                    combined_similarity=combined,
                    verified=False,
                    reason=reason + f", failures={self.current_verify_failures}",
                )

            self.current_verify_failures = 0
            self.target_bbox = target.bbox
            self.last_seen_time = now
            self.last_seen_frame_index = self.frame_index
            # Only update the gallery after a verified frame, otherwise a wrong ID can poison memory.
            self._add_target_signature(
                frame,
                target,
                extract_features,
                force=False,
                feature=feature,
                color_hist=color_hist,
            )
            return TargetResult(
                "TRACKING",
                target,
                self.target_track_id,
                reid_similarity=reid_sim,
                color_similarity=color_sim,
                combined_similarity=combined,
                verified=True,
                reason=reason,
            )

        lost_age = 0.0 if self.last_seen_time is None else max(0.0, now - self.last_seen_time)
        if lost_age <= self.max_lost_sec:
            return self._try_reid_recover(frame, tracks, extract_features, lost_age)

        old_id = self.target_track_id
        self.target_track_id = None
        self.current_verify_failures = 0
        return TargetResult("TARGET_TIMEOUT", None, old_id, lost_age_sec=lost_age, reason="lost_too_long")
