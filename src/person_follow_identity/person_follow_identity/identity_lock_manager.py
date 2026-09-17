from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

import numpy as np

from person_reid_tracker.types import BBox, TrackCandidate

from .identity_gallery import ColorGallery, FeatureGallery, person_color_histogram
from .gait_identity import GaitIdentityVerifier, GaitScore

FeatureExtractorFn = Callable[[np.ndarray, List[BBox]], List[Optional[np.ndarray]]]


@dataclass
class IdentityResult:
    status: str
    target: Optional[TrackCandidate]
    target_track_id: Optional[int]
    reid_similarity: float = -1.0
    color_similarity: float = -1.0
    negative_similarity: float = -1.0
    identity_margin: float = -1.0
    combined_similarity: float = -1.0
    gait_similarity: float = -1.0
    body_similarity: float = -1.0
    silhouette_similarity: float = -1.0
    gait_ready: bool = False
    live_gait_ready: bool = False
    second_similarity: float = -1.0
    recovered: bool = False
    verified: bool = False
    lost_age_sec: float = 0.0
    enrolled_samples: int = 0
    negative_samples: int = 0
    enrollment_progress: float = 0.0
    reason: str = ""


class TargetIdentityManager:
    """Strict single-person identity lock.

    The manager never picks a new person after the target is lost. It only locks
    when the operator starts enrollment. After enrollment, a candidate must pass
    ReID, color, negative-gallery, and best-vs-second margin checks.
    """

    def __init__(
        self,
        min_stable_frames: int = 4,
        min_bbox_area: int = 1800,
        select_mode: str = "center_largest",
        auto_enroll_on_start: bool = False,
        enroll_seconds: float = 30.0,
        enroll_min_samples: int = 80,
        enroll_sample_every_n_frames: int = 3,
        positive_gallery_size: int = 160,
        negative_gallery_size: int = 160,
        color_gallery_size: int = 100,
        reid_match_mode: str = "topk_mean",
        reid_top_k: int = 5,
        reid_threshold: float = 0.82,
        current_min_reid: float = 0.60,
        current_min_combined: float = 0.64,
        current_min_color: float = 0.28,
        recover_min_combined: float = 0.78,
        recover_min_reid: float = 0.68,
        recover_min_color: float = 0.25,
        reid_margin: float = 0.16,
        combined_margin: float = 0.12,
        negative_margin: float = 0.18,
        verification_fail_limit: int = 1,
        verify_every_n_frames: int = 4,
        update_gallery_every_n_frames: int = 4,
        target_update_min_similarity: float = 0.62,
        max_duplicate_similarity: float = 0.985,
        negative_add_max_reid: float = 0.55,
        negative_add_min_margin: float = 0.12,
        spatial_gate_px: float = 220.0,
        global_search_after_sec: float = 2.5,
        global_search_min_reid: float = 0.90,
        enable_global_recovery: bool = False,
        allow_single_candidate_global_recovery: bool = False,
        recovery_confirm_frames: int = 12,
        recovery_single_candidate_min_reid: float = 0.94,
        recovery_single_candidate_min_combined: float = 0.88,
        max_lost_keep_id_sec: float = 2.0,
        reid_weight: float = 0.70,
        color_weight: float = 0.20,
        gait_identity_weight: float = 0.10,
        enable_gait: bool = True,
        gait_required_for_recovery: bool = True,
        gait_required_for_enrollment: bool = False,
        gait_sample_every_n_frames: int = 2,
        gait_window: int = 45,
        gait_min_frames: int = 18,
        gait_gallery_size: int = 80,
        gait_current_min_score: float = 0.50,
        gait_recovery_min_score: float = 0.58,
        gait_model_complexity: int = 1,
        gait_min_detection_confidence: float = 0.45,
        gait_min_tracking_confidence: float = 0.45,
        gait_min_landmark_visibility: float = 0.25,
    ) -> None:
        self.min_stable_frames = int(min_stable_frames)
        self.min_bbox_area = int(min_bbox_area)
        self.select_mode = str(select_mode)
        self.auto_enroll_on_start = bool(auto_enroll_on_start)
        self.enroll_seconds = float(enroll_seconds)
        self.enroll_min_samples = int(enroll_min_samples)
        self.enroll_sample_every_n_frames = max(1, int(enroll_sample_every_n_frames))
        self.reid_match_mode = str(reid_match_mode)
        self.reid_top_k = max(1, int(reid_top_k))
        self.reid_threshold = float(reid_threshold)
        self.current_min_reid = float(current_min_reid)
        self.current_min_combined = float(current_min_combined)
        self.current_min_color = float(current_min_color)
        self.recover_min_combined = float(recover_min_combined)
        self.recover_min_reid = float(recover_min_reid)
        self.recover_min_color = float(recover_min_color)
        self.reid_margin = float(reid_margin)
        self.combined_margin = float(combined_margin)
        self.negative_margin = float(negative_margin)
        self.verification_fail_limit = max(1, int(verification_fail_limit))
        self.verify_every_n_frames = max(1, int(verify_every_n_frames))
        self.update_gallery_every_n_frames = max(1, int(update_gallery_every_n_frames))
        self.target_update_min_similarity = float(target_update_min_similarity)
        self.max_duplicate_similarity = float(max_duplicate_similarity)
        self.negative_add_max_reid = float(negative_add_max_reid)
        self.negative_add_min_margin = float(negative_add_min_margin)
        self.spatial_gate_px = float(spatial_gate_px)
        self.global_search_after_sec = float(global_search_after_sec)
        self.global_search_min_reid = float(global_search_min_reid)
        self.enable_global_recovery = bool(enable_global_recovery)
        self.allow_single_candidate_global_recovery = bool(allow_single_candidate_global_recovery)
        self.recovery_confirm_frames = max(1, int(recovery_confirm_frames))
        self.recovery_single_candidate_min_reid = float(recovery_single_candidate_min_reid)
        self.recovery_single_candidate_min_combined = float(recovery_single_candidate_min_combined)
        self.max_lost_keep_id_sec = float(max_lost_keep_id_sec)
        self.gait_identity_weight = max(0.0, min(0.80, float(gait_identity_weight)))
        total = max(1e-9, float(reid_weight) + float(color_weight))
        self.reid_weight = float(reid_weight) / total
        self.color_weight = float(color_weight) / total
        self.gait_sample_every_n_frames = max(1, int(gait_sample_every_n_frames))
        self.gait_current_min_score = float(gait_current_min_score)
        self.gait_recovery_min_score = float(gait_recovery_min_score)
        self.gait = GaitIdentityVerifier(
            enabled=bool(enable_gait),
            required_for_recovery=bool(gait_required_for_recovery),
            required_for_enrollment=bool(gait_required_for_enrollment),
            window=int(gait_window),
            min_frames=int(gait_min_frames),
            gallery_size=int(gait_gallery_size),
            model_complexity=int(gait_model_complexity),
            min_detection_confidence=float(gait_min_detection_confidence),
            min_tracking_confidence=float(gait_min_tracking_confidence),
            min_landmark_visibility=float(gait_min_landmark_visibility),
        )

        self.positive_gallery = FeatureGallery(max_size=positive_gallery_size)
        self.negative_gallery = FeatureGallery(max_size=negative_gallery_size)
        self.color_gallery = ColorGallery(max_size=color_gallery_size)

        self.frame_index = 0
        self.track_frames: Dict[int, int] = {}
        self.target_track_id: Optional[int] = None
        self.target_bbox: Optional[BBox] = None
        self.last_seen_time: Optional[float] = None
        self.current_verify_failures = 0
        self.pending_recovery_track_id: Optional[int] = None
        self.pending_recovery_hits = 0

        self.enrolling = False
        self.enrollment_start_time: Optional[float] = None
        self.enrollment_track_id: Optional[int] = None
        self._auto_enroll_started = False

    @property
    def identity_ready(self) -> bool:
        if len(self.positive_gallery) < self.enroll_min_samples:
            return False
        if self.gait.enabled and self.gait.required_for_enrollment:
            return self.gait.target_ready
        return True

    def reset(self) -> None:
        self.positive_gallery.clear()
        self.negative_gallery.clear()
        self.color_gallery.clear()
        self.gait.clear()
        self.frame_index = 0
        self.track_frames.clear()
        self.target_track_id = None
        self.target_bbox = None
        self.last_seen_time = None
        self.current_verify_failures = 0
        self.pending_recovery_track_id = None
        self.pending_recovery_hits = 0
        self.enrolling = False
        self.enrollment_start_time = None
        self.enrollment_track_id = None
        self._auto_enroll_started = False

    def start_enrollment(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else float(now)
        # Keep negative memory optional? For a clean new target, reset all memory.
        self.positive_gallery.clear()
        self.negative_gallery.clear()
        self.color_gallery.clear()
        self.gait.clear()
        self.target_track_id = None
        self.target_bbox = None
        self.last_seen_time = None
        self.current_verify_failures = 0
        self.pending_recovery_track_id = None
        self.pending_recovery_hits = 0
        self.enrolling = True
        self.enrollment_start_time = now
        self.enrollment_track_id = None

    def finish_enrollment(self) -> bool:
        # Manual finish is allowed only after enough clean samples. If not ready,
        # stay in enrollment mode instead of accidentally falling back to an
        # incomplete identity memory.
        self.gait.finalize_target()
        if not self.identity_ready:
            return False

        # IMPORTANT FIX:
        # When the operator presses finish while the enrolled person is still
        # visible, keep that DeepSORT track as the locked target. The previous
        # version set target_track_id=None, so the very next frame entered strict
        # recovery mode and could reject the same person because of margin/negative
        # gates. This is why the UI showed identity_ready=True but
        # LOST_REJECTED_CANDIDATE for the original person.
        keep_target_id = self.enrollment_track_id

        self.enrolling = False
        self.enrollment_start_time = None
        self.enrollment_track_id = None
        self.target_track_id = int(keep_target_id) if keep_target_id is not None else None
        self.current_verify_failures = 0
        self.pending_recovery_track_id = None
        self.pending_recovery_hits = 0
        return True

    def _progress(self, now: float) -> float:
        sample_p = len(self.positive_gallery) / max(1.0, float(self.enroll_min_samples))
        if self.enrollment_start_time is None or self.enroll_seconds <= 0:
            time_p = sample_p
        else:
            time_p = (now - self.enrollment_start_time) / self.enroll_seconds
        progress = min(sample_p, time_p)
        if self.gait.enabled and self.gait.required_for_enrollment:
            progress = min(progress, self.gait.progress())
        return float(max(0.0, min(1.0, progress)))

    def _update_track_counts(self, tracks: Iterable[TrackCandidate]) -> None:
        current_ids = {int(t.track_id) for t in tracks}
        for tid in current_ids:
            self.track_frames[tid] = self.track_frames.get(tid, 0) + 1
        for tid in list(self.track_frames.keys()):
            if tid not in current_ids and tid not in {self.target_track_id, self.enrollment_track_id}:
                self.track_frames[tid] = max(0, self.track_frames[tid] - 1)
                if self.track_frames[tid] == 0:
                    del self.track_frames[tid]
        self.gait.prune_tracks(list(current_ids))

    def _stable_tracks(self, tracks: List[TrackCandidate]) -> List[TrackCandidate]:
        return [
            t for t in tracks
            if self.track_frames.get(int(t.track_id), 0) >= self.min_stable_frames
            and int(t.area) >= self.min_bbox_area
        ]

    def _select_enrollment_target(self, tracks: List[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Optional[TrackCandidate]:
        stable = self._stable_tracks(tracks)
        if not stable:
            return None
        if self.enrollment_track_id is not None:
            for t in stable:
                if int(t.track_id) == self.enrollment_track_id:
                    return t
            # Do NOT switch enrollment to another person automatically.
            # If the selected person leaves during enrollment, wait until that
            # same DeepSORT track returns or the operator presses r/e again.
            return None
        h, w = frame_shape[:2]
        cx0 = w * 0.5
        cy0 = h * 0.55

        def score(t: TrackCandidate) -> float:
            cx, cy = t.center
            center_penalty = abs(cx - cx0) / max(1.0, cx0) + 0.25 * abs(cy - cy0) / max(1.0, cy0)
            return float(t.area) * (1.0 - 0.40 * center_penalty)

        if self.select_mode == "largest_box":
            selected = max(stable, key=lambda t: t.area)
        else:
            selected = max(stable, key=score)
        self.enrollment_track_id = int(selected.track_id)
        return selected

    @staticmethod
    def _bbox_center_distance(a: BBox, b: BBox) -> float:
        ax = (a[0] + a[2]) * 0.5
        ay = (a[1] + a[3]) * 0.5
        bx = (b[0] + b[2]) * 0.5
        by = (b[1] + b[3]) * 0.5
        return math.hypot(ax - bx, ay - by)

    def _combined_score(self, reid_sim: float, color_sim: float, gait_score: Optional[GaitScore] = None) -> float:
        r = 0.0 if reid_sim < 0 else min(1.0, max(0.0, reid_sim))
        c = 0.0 if color_sim < 0 else min(1.0, max(0.0, color_sim))
        base = self.reid_weight * r + self.color_weight * c
        if gait_score is None or gait_score.combined_similarity < 0:
            return float(base)
        # Blend gait/body/silhouette into the identity score without letting it
        # completely override strong ReID/color evidence.
        g = min(1.0, max(0.0, gait_score.combined_similarity))
        w = self.gait_identity_weight
        return float((1.0 - w) * base + w * g)

    def _extract_many(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        with_gait: bool = True,
    ) -> List[Tuple[TrackCandidate, Optional[np.ndarray], Optional[np.ndarray], Optional[GaitScore]]]:
        if not tracks:
            return []
        features = extract_features(frame, [t.bbox for t in tracks])
        out: List[Tuple[TrackCandidate, Optional[np.ndarray], Optional[np.ndarray], Optional[GaitScore]]] = []
        for t, f in zip(tracks, features):
            gait_score = self.gait.score_candidate(frame, t.bbox, int(t.track_id)) if with_gait and self.gait.enabled else None
            out.append((t, f, person_color_histogram(frame, t.bbox), gait_score))
        return out

    def _scores(
        self,
        feature: Optional[np.ndarray],
        color_hist: Optional[np.ndarray],
        gait_score: Optional[GaitScore] = None,
    ) -> Tuple[float, float, float, float, float, float, float, float, bool]:
        pos = self.positive_gallery.similarity(feature, mode=self.reid_match_mode, top_k=self.reid_top_k)
        neg = self.negative_gallery.similarity(feature, mode="max")
        color = self.color_gallery.similarity(color_hist, mode="topk_mean", top_k=self.reid_top_k)
        margin_to_negative = pos - neg if neg >= 0 else 1.0
        combined = self._combined_score(pos, color, gait_score)
        gait_sim = -1.0 if gait_score is None else gait_score.combined_similarity
        body_sim = -1.0 if gait_score is None else gait_score.body_similarity
        silh_sim = -1.0 if gait_score is None else gait_score.silhouette_similarity
        live_ready = False if gait_score is None else gait_score.live_gait_ready
        return pos, color, neg, margin_to_negative, combined, gait_sim, body_sim, silh_sim, live_ready

    def _maybe_add_negative(self, feature: Optional[np.ndarray], pos_sim: float, neg_sim: float) -> None:
        if feature is None:
            return
        # Only learn obvious non-targets. Ambiguous people are not added because
        # they may be the true target under a different ID/viewpoint.
        if pos_sim <= self.negative_add_max_reid:
            self.negative_gallery.add_diverse(feature, max_similarity_to_last=self.max_duplicate_similarity)

    def _handle_enrollment(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        now: float,
    ) -> IdentityResult:
        target = self._select_enrollment_target(tracks, frame.shape)
        if target is None:
            return IdentityResult(
                "ENROLLING_WAITING_FOR_PERSON",
                None,
                None,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=self._progress(now),
                reason="stand alone near camera center; no stable person yet",
            )

        stable = self._stable_tracks(tracks)
        extracted = self._extract_many(frame, stable, extract_features, with_gait=False)
        target_feature: Optional[np.ndarray] = None
        target_color: Optional[np.ndarray] = None
        for t, f, c, _g in extracted:
            if int(t.track_id) == int(target.track_id):
                target_feature, target_color = f, c
                if (self.frame_index % self.enroll_sample_every_n_frames) == 0:
                    self.positive_gallery.add_diverse(
                        f,
                        min_similarity_to_accept=-1.0 if len(self.positive_gallery) < 5 else 0.35,
                        max_similarity_to_last=self.max_duplicate_similarity,
                    )
                    self.color_gallery.add(c)
                if self.gait.enabled and (self.frame_index % self.gait_sample_every_n_frames) == 0:
                    self.gait.add_target_sample(frame, target.bbox)
            else:
                # Other people seen during enrollment become known non-targets.
                self.negative_gallery.add_diverse(f, max_similarity_to_last=self.max_duplicate_similarity)

        self.target_bbox = target.bbox
        self.last_seen_time = now
        progress = self._progress(now)
        enough_samples = len(self.positive_gallery) >= self.enroll_min_samples
        enough_time = self.enrollment_start_time is not None and (now - self.enrollment_start_time) >= self.enroll_seconds
        if enough_samples and enough_time:
            self.gait.finalize_target()
            if self.gait.enabled and self.gait.required_for_enrollment and not self.gait.target_ready:
                return IdentityResult(
                    "ENROLLING_NEED_GAIT",
                    target,
                    int(target.track_id),
                    verified=True,
                    enrolled_samples=len(self.positive_gallery),
                    negative_samples=len(self.negative_gallery),
                    enrollment_progress=self._progress(now),
                    gait_ready=self.gait.target_ready,
                    reason=f"appearance ready but gait needs motion samples {self.gait.target_sample_count}/{self.gait.min_frames}; ask target to walk slowly",
                )
            self.enrolling = False
            self.enrollment_start_time = None
            self.enrollment_track_id = None
            self.target_track_id = int(target.track_id)
            self.current_verify_failures = 0
            pos, color, neg, ident_margin, combined, gait_sim, body_sim, silh_sim, live_ready = self._scores(target_feature, target_color, None)
            return IdentityResult(
                "ENROLLMENT_DONE",
                target,
                self.target_track_id,
                reid_similarity=pos,
                color_similarity=color,
                negative_similarity=neg,
                identity_margin=ident_margin,
                combined_similarity=combined,
                gait_similarity=gait_sim,
                body_similarity=body_sim,
                silhouette_similarity=silh_sim,
                gait_ready=self.gait.target_ready,
                live_gait_ready=live_ready,
                verified=True,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=1.0,
                reason="target identity memory is ready",
            )

        return IdentityResult(
            "ENROLLING",
            target,
            int(target.track_id),
            verified=True,
            enrolled_samples=len(self.positive_gallery),
            negative_samples=len(self.negative_gallery),
            enrollment_progress=progress,
            gait_ready=self.gait.target_ready,
            live_gait_ready=False,
            reason=(
                f"collecting target samples; app={len(self.positive_gallery)}/{self.enroll_min_samples}, "
                f"gait={self.gait.target_sample_count}/{self.gait.min_frames}, gait_status={self.gait.reason}"
            ),
        )

    def _score_candidates(
        self,
        frame: np.ndarray,
        candidates: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
    ) -> List[Tuple[float, float, float, float, float, float, float, float, bool, TrackCandidate, Optional[np.ndarray], Optional[np.ndarray], Optional[GaitScore]]]:
        scored = []
        for t, f, c, g in self._extract_many(frame, candidates, extract_features, with_gait=True):
            pos, color, neg, ident_margin, combined, gait_sim, body_sim, silh_sim, live_ready = self._scores(f, c, g)
            scored.append((combined, pos, color, neg, ident_margin, gait_sim, body_sim, silh_sim, live_ready, t, f, c, g))
        scored.sort(key=lambda x: x[0], reverse=True)
        return scored

    def _accept_recovery(
        self,
        scored: List[Tuple[float, float, float, float, float, float, float, float, bool, TrackCandidate, Optional[np.ndarray], Optional[np.ndarray], Optional[GaitScore]]],
        using_global: bool,
    ) -> Tuple[bool, str]:
        if not scored:
            return False, "no_scores"
        best_combined, best_reid, best_color, best_neg, ident_margin, best_gait, best_body, best_silh, live_gait_ready, _track, _feature, _color, _gait_score = scored[0]
        has_second = len(scored) > 1
        second_combined = scored[1][0] if has_second else -1.0
        second_reid = scored[1][1] if has_second else -1.0

        # Important: when A is outside the camera and only B is visible, there
        # is no second candidate. The old code treated margin vs -1 as huge and
        # could accept B. Here, single-candidate recovery is considered risky.
        c_margin = best_combined - second_combined if has_second else 0.0
        r_margin = best_reid - second_reid if has_second else 0.0

        if using_global and not self.enable_global_recovery:
            return False, (
                f"global_recovery_disabled; best={best_combined:.3f}, "
                f"reid={best_reid:.3f}, color={best_color:.3f}, "
                f"neg={best_neg:.3f}, id_margin={ident_margin:.3f}"
            )

        if using_global and not has_second and not self.allow_single_candidate_global_recovery:
            return False, (
                f"single_candidate_global_rejected; best={best_combined:.3f}, "
                f"reid={best_reid:.3f}, color={best_color:.3f}, "
                f"neg={best_neg:.3f}, id_margin={ident_margin:.3f}"
            )

        min_reid = self.recover_min_reid
        min_combined = self.recover_min_combined
        min_color = self.recover_min_color
        min_negative_margin = self.negative_margin

        if not has_second:
            # One visible person after loss is exactly the dangerous case.
            # Require much stronger evidence.
            min_reid = max(min_reid, self.recovery_single_candidate_min_reid)
            min_combined = max(min_combined, self.recovery_single_candidate_min_combined)
            required_c_margin = 0.0
            required_r_margin = 0.0
        else:
            required_c_margin = self.combined_margin
            required_r_margin = self.reid_margin

        if using_global:
            min_reid = max(min_reid, self.global_search_min_reid)
            min_negative_margin = max(min_negative_margin, self.negative_margin + 0.08)

        gait_gate_ok = True
        gait_gate_reason = "gait_not_required"
        if self.gait.enabled and self.gait.required_for_recovery and self.gait.target_ready:
            if not live_gait_ready:
                gait_gate_ok = False
                gait_gate_reason = f"waiting_live_gait {best_gait:.3f}"
            elif best_gait < self.gait_recovery_min_score:
                gait_gate_ok = False
                gait_gate_reason = f"gait_low {best_gait:.3f} < {self.gait_recovery_min_score:.3f}"
            else:
                gait_gate_reason = f"gait_ok {best_gait:.3f}"

        accept = (
            gait_gate_ok
            and best_combined >= min_combined
            and best_reid >= min_reid
            and best_color >= min_color
            and ident_margin >= min_negative_margin
            and c_margin >= required_c_margin
            and r_margin >= required_r_margin
        )

        reason = (
            f"best={best_combined:.3f}, second={second_combined:.3f}, "
            f"reid={best_reid:.3f}, second_reid={second_reid:.3f}, "
            f"color={best_color:.3f}, gait={best_gait:.3f}, body={best_body:.3f}, silh={best_silh:.3f}, "
            f"neg={best_neg:.3f}, id_margin={ident_margin:.3f}, "
            f"c_margin={c_margin:.3f}, r_margin={r_margin:.3f}, "
            f"has_second={has_second}, global={using_global}, {gait_gate_reason}"
        )
        return bool(accept), reason

    def _try_recover(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        lost_age: float,
        exclude_ids: Optional[Set[int]] = None,
    ) -> IdentityResult:
        exclude_ids = exclude_ids or set()
        stable = self._stable_tracks(tracks)
        candidates = [t for t in (stable or tracks) if int(t.track_id) not in exclude_ids and int(t.area) >= self.min_bbox_area]
        if not self.identity_ready:
            return IdentityResult("WAITING_FOR_ENROLLMENT", None, self.target_track_id, lost_age_sec=lost_age, reason="identity_not_enrolled")
        if not candidates:
            return IdentityResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="no_candidate")

        gated: List[TrackCandidate] = []
        for t in candidates:
            if self.target_bbox is None or self.spatial_gate_px <= 0:
                gated.append(t)
                continue
            gate = self.spatial_gate_px * (1.0 + 0.25 * min(lost_age, 4.0))
            if self._bbox_center_distance(t.bbox, self.target_bbox) <= gate:
                gated.append(t)

        allow_global = self.enable_global_recovery and lost_age >= self.global_search_after_sec
        using_global = False
        if not gated and allow_global:
            gated = candidates
            using_global = True
        if not gated:
            return IdentityResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="spatial_gate_rejected")

        scored = self._score_candidates(frame, gated, extract_features)
        if not scored:
            return IdentityResult("LOST", None, self.target_track_id, lost_age_sec=lost_age, reason="no_scores")

        ok, reason = self._accept_recovery(scored, using_global=using_global)
        best_combined, best_reid, best_color, best_neg, ident_margin, best_gait, best_body, best_silh, live_gait_ready, best_track, best_feature, best_color_hist, best_gait_score = scored[0]
        second = scored[1][0] if len(scored) > 1 else -1.0

        # Do not learn negatives while the target is lost/recovering.
        # At this stage duplicate tracks or fragmented detections may actually be
        # the target, so adding scored[1:] to the negative gallery can poison the
        # gallery and reject the true person after finish. Hard negatives are still
        # learned in _verify_visible_target() only after the target is verified.

        if ok:
            # Do not switch to a new DeepSORT ID immediately. Require the same
            # candidate to pass identity checks for several consecutive frames.
            best_id = int(best_track.track_id)
            if best_id != self.target_track_id:
                if self.pending_recovery_track_id == best_id:
                    self.pending_recovery_hits += 1
                else:
                    self.pending_recovery_track_id = best_id
                    self.pending_recovery_hits = 1
                if self.pending_recovery_hits < self.recovery_confirm_frames:
                    return IdentityResult(
                        "REID_CANDIDATE_PENDING",
                        None,
                        self.target_track_id,
                        reid_similarity=best_reid,
                        color_similarity=best_color,
                        negative_similarity=best_neg,
                        identity_margin=ident_margin,
                        combined_similarity=best_combined,
                        gait_similarity=best_gait,
                        body_similarity=best_body,
                        silhouette_similarity=best_silh,
                        gait_ready=self.gait.target_ready,
                        live_gait_ready=live_gait_ready,
                        second_similarity=second,
                        lost_age_sec=lost_age,
                        enrolled_samples=len(self.positive_gallery),
                        negative_samples=len(self.negative_gallery),
                        enrollment_progress=1.0,
                        reason=reason + f", confirm={self.pending_recovery_hits}/{self.recovery_confirm_frames}",
                    )
            self.pending_recovery_track_id = None
            self.pending_recovery_hits = 0
            self.target_track_id = int(best_track.track_id)
            self.target_bbox = best_track.bbox
            self.last_seen_time = time.time()
            self.current_verify_failures = 0
            best_track.feature = best_feature
            self.positive_gallery.add_diverse(
                best_feature,
                min_similarity_to_accept=self.target_update_min_similarity,
                max_similarity_to_last=self.max_duplicate_similarity,
            )
            self.color_gallery.add(best_color_hist)
            return IdentityResult(
                "REID_RECOVERED",
                best_track,
                self.target_track_id,
                reid_similarity=best_reid,
                color_similarity=best_color,
                negative_similarity=best_neg,
                identity_margin=ident_margin,
                combined_similarity=best_combined,
                gait_similarity=best_gait,
                body_similarity=best_body,
                silhouette_similarity=best_silh,
                gait_ready=self.gait.target_ready,
                live_gait_ready=live_gait_ready,
                second_similarity=second,
                recovered=True,
                verified=True,
                lost_age_sec=lost_age,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=1.0,
                reason=reason + (", global" if using_global else ", gated"),
            )

        self.pending_recovery_track_id = None
        self.pending_recovery_hits = 0
        return IdentityResult(
            "LOST_REJECTED_CANDIDATE",
            None,
            self.target_track_id,
            reid_similarity=best_reid,
            color_similarity=best_color,
            negative_similarity=best_neg,
            identity_margin=ident_margin,
            combined_similarity=best_combined,
            gait_similarity=best_gait,
            body_similarity=best_body,
            silhouette_similarity=best_silh,
            gait_ready=self.gait.target_ready,
            live_gait_ready=live_gait_ready,
            second_similarity=second,
            lost_age_sec=lost_age,
            enrolled_samples=len(self.positive_gallery),
            negative_samples=len(self.negative_gallery),
            enrollment_progress=1.0,
            reason=reason,
        )

    def _verify_visible_target(
        self,
        frame: np.ndarray,
        target: TrackCandidate,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        now: float,
    ) -> IdentityResult:
        if self.frame_index % self.verify_every_n_frames != 0:
            self.target_bbox = target.bbox
            self.last_seen_time = now
            return IdentityResult(
                "TRACKING",
                target,
                int(target.track_id),
                verified=True,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=1.0,
                reason="not_verify_frame",
            )

        extracted = self._extract_many(frame, tracks, extract_features, with_gait=True)
        target_feature: Optional[np.ndarray] = None
        target_color: Optional[np.ndarray] = None
        target_gait_score: Optional[GaitScore] = None
        other_scores = []
        target_scores = (-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, False)
        for t, f, c, g in extracted:
            pos, color, neg, ident_margin, combined, gait_sim, body_sim, silh_sim, live_ready = self._scores(f, c, g)
            if int(t.track_id) == int(target.track_id):
                target_feature, target_color = f, c
                target_gait_score = g
                target_scores = (pos, color, neg, ident_margin, combined, gait_sim, body_sim, silh_sim, live_ready)
            else:
                other_scores.append((pos, neg, f))

        if target_feature is None:
            return self._try_recover(frame, tracks, extract_features, lost_age=0.0, exclude_ids={int(target.track_id)})

        pos, color, neg, ident_margin, combined, gait_sim, body_sim, silh_sim, live_ready = target_scores
        gait_visible_ok = True
        if self.gait.enabled and self.gait.target_ready and target_gait_score is not None and target_gait_score.live_gait_ready:
            gait_visible_ok = gait_sim >= self.gait_current_min_score
        ok = (
            gait_visible_ok
            and pos >= self.current_min_reid
            and combined >= self.current_min_combined
            and color >= self.current_min_color
            and ident_margin >= max(0.08, self.negative_margin * 0.55)
        )

        if not ok:
            self.current_verify_failures += 1
            if self.current_verify_failures >= self.verification_fail_limit:
                # Safety-first policy: never jump directly from a suspicious
                # visible target ID to another person. Mark target as lost and
                # let strict recovery confirm a new ID over multiple frames.
                self.target_track_id = None
                self.pending_recovery_track_id = None
                self.pending_recovery_hits = 0
                return IdentityResult(
                    "TRACK_ID_SUSPECT",
                    None,
                    int(target.track_id),
                    reid_similarity=pos,
                    color_similarity=color,
                    negative_similarity=neg,
                    identity_margin=ident_margin,
                    combined_similarity=combined,
                    gait_similarity=gait_sim,
                    body_similarity=body_sim,
                    silhouette_similarity=silh_sim,
                    gait_ready=self.gait.target_ready,
                    live_gait_ready=live_ready,
                    verified=False,
                    enrolled_samples=len(self.positive_gallery),
                    negative_samples=len(self.negative_gallery),
                    enrollment_progress=1.0,
                    reason=f"visible track failed identity check; failures={self.current_verify_failures}",
                )

            return IdentityResult(
                "TRACK_VERIFY_WARN",
                target,
                int(target.track_id),
                reid_similarity=pos,
                color_similarity=color,
                negative_similarity=neg,
                identity_margin=ident_margin,
                combined_similarity=combined,
                gait_similarity=gait_sim,
                body_similarity=body_sim,
                silhouette_similarity=silh_sim,
                gait_ready=self.gait.target_ready,
                live_gait_ready=live_ready,
                verified=False,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=1.0,
                reason=f"identity weak; failures={self.current_verify_failures}",
            )

        # When the target is verified, every other visible person in this frame
        # is definitely a non-target. Add them as hard negatives even if their
        # appearance is similar; these are the exact distractors we must reject.
        for _other_pos, _other_neg, other_feature in other_scores:
            self.negative_gallery.add_diverse(other_feature, max_similarity_to_last=self.max_duplicate_similarity)

        self.current_verify_failures = 0
        self.pending_recovery_track_id = None
        self.pending_recovery_hits = 0
        self.target_bbox = target.bbox
        self.last_seen_time = now
        if (self.frame_index % self.update_gallery_every_n_frames) == 0:
            target.feature = target_feature
            self.positive_gallery.add_diverse(
                target_feature,
                min_similarity_to_accept=self.target_update_min_similarity,
                max_similarity_to_last=self.max_duplicate_similarity,
            )
            self.color_gallery.add(target_color)

        return IdentityResult(
            "TRACKING",
            target,
            int(target.track_id),
            reid_similarity=pos,
            color_similarity=color,
            negative_similarity=neg,
            identity_margin=ident_margin,
            combined_similarity=combined,
            gait_similarity=gait_sim,
            body_similarity=body_sim,
            silhouette_similarity=silh_sim,
            gait_ready=self.gait.target_ready,
            live_gait_ready=live_ready,
            verified=True,
            enrolled_samples=len(self.positive_gallery),
            negative_samples=len(self.negative_gallery),
            enrollment_progress=1.0,
            reason="verified_target",
        )

    def update(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        now: Optional[float] = None,
    ) -> IdentityResult:
        now = time.time() if now is None else float(now)
        self.frame_index += 1
        self._update_track_counts(tracks)

        if self.auto_enroll_on_start and not self._auto_enroll_started and not self.identity_ready:
            self._auto_enroll_started = True
            self.start_enrollment(now=now)

        if self.enrolling:
            return self._handle_enrollment(frame, tracks, extract_features, now)

        if not self.identity_ready:
            return IdentityResult(
                "WAITING_FOR_ENROLLMENT",
                None,
                None,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=0.0,
                gait_ready=self.gait.target_ready,
                live_gait_ready=False,
                reason="press e or call /person_reid/start_enroll",
            )

        track_by_id = {int(t.track_id): t for t in tracks}
        if self.target_track_id is not None and self.target_track_id in track_by_id:
            return self._verify_visible_target(frame, track_by_id[self.target_track_id], tracks, extract_features, now)

        lost_age = 0.0 if self.last_seen_time is None else max(0.0, now - self.last_seen_time)
        if lost_age > self.max_lost_keep_id_sec:
            self.target_track_id = None
        return self._try_recover(frame, tracks, extract_features, lost_age=lost_age)
