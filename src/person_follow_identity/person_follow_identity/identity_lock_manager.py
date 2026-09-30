from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple

import numpy as np

from person_reid_tracker.types import BBox, TrackCandidate

from .identity_gallery import ColorGallery, FeatureGallery, NegativeMemory, normalize_feature, person_color_histogram, view_bucket
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
    # Ti le bbox bi nguoi dung truoc che, diem bang chung khoa/tim lai, kieu khung
    occlusion: float = 0.0
    evidence: float = 0.0
    view_bucket: int = 0


@dataclass
class _Obs:
    """Quan sat mot track trong khung hien tai: dac trung, hinh hoc, diem so."""

    track: TrackCandidate
    feature: Optional[np.ndarray]
    color: Optional[np.ndarray]
    gait: Optional[GaitScore]
    bucket: int
    occ: float
    overlap: float
    quality: float
    pos: float = -1.0
    color_sim: float = -1.0
    neg: float = -1.0
    combined: float = -1.0
    gait_sim: float = -1.0
    body_sim: float = -1.0
    silh_sim: float = -1.0
    live_ready: bool = False
    # Diem cao nhat cua NGUOI KHAC (du ro) trong cung khung — doi thu truc tiep
    rival: float = -1.0
    rival_combined: float = -1.0

    @property
    def tid(self) -> int:
        return int(self.track.track_id)

    @property
    def id_margin(self) -> float:
        return self.pos - self.neg if self.neg >= 0 else 1.0


def _intersects(a: BBox, b: BBox) -> bool:
    return min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])


class TargetIdentityManager:
    """Strict single-person identity lock.

    The manager never picks a new person after the target is lost. It only locks
    when the operator starts enrollment. After enrollment, a candidate must pass
    ReID, color, negative-gallery, and best-vs-second margin checks.

    Ban toi uu cho cho dong nguoi (29/09):
    - Moi khung cham diem TAT CA nguoi (feature ReID lay lai tu DeepSORT, gan nhu mien phi).
    - Biet ai che ai (chan thap hon trong anh = gan camera hon = dung truoc). Khung bi che
      khong duoc hoc vao gallery, va chi tru it bang chung khoa.
    - Giu khoa bang BANG CHUNG tich luy (toi da verification_fail_limit) thay vi bo khoa
      ngay lan kiem tra hong dau tien. Bo NGAY khi co mau thuan ro: anh giong mot nguoi
      da biet hon muc tieu, nguoi khac giong muc tieu hon han, hoac ngoai hinh cua chinh
      track doi dot ngot (DeepSORT tron ID).
    - Tim lai bang bang chung tung ung vien: khop thuong +1, khop manh +2, hong -1. Ro
      rang thi 2 khung, mo ho thi khong bao gio.
    - Gallery am tach theo tung nguoi; chi hoc nguoi KHONG cham bbox muc tieu (bbox chong
      nhau chua diem anh cua muc tieu -> "dau doc" gallery am).
    - Gallery theo kieu khung (toan than / cat dau / cat chan) cho nguoi dung gan xe.
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
        occlusion_ratio_max: float = 0.35,
        occluded_fail_weight: float = 0.25,
        learn_min_quality: float = 0.75,
        eval_min_quality: float = 0.45,
        switch_margin: float = 0.08,
        strong_negative_margin: float = 0.04,
        current_negative_margin: float = 0.0,
        self_consistency_min: float = 0.72,
        recover_strong_reid: float = 0.88,
        view_bucket_min_samples: int = 5,
        bucket_bootstrap_frames: int = 8,
        suspect_penalty: float = 2.0,
        edge_margin_px: int = 6,
        occluded_loss_hold_sec: float = 2.5,
        recovery_probation_sec: float = 3.0,
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
        # So lan kiem tra hong (anh sach) lien tiep truoc khi bo khoa = tran bang chung khoa
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
        # Diem bang chung can de nhan lai muc tieu (khop thuong +1, khop manh +2)
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
        self.occlusion_ratio_max = float(occlusion_ratio_max)
        self.occluded_fail_weight = float(occluded_fail_weight)
        self.learn_min_quality = float(learn_min_quality)
        self.eval_min_quality = float(eval_min_quality)
        self.switch_margin = float(switch_margin)
        self.strong_negative_margin = float(strong_negative_margin)
        self.current_negative_margin = float(current_negative_margin)
        self.self_consistency_min = float(self_consistency_min)
        self.recover_strong_reid = float(recover_strong_reid)
        self.view_bucket_min_samples = max(1, int(view_bucket_min_samples))
        self.bucket_bootstrap_frames = max(1, int(bucket_bootstrap_frames))
        self.suspect_penalty = max(0.0, float(suspect_penalty))
        self.edge_margin_px = max(0, int(edge_margin_px))
        self.occluded_loss_hold_sec = max(0.0, float(occluded_loss_hold_sec))
        self.recovery_probation_sec = max(0.0, float(recovery_probation_sec))
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
        self.negative_gallery = NegativeMemory(max_size=negative_gallery_size, max_people=max(16, int(negative_gallery_size) // 8))
        self.color_gallery = ColorGallery(max_size=color_gallery_size)

        self.frame_index = 0
        self.track_frames: Dict[int, int] = {}
        self.target_track_id: Optional[int] = None
        self.target_bbox: Optional[BBox] = None
        self.last_seen_time: Optional[float] = None
        self.lock_evidence = 0.0
        self.lock_stable_frames = 0
        self.target_recent: deque[np.ndarray] = deque(maxlen=8)
        self.recovery_evidence: Dict[int, float] = {}
        # Track ID da thay CUNG LUC voi muc tieu da xac nhan -> chac chan la nguoi khac
        # (tru khi DeepSORT trao ID). tid -> lan cuoi thay cung.
        self.coseen: Dict[int, float] = {}
        # Track ID vua cham bbox muc tieu: tid -> lan cuoi cham
        self.contacts: Dict[int, float] = {}
        # Muc tieu bien mat khi dang cham nguoi khac -> dang bi che: chan nguoi che toi luc nay
        self.hidden_until = 0.0
        self._was_visible = False
        # Vua nhan lai: trong thoi gian thu thach, ai giong muc tieu hon track dang khoa du
        # chi 0.03 thi tinh la mot lan hong (de chuyen nhanh sang nguoi dung neu nhan nham)
        self.probation_until = 0.0
        # Mat khi dang cham nguoi khac: ID cu con giu toi luc nay, quay lai thi chi tin muc thap nhat
        self.hidden_keep_id_until = 0.0

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

    def _clear_lock_state(self) -> None:
        self.target_track_id = None
        self.target_bbox = None
        self.last_seen_time = None
        self.lock_evidence = 0.0
        self.lock_stable_frames = 0
        self.target_recent.clear()
        self.recovery_evidence.clear()
        self.coseen.clear()
        self.contacts.clear()
        self.hidden_until = 0.0
        self._was_visible = False
        self.probation_until = 0.0
        self.hidden_keep_id_until = 0.0

    def reset(self) -> None:
        self.positive_gallery.clear()
        self.negative_gallery.clear()
        self.color_gallery.clear()
        self.gait.clear()
        self.frame_index = 0
        self.track_frames.clear()
        self._clear_lock_state()
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
        self._clear_lock_state()
        self.enrolling = True
        self.enrollment_start_time = now
        self.enrollment_track_id = None

    def _lock_after_enrollment(self, track_id: Optional[int]) -> None:
        # Mau enroll thanh mau neo: hoc online sau nay khong day chung ra khoi gallery
        self.positive_gallery.freeze_anchor()
        self.color_gallery.freeze_anchor()
        self.enrolling = False
        self.enrollment_start_time = None
        self.enrollment_track_id = None
        self.target_track_id = int(track_id) if track_id is not None else None
        self.lock_evidence = float(self.verification_fail_limit)
        # Track enroll da duoc nguoi van hanh xac nhan -> tin ngay, cho hoc tiep
        self.lock_stable_frames = self.bucket_bootstrap_frames
        self.target_recent.clear()
        self.recovery_evidence.clear()

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
        self._lock_after_enrollment(self.enrollment_track_id)
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

    # ------------------------------------------------------------------
    # Quan sat moi khung
    # ------------------------------------------------------------------

    def _geometry(self, tracks: List[TrackCandidate], frame_shape: Tuple[int, int, int]) -> Dict[int, Tuple[float, float, float]]:
        """(ti le bi che, ti le giao voi bat ky ai, chat luong 0..1) cho moi track."""
        h, w = frame_shape[:2]
        m = self.edge_margin_px
        out: Dict[int, Tuple[float, float, float]] = {}
        for a in tracks:
            ax1, ay1, ax2, ay2 = a.bbox
            area = max(1.0, float((ax2 - ax1) * (ay2 - ay1)))
            a_bottom = ay2 >= h - 1 - m
            occ = 0.0
            overlap = 0.0
            for b in tracks:
                if b is a:
                    continue
                bx1, by1, bx2, by2 = b.bbox
                iw = min(ax2, bx2) - max(ax1, bx1)
                ih = min(ay2, by2) - max(ay1, by1)
                if iw <= 0 or ih <= 0:
                    continue
                ioa = float(iw * ih) / area
                overlap = max(overlap, ioa)
                # San phang: chan thap hon trong anh = gan camera hon = dung truoc.
                # Ca hai cung bi cat chan (deu sat xe) thi khong biet ai truoc -> tinh ca hai.
                b_bottom = by2 >= h - 1 - m
                if by2 > ay2 + 2 or (b_bottom and a_bottom):
                    occ += ioa
            occ = min(1.0, occ)
            q = 1.0 - occ
            if ax1 <= m or ax2 >= w - 1 - m:
                # Bi cat ngang o mep trai/phai: chi thay nua nguoi, mep anh goc rong con bi meo
                q *= 0.6
            q *= min(1.0, max(0.3, (float(ay2 - ay1) - 40.0) / 60.0))
            out[id(a)] = (occ, overlap, q)
        return out

    def _observe(
        self,
        frame: np.ndarray,
        tracks: List[TrackCandidate],
        extract_features: FeatureExtractorFn,
        with_scores: bool = True,
        with_gait: bool = True,
    ) -> List[_Obs]:
        if not tracks:
            return []
        geo = self._geometry(tracks, frame.shape)
        features = extract_features(frame, [t.bbox for t in tracks])
        obs: List[_Obs] = []
        for t, f in zip(tracks, features):
            occ, overlap, q = geo[id(t)]
            g = self.gait.score_candidate(frame, t.bbox, int(t.track_id)) if (with_gait and self.gait.enabled) else None
            o = _Obs(
                track=t,
                feature=f,
                color=person_color_histogram(frame, t.bbox),
                gait=g,
                bucket=view_bucket(t.bbox, frame.shape, self.edge_margin_px),
                occ=occ,
                overlap=overlap,
                quality=q,
            )
            if with_scores:
                self._score(o)
            obs.append(o)
        if with_scores:
            for o in obs:
                others = [x for x in obs if x is not o and x.feature is not None and x.quality >= self.eval_min_quality]
                o.rival = max((x.pos for x in others), default=-1.0)
                o.rival_combined = max((x.combined for x in others), default=-1.0)
        return obs

    def _score(self, o: _Obs) -> None:
        k = self.view_bucket_min_samples
        o.pos = self.positive_gallery.similarity(o.feature, mode=self.reid_match_mode, top_k=self.reid_top_k, tag=o.bucket, min_tag_samples=k)
        o.neg = self.negative_gallery.similarity(o.feature, tag=o.bucket, top_k=3)
        o.color_sim = self.color_gallery.similarity(o.color, mode="topk_mean", top_k=self.reid_top_k, tag=o.bucket, min_tag_samples=k)
        o.combined = self._combined_score(o.pos, o.color_sim, o.gait)
        if o.gait is not None:
            o.gait_sim = o.gait.combined_similarity
            o.body_sim = o.gait.body_similarity
            o.silh_sim = o.gait.silhouette_similarity
            o.live_ready = o.gait.live_gait_ready

    def _result(self, status: str, o: Optional[_Obs], target: Optional[TrackCandidate], track_id: Optional[int], **kw) -> IdentityResult:
        fields = dict(
            enrolled_samples=len(self.positive_gallery),
            negative_samples=len(self.negative_gallery),
            enrollment_progress=1.0,
            gait_ready=self.gait.target_ready,
        )
        if o is not None:
            fields.update(
                reid_similarity=o.pos,
                color_similarity=o.color_sim,
                negative_similarity=o.neg,
                identity_margin=o.id_margin,
                combined_similarity=o.combined,
                gait_similarity=o.gait_sim,
                body_similarity=o.body_sim,
                silhouette_similarity=o.silh_sim,
                live_gait_ready=o.live_ready,
                second_similarity=o.rival,
                occlusion=o.occ,
                view_bucket=o.bucket,
            )
        fields.update(kw)
        return IdentityResult(status, target, track_id, **fields)

    # ------------------------------------------------------------------
    # Enroll
    # ------------------------------------------------------------------

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
        obs = self._observe(frame, stable, extract_features, with_scores=False, with_gait=False)
        sample_frame = (self.frame_index % self.enroll_sample_every_n_frames) == 0
        for o in obs:
            if o.tid == int(target.track_id):
                # Chi hoc anh sach: bi nguoi khac che/cham vao thi crop chua diem anh cua ho
                clean = o.quality >= self.learn_min_quality and o.overlap < 0.05 and o.track.conf > 0.0
                if sample_frame and clean:
                    self.positive_gallery.add_diverse(
                        o.feature,
                        min_similarity_to_accept=-1.0 if len(self.positive_gallery) < 5 else 0.35,
                        max_similarity_to_last=self.max_duplicate_similarity,
                        tag=o.bucket,
                    )
                    self.color_gallery.add(o.color, tag=o.bucket)
                if self.gait.enabled and (self.frame_index % self.gait_sample_every_n_frames) == 0:
                    self.gait.add_target_sample(frame, target.bbox)
            elif o.track.conf > 0.0 and not _intersects(o.track.bbox, target.bbox):
                # Other people seen during enrollment become known non-targets.
                self.negative_gallery.add(o.feature, o.tid, tag=o.bucket, max_similarity_to_last=self.max_duplicate_similarity)

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
            self._lock_after_enrollment(int(target.track_id))
            return IdentityResult(
                "ENROLLMENT_DONE",
                target,
                self.target_track_id,
                gait_ready=self.gait.target_ready,
                verified=True,
                enrolled_samples=len(self.positive_gallery),
                negative_samples=len(self.negative_gallery),
                enrollment_progress=1.0,
                reason="target identity memory is ready",
            )

        buckets = "/".join(str(self.positive_gallery.tag_count(b)) for b in range(4))
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
                f"collecting target samples; app={len(self.positive_gallery)}/{self.enroll_min_samples} "
                f"(full/top-cut/bottom-cut/both={buckets}), "
                f"gait={self.gait.target_sample_count}/{self.gait.min_frames}, gait_status={self.gait.reason}"
            ),
        )

    # ------------------------------------------------------------------
    # Giu khoa
    # ------------------------------------------------------------------

    def _alone(self, t: _Obs, obs: List[_Obs]) -> bool:
        """Khong ai dung trong/nga sat bbox muc tieu (noi rong nua be ngang moi ben)."""
        x1, y1, x2, y2 = t.track.bbox
        pad = 0.5 * (x2 - x1)
        zone = (int(x1 - pad), y1, int(x2 + pad), y2)
        return not any(o is not t and _intersects(o.track.bbox, zone) for o in obs)

    def _gait_gate(self, o: _Obs, min_score: float, required: bool) -> Tuple[bool, str]:
        if not (self.gait.enabled and self.gait.target_ready):
            return True, "gait_not_required"
        if o.gait is None or not o.live_ready:
            return (not required), f"waiting_live_gait {o.gait_sim:.3f}"
        if o.gait_sim < min_score:
            return False, f"gait_low {o.gait_sim:.3f} < {min_score:.3f}"
        return True, f"gait_ok {o.gait_sim:.3f}"

    def _maintain_lock(self, t: _Obs, obs: List[_Obs], now: float) -> IdentityResult:
        cap = float(self.verification_fail_limit)
        gap = 0.0 if self.last_seen_time is None else now - self.last_seen_time
        if gap > 0.5:
            # Cung ID DeepSORT quay lai sau mot luc mat: can xac nhan lai truoc khi hoc
            self.lock_evidence = min(self.lock_evidence, max(1.0, cap - 1.0))
            self.lock_stable_frames = 0
        if now < self.hidden_keep_id_until:
            # Vua mat khi dang chong voi nguoi khac (luc DeepSORT hay trao ID): tin muc thap nhat
            self.lock_evidence = min(self.lock_evidence, 1.0)
            self.lock_stable_frames = 0
        self.hidden_keep_id_until = 0.0
        self.hidden_until = 0.0
        self.target_bbox = t.track.bbox
        self.last_seen_time = now
        x1, y1, x2, y2 = t.track.bbox
        pad = 0.2 * (x2 - x1)
        near = (int(x1 - pad), y1, int(x2 + pad), y2)
        for o in obs:
            if o is not t and _intersects(o.track.bbox, near):
                self.contacts[o.tid] = now

        alone = self._alone(t, obs)
        if (
            self.verify_every_n_frames > 1
            and (self.frame_index % self.verify_every_n_frames) != 0
            and alone
            and self.lock_evidence >= cap
        ):
            return self._result("TRACKING", t, t.track, t.tid, verified=True, evidence=self.lock_evidence, reason="not_verify_frame")

        if t.feature is None:
            return self._result("TRACKING", t, t.track, t.tid, verified=False, evidence=self.lock_evidence, reason="no_feature_this_frame")
        if t.track.conf <= 0.0:
            # YOLO bo sot khung nay, DeepSORT chi xuat vi tri du doan: van bao (lien tuc cho tracker)
            # nhung khong cham diem/khong hoc — hop co the lech khoi nguoi.
            return self._result("TRACKING", t, t.track, t.tid, verified=False, evidence=self.lock_evidence,
                                reason="track du doan 1 khung (YOLO bo sot), khong kiem tra")

        gait_ok, gait_reason = self._gait_gate(t, self.gait_current_min_score, required=False)
        abs_ok = (
            t.pos >= self.current_min_reid
            and t.combined >= self.current_min_combined
            and (t.color_sim < 0 or t.color_sim >= self.current_min_color)
        )
        recent_contact = any(now - when <= 1.0 for when in self.contacts.values())
        neg_need = self.negative_margin if recent_contact else self.current_negative_margin
        neg_ok = t.neg < 0 or t.pos - t.neg >= neg_need
        rival_margin = 0.03 if now < self.probation_until else self.switch_margin
        rival_ok = t.rival < 0 or (t.rival - t.pos) < rival_margin
        # Nguoi VUA CAT QUA muc tieu (cham trong 3 s) ma gio giong muc tieu hon track dang
        # khoa -> co the DeepSORT da trao ID luc tach ra. Nguoi mac giong het thi moi khung
        # chi hon kem chut it, nen tinh nhu mot lan hong (tich luy qua nhieu khung).
        partner = max(
            (o.pos for o in obs if o is not t and o.feature is not None and o.occ < 0.2
             and now - self.contacts.get(o.tid, -1e9) <= 3.0),
            default=-1.0,
        )
        partner_ok = partner < 0 or partner - t.pos < 0.03
        passed = gait_ok and abs_ok and neg_ok and rival_ok and partner_ok

        self_sim = -1.0
        if len(self.target_recent) >= 3:
            proto = normalize_feature(np.mean(np.stack(list(self.target_recent), axis=0), axis=0))
            self_sim = float(np.dot(proto, normalize_feature(t.feature)))

        # Mau thuan ro — chi xet khi anh SACH. Bi che mot phan thi crop lan diem anh nguoi
        # che: pos cua muc tieu tut, neg/rival (chinh nguoi che) len -> bo khoa sai, roi
        # luc tim lai co the nhan nham nguoi che. Luc bi che chi tru bang chung nhe.
        contradiction = ""
        # Doi thu RO va MANH: nguoi khac anh sach, khop muc tieu rat tot va hon han track dang khoa,
        # lai khong phai nguoi da biet -> bo khoa NGAY du track dang khoa bi che (luc DeepSORT tron
        # ID giua hai nguoi chong nhau, track dang khoa chinh la nguoi kia dang bi che).
        strong_rival = max(
            (o for o in obs if o is not t and o.feature is not None and o.occ < 0.2
             and o.quality >= self.eval_min_quality and (o.neg < 0 or o.pos - o.neg >= self.negative_margin)),
            key=lambda o: o.pos, default=None,
        )
        if (
            strong_rival is not None
            and strong_rival.pos >= self.recover_strong_reid
            and strong_rival.pos - t.pos >= 2.0 * self.switch_margin
        ):
            contradiction = (
                f"nguoi khac (ID {strong_rival.tid}) khop muc tieu hon han: {strong_rival.pos:.3f} > pos {t.pos:.3f}"
            )
        elif t.occ < 0.2 and t.quality >= self.eval_min_quality:
            if t.neg >= 0 and t.neg - t.pos >= self.strong_negative_margin:
                contradiction = f"giong nguoi da biet hon muc tieu (neg {t.neg:.3f} > pos {t.pos:.3f})"
            elif t.rival >= self.recover_min_reid and t.rival - t.pos >= self.switch_margin:
                contradiction = f"nguoi khac giong muc tieu hon (rival {t.rival:.3f} > pos {t.pos:.3f})"
            elif 0 <= self_sim < self.self_consistency_min and t.pos < self.reid_threshold:
                contradiction = f"ngoai hinh track doi dot ngot (self {self_sim:.3f}), nghi DeepSORT tron ID"

        occluded = t.occ >= self.occlusion_ratio_max or t.quality < self.eval_min_quality
        if contradiction:
            self.lock_evidence = 0.0
        elif passed:
            self.lock_evidence = min(cap, self.lock_evidence + 1.0)
        else:
            if recent_contact and not occluded:
                # Vua tach khoi nguoi khac ma anh sach van hong -> nghi trao ID, bo nhanh
                self.lock_evidence = min(self.lock_evidence, 1.0)
            self.lock_evidence -= self.occluded_fail_weight if occluded else 1.0

        detail = (
            f"pos={t.pos:.3f}, rival={t.rival:.3f}, partner={partner:.3f}, neg={t.neg:.3f}, color={t.color_sim:.3f}, "
            f"self={self_sim:.3f}, occ={t.occ:.2f}, q={t.quality:.2f}, view={t.bucket}, ev={self.lock_evidence:.2f}, {gait_reason}"
        )

        if self.lock_evidence <= 0.0:
            # Safety-first policy: never jump directly from a suspicious
            # visible target ID to another person. Mark target as lost and
            # let strict recovery confirm a new ID over multiple frames.
            tid = t.tid
            self.target_track_id = None
            self.lock_stable_frames = 0
            self.target_recent.clear()
            self.recovery_evidence = {tid: -self.suspect_penalty}
            # Khoa vua bi nghi sai -> nhung ai "da thay cung luc" voi no co the chinh la muc tieu
            self.coseen.clear()
            self.hidden_until = 0.0
            self._was_visible = False
            return self._result(
                "TRACK_ID_SUSPECT", t, None, tid, verified=False, evidence=0.0,
                reason=(contradiction or "visible track failed identity check repeatedly") + "; " + detail,
            )

        if not passed:
            self.lock_stable_frames = 0
            if not (neg_ok and rival_ok and partner_ok):
                # Hong vi co dau hieu la NGUOI KHAC (nguoi khac giong muc tieu hon / anh giong nguoi da
                # biet): giu khoa nhung TAM KHONG BAO muc tieu — bao nham nguy hiem hon mat vai khung.
                return self._result(
                    "TRACK_VERIFY_HOLD", t, None, t.tid, verified=False, evidence=self.lock_evidence,
                    reason="nghi nguoi khac, tam khong bao; " + detail,
                )
            # Hong chi vi diem thap (bi che, quay lung, xa): van bao de tracker co du lieu lien tuc
            return self._result("TRACK_VERIFY_WARN", t, t.track, t.tid, verified=False, evidence=self.lock_evidence, reason="identity weak; " + detail)

        self.lock_stable_frames += 1
        if self.lock_evidence >= cap:
            for o in obs:
                if o is not t:
                    self.coseen[o.tid] = now
        clean = t.quality >= self.learn_min_quality and t.overlap < 0.05
        if clean and (self_sim < 0 or self_sim >= self.self_consistency_min):
            self.target_recent.append(normalize_feature(t.feature))
        if self.lock_evidence >= cap:
            self._learn(t, obs, clean, alone)

        return self._result("TRACKING", t, t.track, t.tid, verified=True, evidence=self.lock_evidence, reason="verified_target; " + detail)

    def _learn(self, t: _Obs, obs: List[_Obs], clean: bool, alone: bool) -> None:
        if clean and t.track.conf > 0.0 and (self.frame_index % self.update_gallery_every_n_frames) == 0 and (t.rival < 0 or t.rival < t.pos - 0.03):
            t.track.feature = t.feature
            have = self.positive_gallery.tag_count(t.bucket) >= self.view_bucket_min_samples
            # Kieu khung moi (vd nguoi vua lai gan, bi cat dau) chi hoc khi da giu khoa on dinh
            # mot luc va khong ai dung sat — luc do su lien tuc cua track la bang chung chinh.
            if have or (self.lock_stable_frames >= self.bucket_bootstrap_frames and alone):
                if self.positive_gallery.add_diverse(
                    t.feature,
                    min_similarity_to_accept=self.target_update_min_similarity,
                    max_similarity_to_last=self.max_duplicate_similarity,
                    tag=t.bucket,
                ):
                    self.color_gallery.add(t.color, tag=t.bucket)

        # When the target is verified, every other visible person in this frame
        # is a non-target. Nguoi dung SAU muc tieu (bbox chong, bi che) co diem anh cua
        # muc tieu trong crop -> khong hoc, khong thi muc tieu bi loai ve sau. Nguoi dung
        # TRUOC (khong ai che) anh sach -> hoc, de luc ho che mat muc tieu thi khong nhan nham.
        trusted = self.lock_stable_frames >= 4 and t.pos >= self.current_min_reid + 0.05
        for o in obs:
            if o is t or o.feature is None or o.quality < self.eval_min_quality or o.track.conf <= 0.0:
                continue
            if _intersects(o.track.bbox, t.track.bbox) and o.occ >= 0.2:
                # Dung SAU muc tieu: crop chua diem anh cua muc tieu -> khong hoc
                continue
            if o.pos <= self.negative_add_max_reid or (trusted and o.pos < t.pos - self.negative_add_min_margin):
                self.negative_gallery.add(o.feature, o.tid, tag=o.bucket, max_similarity_to_last=self.max_duplicate_similarity)

    # ------------------------------------------------------------------
    # Tim lai
    # ------------------------------------------------------------------

    def _recovery_check(self, o: _Obs, using_global: bool) -> Tuple[bool, bool, str]:
        has_rival = o.rival >= 0
        if using_global and not self.enable_global_recovery:
            return False, False, "global_recovery_disabled"
        if using_global and not has_rival and not self.allow_single_candidate_global_recovery:
            return False, False, "single_candidate_global_rejected"

        min_reid = self.recover_min_reid
        min_combined = self.recover_min_combined
        min_negative_margin = self.negative_margin
        if not has_rival:
            # One visible person after loss is exactly the dangerous case.
            # Require much stronger evidence.
            min_reid = max(min_reid, self.recovery_single_candidate_min_reid)
            min_combined = max(min_combined, self.recovery_single_candidate_min_combined)
        if using_global:
            min_reid = max(min_reid, self.global_search_min_reid)
            min_negative_margin = self.negative_margin + 0.03

        r_margin = o.pos - o.rival if has_rival else 1.0
        c_margin = o.combined - o.rival_combined if has_rival else 1.0
        gait_ok, gait_reason = self._gait_gate(o, self.gait_recovery_min_score, required=self.gait.required_for_recovery)
        ok = (
            gait_ok
            and o.pos >= min_reid
            and o.combined >= min_combined
            and (o.color_sim < 0 or o.color_sim >= self.recover_min_color)
            and o.id_margin >= min_negative_margin
            and r_margin >= self.reid_margin
            and c_margin >= self.combined_margin
        )
        strong = (
            ok
            and o.pos >= max(min_reid, self.recover_strong_reid)
            and r_margin >= 2.0 * self.reid_margin
            and o.id_margin >= 2.0 * min_negative_margin
        )
        reason = (
            f"reid={o.pos:.3f}, rival={o.rival:.3f}, color={o.color_sim:.3f}, neg={o.neg:.3f}, "
            f"id_margin={o.id_margin:.3f}, r_margin={r_margin:.3f}, c_margin={c_margin:.3f}, "
            f"occ={o.occ:.2f}, q={o.quality:.2f}, view={o.bucket}, global={using_global}, {gait_reason}"
        )
        return ok, strong, reason

    def _recover(self, obs: List[_Obs], lost_age: float, now: float) -> IdentityResult:
        present = {o.tid for o in obs}
        self.recovery_evidence = {k: v for k, v in self.recovery_evidence.items() if k in present}
        # ID DeepSORT mat qua lau thi khong con dung nua (max_age ~30 khung)
        self.coseen = {k: v for k, v in self.coseen.items() if k in present or now - v < 10.0}
        self.contacts = {k: v for k, v in self.contacts.items() if now - v < 5.0}
        candidates = [o for o in obs if o.feature is not None and o.track.conf > 0.0 and int(o.track.area) >= self.min_bbox_area]
        if not candidates:
            return self._result("LOST", None, None, self.target_track_id, lost_age_sec=lost_age, reason="no_candidate")

        gated: List[_Obs] = []
        for o in candidates:
            if self.target_bbox is None or self.spatial_gate_px <= 0:
                gated.append(o)
                continue
            gate = self.spatial_gate_px * (1.0 + 0.25 * min(lost_age, 4.0))
            if self._bbox_center_distance(o.track.bbox, self.target_bbox) <= gate:
                gated.append(o)

        allow_global = self.enable_global_recovery and lost_age >= self.global_search_after_sec
        using_global = False
        if not gated and allow_global:
            gated = candidates
            using_global = True
        if not gated:
            return self._result("LOST", None, None, self.target_track_id, lost_age_sec=lost_age, reason="spatial_gate_rejected")

        # Muc tieu vua nup sau nguoi khac (nhieu kha nang van o do) -> can them bang chung
        need = self.recovery_confirm_frames + (1 if now < self.hidden_until else 0)
        passing: List[Tuple[float, _Obs, str]] = []
        best_fail: Optional[Tuple[_Obs, str]] = None
        for o in gated:
            ok, strong, reason = self._recovery_check(o, using_global)
            ev = self.recovery_evidence.get(o.tid, 0.0)
            hidden = now < self.hidden_until
            if o.tid in self.coseen and (hidden or o.tid not in self.contacts):
                # Da thay CUNG LUC voi muc tieu da xac nhan ma chua tung dung sat no -> DeepSORT
                # khong the trao ID giua hai nguoi -> chac chan la nguoi khac, du giong the nao
                ev = min(ev, 0.0)
                ok = strong = False
                reason = "da thay cung luc voi muc tieu; " + reason
            elif o.tid in self.coseen and not strong:
                # Da thay CUNG LUC voi muc tieu -> la nguoi khac; chi khop manh moi xet
                ok = False
                reason = "da thay cung luc voi muc tieu; " + reason
            if o.quality < self.eval_min_quality or o.overlap >= 0.10:
                # Bi che nang / qua nho / dang cham nguoi khac: diem cua CA HAI nguoi deu
                # khong tin duoc (muc tieu bi che thi diem tut, nguoi che giong muc tieu thi
                # thanh "khop"). Giu nguyen bang chung, cho tach ra.
                pass
            elif strong:
                ev += 2.0
            elif ok:
                ev += 1.0
            else:
                ev = max(-self.suspect_penalty, ev - 1.0)
            self.recovery_evidence[o.tid] = ev
            if ok and o.quality >= self.eval_min_quality and o.overlap < 0.10:
                passing.append((ev, o, reason))
            elif best_fail is None or o.pos > best_fail[0].pos:
                best_fail = (o, reason)

        if passing:
            ev, best, reason = max(passing, key=lambda x: (x[0], x[1].pos))
            if ev >= need + (2 if best.tid in self.coseen else 0):
                self.target_track_id = best.tid
                self.target_bbox = best.track.bbox
                self.last_seen_time = now
                self.coseen.pop(best.tid, None)
                self.hidden_until = 0.0
                self.probation_until = now + self.recovery_probation_sec
                self.lock_evidence = min(float(self.verification_fail_limit), 2.0)
                self.lock_stable_frames = 0
                self.target_recent.clear()
                self.recovery_evidence.clear()
                best.track.feature = best.feature
                return self._result(
                    "REID_RECOVERED", best, best.track, best.tid, recovered=True, verified=True,
                    lost_age_sec=lost_age, evidence=ev,
                    reason=reason + (", global" if using_global else ", gated"),
                )
            return self._result(
                "REID_CANDIDATE_PENDING", best, None, self.target_track_id, lost_age_sec=lost_age, evidence=ev,
                reason=reason + f", evidence={ev:.1f}/{need}",
            )

        o, reason = best_fail if best_fail is not None else (gated[0], "")
        return self._result(
            "LOST_REJECTED_CANDIDATE", o, None, self.target_track_id, lost_age_sec=lost_age,
            evidence=self.recovery_evidence.get(o.tid, 0.0), reason=reason,
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

        obs = self._observe(frame, tracks, extract_features)
        if self.target_track_id is not None:
            for o in obs:
                if o.tid == self.target_track_id:
                    self._was_visible = True
                    return self._maintain_lock(o, obs, now)

        if self._was_visible:
            # Khung dau tien mat muc tieu: neu ngay truoc do no dang cham ai -> dang nup sau ho
            self._was_visible = False
            last = self.last_seen_time if self.last_seen_time is not None else now
            blockers = {k for k, when in self.contacts.items() if last - when <= 0.3}
            if blockers and self.occluded_loss_hold_sec > 0:
                self.hidden_until = now + self.occluded_loss_hold_sec
                # Hai nguoi chong nhau la luc DeepSORT hay trao ID: ID cu cua muc tieu co the
                # dang nam tren nguoi che, va nguoc lai. Giu ID them 0.5 s (YOLO bo sot 1-2 khung
                # la chuyen thuong), quay lai thi chi tin muc thap nhat; lau hon thi bo ID, tim
                # lai bang ngoai hinh (nguoi che da duoc hoc vao gallery am luc dung truoc).
                self.hidden_keep_id_until = now + 0.5
                for k in blockers:
                    self.coseen.pop(k, None)
        lost_age = 0.0 if self.last_seen_time is None else max(0.0, now - self.last_seen_time)
        if lost_age > self.max_lost_keep_id_sec or (now < self.hidden_until and now >= self.hidden_keep_id_until):
            self.target_track_id = None
        return self._recover(obs, lost_age, now)
