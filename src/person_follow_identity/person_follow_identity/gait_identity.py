from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from person_reid_tracker.types import BBox


def _cos01(a: Optional[np.ndarray], b: Optional[np.ndarray], eps: float = 1e-8) -> float:
    if a is None or b is None:
        return -1.0
    aa = np.asarray(a, dtype=np.float32).reshape(-1)
    bb = np.asarray(b, dtype=np.float32).reshape(-1)
    if aa.size == 0 or bb.size == 0 or aa.size != bb.size:
        return -1.0
    denom = float(np.linalg.norm(aa) * np.linalg.norm(bb))
    if denom < eps:
        return -1.0
    return float(np.clip(np.dot(aa, bb) / denom, 0.0, 1.0))


def _distance_similarity(a: Optional[np.ndarray], b: Optional[np.ndarray], scale: float = 0.18) -> float:
    """Similarity for normalized gait signatures, in [0, 1]."""
    if a is None or b is None:
        return -1.0
    aa = np.asarray(a, dtype=np.float32).reshape(-1)
    bb = np.asarray(b, dtype=np.float32).reshape(-1)
    if aa.size == 0 or bb.size == 0 or aa.size != bb.size:
        return -1.0
    dist = float(np.linalg.norm(aa - bb) / math.sqrt(max(1, aa.size)))
    return float(np.exp(-dist / max(1e-6, scale)))


def _best_cos(candidate: Optional[np.ndarray], refs: Sequence[np.ndarray]) -> float:
    if candidate is None or not refs:
        return -1.0
    sims = [_cos01(candidate, ref) for ref in refs]
    sims = [s for s in sims if s >= 0.0]
    return max(sims) if sims else -1.0


def _clip_bbox_with_pad(bbox: BBox, frame_shape: Tuple[int, int, int], pad_ratio: float = 0.30) -> BBox:
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = [int(v) for v in bbox]
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    pad_x = int(round(bw * pad_ratio))
    pad_y = int(round(bh * pad_ratio))
    x1 = max(0, min(w - 1, x1 - pad_x))
    x2 = max(0, min(w - 1, x2 + pad_x))
    y1 = max(0, min(h - 1, y1 - pad_y))
    y2 = max(0, min(h - 1, y2 + pad_y))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return x1, y1, x2, y2


@dataclass
class GaitObservation:
    body: Optional[np.ndarray]
    silhouette: Optional[np.ndarray]
    motion: Optional[np.ndarray]
    pose_ok: bool = False


@dataclass
class GaitScore:
    gait_similarity: float = -1.0
    body_similarity: float = -1.0
    silhouette_similarity: float = -1.0
    combined_similarity: float = -1.0
    target_gait_ready: bool = False
    live_gait_ready: bool = False
    pose_ok: bool = False
    available: bool = False
    reason: str = ""


class RunningGaitBuffer:
    def __init__(self, window: int = 45, min_frames: int = 18) -> None:
        self.window = max(3, int(window))
        self.min_frames = max(3, int(min_frames))
        self._buf: deque[np.ndarray] = deque(maxlen=self.window)

    def clear(self) -> None:
        self._buf.clear()

    def update(self, frame_feature: Optional[np.ndarray]) -> None:
        if frame_feature is None:
            return
        f = np.asarray(frame_feature, dtype=np.float32).reshape(-1)
        if f.size == 0:
            return
        self._buf.append(f)

    @property
    def ready(self) -> bool:
        return len(self._buf) >= self.min_frames

    def __len__(self) -> int:
        return len(self._buf)

    def signature(self) -> Optional[np.ndarray]:
        if not self.ready:
            return None
        data = np.stack(list(self._buf), axis=0).astype(np.float32)
        return np.concatenate([data.mean(axis=0), data.std(axis=0)], axis=0).astype(np.float32)


class PoseGaitExtractor:
    """Lightweight pose/body/gait extractor adapted from the uploaded robot_follow repo.

    It uses MediaPipe Pose on each person crop. If MediaPipe is missing or fails,
    the extractor disables itself gracefully and the identity manager falls back
    to ReID/color behavior.
    """

    _L_SHOULDER, _R_SHOULDER = 11, 12
    _L_ELBOW, _R_ELBOW = 13, 14
    _L_WRIST, _R_WRIST = 15, 16
    _L_HIP, _R_HIP = 23, 24
    _L_KNEE, _R_KNEE = 25, 26
    _L_ANKLE, _R_ANKLE = 27, 28

    def __init__(
        self,
        enabled: bool = True,
        model_complexity: int = 1,
        min_detection_confidence: float = 0.45,
        min_tracking_confidence: float = 0.45,
        min_landmark_visibility: float = 0.25,
    ) -> None:
        self.enabled = bool(enabled)
        self.available = False
        self.reason = "disabled"
        self.min_landmark_visibility = float(min_landmark_visibility)
        self.pose_model = None
        if not self.enabled:
            return
        try:
            import mediapipe as mp  # type: ignore

            self.pose_model = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=int(model_complexity),
                smooth_landmarks=True,
                min_detection_confidence=float(min_detection_confidence),
                min_tracking_confidence=float(min_tracking_confidence),
            )
            self.available = True
            self.reason = "mediapipe_pose_ready"
        except Exception as exc:
            self.available = False
            self.reason = f"mediapipe_unavailable: {exc}"

    def close(self) -> None:
        try:
            if self.pose_model is not None:
                self.pose_model.close()
        except Exception:
            pass

    @staticmethod
    def silhouette_profile(crop: np.ndarray) -> Optional[np.ndarray]:
        if crop is None or crop.size == 0:
            return None
        h, w = crop.shape[:2]
        if h < 24 or w < 10:
            return None
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        zones = np.array_split(bw, 8, axis=0)
        features: List[float] = []
        for zone in zones:
            if zone.size == 0:
                features.append(0.0)
                continue
            cols = (zone > 127).sum(axis=0)
            nonzero = np.where(cols > 0)[0]
            width = float(nonzero[-1] - nonzero[0]) / max(1, w) if len(nonzero) > 1 else 0.0
            features.append(width)
        arr = np.asarray(features, dtype=np.float32)
        if float(np.linalg.norm(arr)) < 1e-8:
            return None
        return arr


    @staticmethod
    def fallback_motion_from_silhouette(crop: np.ndarray, silh: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """6D gait fallback when MediaPipe Pose cannot lock enough joints.

        This uses temporal silhouette/body-shape information from the detected
        person crop. It is weaker than knee/hip pose gait, but keeps gait
        enrollment from getting stuck with low-angle / wide-angle / small crops.
        """
        if crop is None or crop.size == 0:
            return None
        h, w = crop.shape[:2]
        if h < 20 or w < 8:
            return None
        if silh is None:
            silh = PoseGaitExtractor.silhouette_profile(crop)
        if silh is None:
            return None
        arr = np.asarray(silh, dtype=np.float32).reshape(-1)
        if arr.size < 8:
            return None

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        fg_ratio = float((bw > 127).mean())

        # 6 numbers per frame. RunningGaitBuffer later stores mean+std across frames,
        # so the final target gait signature is 12D and changes with walking dynamics.
        feat = np.asarray([
            float(np.mean(arr[0:2])),        # head/shoulder zone width
            float(np.mean(arr[2:4])),        # torso zone width
            float(np.mean(arr[4:6])),        # hip/thigh zone width
            float(np.mean(arr[6:8])),        # lower-leg zone width
            float(w) / max(1.0, float(h)),   # bbox aspect ratio
            fg_ratio,                        # foreground density
        ], dtype=np.float32)
        if not np.all(np.isfinite(feat)) or float(np.linalg.norm(feat)) < 1e-8:
            return None
        return feat

    def _landmarks_good_enough(self, lm) -> bool:
        needed = [
            self._L_SHOULDER,
            self._R_SHOULDER,
            self._L_HIP,
            self._R_HIP,
            self._L_KNEE,
            self._R_KNEE,
            self._L_ANKLE,
            self._R_ANKLE,
        ]
        vis = [float(getattr(lm[i], "visibility", 1.0)) for i in needed]
        # Do not require every point to be perfect; rear-view pose often has a few weak landmarks.
        return float(np.mean(vis)) >= self.min_landmark_visibility

    @classmethod
    def body_proportions(cls, lm) -> Optional[np.ndarray]:
        def d(a: int, b: int) -> float:
            dx = float(lm[a].x - lm[b].x)
            dy = float(lm[a].y - lm[b].y)
            return math.sqrt(dx * dx + dy * dy) + 1e-7

        shoulder_w = d(cls._L_SHOULDER, cls._R_SHOULDER)
        hip_w = d(cls._L_HIP, cls._R_HIP)
        torso_h = 0.5 * (d(cls._L_SHOULDER, cls._L_HIP) + d(cls._R_SHOULDER, cls._R_HIP))
        l_arm = d(cls._L_SHOULDER, cls._L_ELBOW) + d(cls._L_ELBOW, cls._L_WRIST)
        r_arm = d(cls._R_SHOULDER, cls._R_ELBOW) + d(cls._R_ELBOW, cls._R_WRIST)
        l_leg = d(cls._L_HIP, cls._L_KNEE) + d(cls._L_KNEE, cls._L_ANKLE)
        r_leg = d(cls._R_HIP, cls._R_KNEE) + d(cls._R_KNEE, cls._R_ANKLE)
        total = torso_h + (l_leg + r_leg) / 2.0 + 1e-7
        feat = np.asarray(
            [
                shoulder_w / total,
                hip_w / total,
                shoulder_w / (hip_w + 1e-7),
                torso_h / total,
                l_arm / total,
                r_arm / total,
                l_leg / total,
                r_leg / total,
            ],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(feat)):
            return None
        return feat

    @classmethod
    def motion_frame(cls, lm) -> Optional[np.ndarray]:
        def angle(a: int, b: int, c: int) -> float:
            v1 = np.asarray([lm[a].x - lm[b].x, lm[a].y - lm[b].y], dtype=np.float32)
            v2 = np.asarray([lm[c].x - lm[b].x, lm[c].y - lm[b].y], dtype=np.float32)
            n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
            if n1 < 1e-7 or n2 < 1e-7:
                return 0.0
            cos_a = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
            return math.degrees(math.acos(cos_a)) / 180.0

        feat = np.asarray(
            [
                angle(cls._L_HIP, cls._L_KNEE, cls._L_ANKLE),
                angle(cls._R_HIP, cls._R_KNEE, cls._R_ANKLE),
                angle(cls._L_SHOULDER, cls._L_HIP, cls._L_KNEE),
                angle(cls._R_SHOULDER, cls._R_HIP, cls._R_KNEE),
                float(lm[cls._L_SHOULDER].y - lm[cls._R_SHOULDER].y),
                float(lm[cls._L_HIP].y - lm[cls._R_HIP].y),
            ],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(feat)):
            return None
        return feat

    def extract(self, frame_bgr: np.ndarray, bbox: BBox) -> GaitObservation:
        x1, y1, x2, y2 = _clip_bbox_with_pad(bbox, frame_bgr.shape)
        if x2 <= x1 + 8 or y2 <= y1 + 20:
            return GaitObservation(None, None, None, pose_ok=False)
        crop = frame_bgr[y1:y2, x1:x2]
        silh = self.silhouette_profile(crop)
        fallback_motion = self.fallback_motion_from_silhouette(crop, silh)

        if not self.available or self.pose_model is None:
            return GaitObservation(None, silh, fallback_motion, pose_ok=False)

        # MediaPipe Pose often fails when YOLO/DeepSORT crop is small. Upscale only
        # for pose inference. MediaPipe landmarks are normalized, so ratios remain valid.
        pose_crop = crop
        ph, pw = pose_crop.shape[:2]
        if ph < 320 or pw < 160:
            scale = max(1.0, 320.0 / max(1, ph), 160.0 / max(1, pw))
            scale = min(scale, 4.0)
            pose_crop = cv2.resize(
                pose_crop,
                (max(32, int(round(pw * scale))), max(64, int(round(ph * scale)))),
                interpolation=cv2.INTER_CUBIC,
            )
        try:
            rgb = cv2.cvtColor(pose_crop, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            res = self.pose_model.process(rgb)
        except Exception:
            return GaitObservation(None, silh, fallback_motion, pose_ok=False)

        if not res or not getattr(res, "pose_landmarks", None):
            return GaitObservation(None, silh, fallback_motion, pose_ok=False)
        lm = res.pose_landmarks.landmark
        if len(lm) < 29 or not self._landmarks_good_enough(lm):
            return GaitObservation(None, silh, fallback_motion, pose_ok=False)

        body = self.body_proportions(lm)
        motion = self.motion_frame(lm)
        if motion is None:
            motion = fallback_motion
        return GaitObservation(body, silh, motion, pose_ok=(body is not None and motion is not None))


class GaitIdentityVerifier:
    def __init__(
        self,
        enabled: bool = True,
        required_for_recovery: bool = True,
        required_for_enrollment: bool = False,
        window: int = 45,
        min_frames: int = 18,
        gallery_size: int = 80,
        body_weight: float = 0.25,
        silhouette_weight: float = 0.15,
        gait_weight: float = 0.60,
        model_complexity: int = 1,
        min_detection_confidence: float = 0.45,
        min_tracking_confidence: float = 0.45,
        min_landmark_visibility: float = 0.25,
    ) -> None:
        self.enabled = bool(enabled)
        self.required_for_recovery = bool(required_for_recovery)
        self.required_for_enrollment = bool(required_for_enrollment)
        self.window = max(3, int(window))
        self.min_frames = max(3, int(min_frames))
        self.gallery_size = max(3, int(gallery_size))
        self.target_buffer = RunningGaitBuffer(window=self.window, min_frames=self.min_frames)
        self.track_buffers: Dict[int, RunningGaitBuffer] = {}
        self.target_gait_signature: Optional[np.ndarray] = None
        self.target_body: deque[np.ndarray] = deque(maxlen=self.gallery_size)
        self.target_silhouette: deque[np.ndarray] = deque(maxlen=self.gallery_size)
        total = max(1e-9, float(body_weight) + float(silhouette_weight) + float(gait_weight))
        self.body_weight = float(body_weight) / total
        self.silhouette_weight = float(silhouette_weight) / total
        self.gait_weight = float(gait_weight) / total
        self.extractor = PoseGaitExtractor(
            enabled=self.enabled,
            model_complexity=model_complexity,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
            min_landmark_visibility=min_landmark_visibility,
        )

    @property
    def available(self) -> bool:
        return self.enabled and self.extractor.available

    @property
    def reason(self) -> str:
        return self.extractor.reason

    @property
    def target_ready(self) -> bool:
        return self.target_gait_signature is not None

    @property
    def target_sample_count(self) -> int:
        return len(self.target_buffer)

    @property
    def body_ready(self) -> bool:
        return len(self.target_body) > 0

    @property
    def silhouette_ready(self) -> bool:
        return len(self.target_silhouette) > 0

    def clear(self) -> None:
        self.target_buffer.clear()
        self.track_buffers.clear()
        self.target_gait_signature = None
        self.target_body.clear()
        self.target_silhouette.clear()

    def close(self) -> None:
        self.extractor.close()

    def _track_buffer(self, track_id: int) -> RunningGaitBuffer:
        tid = int(track_id)
        if tid not in self.track_buffers:
            self.track_buffers[tid] = RunningGaitBuffer(window=self.window, min_frames=self.min_frames)
        return self.track_buffers[tid]

    def prune_tracks(self, active_ids: Sequence[int]) -> None:
        active = {int(x) for x in active_ids}
        for tid in list(self.track_buffers.keys()):
            if tid not in active:
                del self.track_buffers[tid]

    def add_target_sample(self, frame_bgr: np.ndarray, bbox: BBox) -> GaitObservation:
        obs = self.extractor.extract(frame_bgr, bbox)
        if obs.body is not None:
            self.target_body.append(np.asarray(obs.body, dtype=np.float32))
        if obs.silhouette is not None:
            self.target_silhouette.append(np.asarray(obs.silhouette, dtype=np.float32))
        if obs.motion is not None:
            self.target_buffer.update(obs.motion)
            sig = self.target_buffer.signature()
            if sig is not None:
                self.target_gait_signature = sig
        return obs

    def finalize_target(self) -> bool:
        sig = self.target_buffer.signature()
        if sig is not None:
            self.target_gait_signature = sig
        return self.target_ready

    def progress(self) -> float:
        return float(max(0.0, min(1.0, len(self.target_buffer) / max(1.0, float(self.min_frames)))))

    def score_candidate(self, frame_bgr: np.ndarray, bbox: BBox, track_id: int) -> GaitScore:
        if not self.enabled:
            return GaitScore(reason="gait_disabled")
        obs = self.extractor.extract(frame_bgr, bbox)
        buf = self._track_buffer(track_id)
        if obs.motion is not None:
            buf.update(obs.motion)
        live_sig = buf.signature()
        gait_sim = _distance_similarity(live_sig, self.target_gait_signature) if self.target_ready and live_sig is not None else -1.0
        body_sim = _best_cos(obs.body, list(self.target_body)) if self.body_ready else -1.0
        silh_sim = _best_cos(obs.silhouette, list(self.target_silhouette)) if self.silhouette_ready else -1.0

        parts: List[Tuple[float, float]] = []
        if gait_sim >= 0:
            parts.append((gait_sim, self.gait_weight))
        if body_sim >= 0:
            parts.append((body_sim, self.body_weight))
        if silh_sim >= 0:
            parts.append((silh_sim, self.silhouette_weight))
        if parts:
            total = sum(w for _, w in parts)
            combined = sum(s * w for s, w in parts) / max(1e-9, total)
        else:
            combined = -1.0

        return GaitScore(
            gait_similarity=gait_sim,
            body_similarity=body_sim,
            silhouette_similarity=silh_sim,
            combined_similarity=float(combined),
            target_gait_ready=self.target_ready,
            live_gait_ready=live_sig is not None,
            pose_ok=obs.pose_ok,
            available=bool(parts),
            reason=self.reason,
        )
