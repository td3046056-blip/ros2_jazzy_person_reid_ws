from __future__ import annotations

from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from person_reid_tracker.types import BBox


def normalize_feature(feature: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    arr = np.asarray(feature, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(arr))
    if norm < eps:
        return arr
    return arr / norm


class FeatureGallery:
    """Rolling gallery for ReID vectors.

    It supports stricter matching than a raw max cosine score:
    - topk_mean: average of the top K similarities, stable against one bad sample.
    - mean: similarity to the gallery centroid.
    - max: permissive fallback.

    add_diverse() avoids filling the gallery with near-duplicate consecutive frames.
    """

    def __init__(self, max_size: int = 120) -> None:
        self.max_size = max(1, int(max_size))
        self._features: deque[np.ndarray] = deque(maxlen=self.max_size)

    def clear(self) -> None:
        self._features.clear()

    def __len__(self) -> int:
        return len(self._features)

    @property
    def ready(self) -> bool:
        return len(self._features) > 0

    def as_list(self) -> List[np.ndarray]:
        return list(self._features)

    def add(self, feature: Optional[np.ndarray]) -> bool:
        if feature is None:
            return False
        f = normalize_feature(feature)
        if f.size == 0:
            return False
        self._features.append(f)
        return True

    def add_diverse(
        self,
        feature: Optional[np.ndarray],
        min_similarity_to_accept: float = -1.0,
        max_similarity_to_last: float = 0.985,
    ) -> bool:
        """Add only useful samples.

        min_similarity_to_accept is used after the gallery already has samples. It
        rejects obvious outliers before they poison the target memory.
        max_similarity_to_last rejects almost-identical frames.
        """
        if feature is None:
            return False
        f = normalize_feature(feature)
        if f.size == 0:
            return False
        if self._features:
            if min_similarity_to_accept > -1.0:
                sim = self.similarity(f, mode="topk_mean", top_k=5)
                if sim < min_similarity_to_accept:
                    return False
            last_sim = float(np.dot(self._features[-1], f))
            if last_sim > max_similarity_to_last:
                return False
        self._features.append(f)
        return True

    def mean(self) -> Optional[np.ndarray]:
        if not self._features:
            return None
        mean_feature = np.mean(np.stack(list(self._features), axis=0), axis=0)
        return normalize_feature(mean_feature)

    def similarities(self, feature: Optional[np.ndarray]) -> List[float]:
        if not self._features or feature is None:
            return []
        cand = normalize_feature(feature)
        if cand.size == 0:
            return []
        return [float(np.dot(ref, cand)) for ref in self._features]

    def similarity(self, feature: Optional[np.ndarray], mode: str = "topk_mean", top_k: int = 5) -> float:
        sims = self.similarities(feature)
        if not sims:
            return -1.0
        mode = str(mode).lower()
        if mode == "mean":
            ref = self.mean()
            if ref is None:
                return -1.0
            return float(np.dot(ref, normalize_feature(feature)))
        if mode == "max":
            return max(sims)
        # default: top-k mean
        sims.sort(reverse=True)
        k = max(1, min(int(top_k), len(sims)))
        return float(np.mean(sims[:k]))


class ColorGallery:
    """Rolling HSV color histograms.

    The target memory stores two histograms: upper body and lower body. This is
    more robust than one full-body histogram when background or floor pixels leak
    into the crop.
    """

    def __init__(self, max_size: int = 80) -> None:
        self.max_size = max(1, int(max_size))
        self._features: deque[np.ndarray] = deque(maxlen=self.max_size)

    def clear(self) -> None:
        self._features.clear()

    def __len__(self) -> int:
        return len(self._features)

    @property
    def ready(self) -> bool:
        return len(self._features) > 0

    def add(self, hist: Optional[np.ndarray]) -> bool:
        if hist is None:
            return False
        h = np.asarray(hist, dtype=np.float32).reshape(-1)
        s = float(np.sum(h))
        if h.size == 0 or s <= 1e-12:
            return False
        self._features.append(h / s)
        return True

    def similarity(self, hist: Optional[np.ndarray], mode: str = "max", top_k: int = 5) -> float:
        if not self._features or hist is None:
            return -1.0
        cand = np.asarray(hist, dtype=np.float32).reshape(-1)
        s = float(np.sum(cand))
        if cand.size == 0 or s <= 1e-12:
            return -1.0
        cand = cand / s
        sims = [float(np.minimum(ref, cand).sum()) for ref in self._features]
        if not sims:
            return -1.0
        mode = str(mode).lower()
        if mode == "mean":
            return float(np.mean(sims))
        if mode == "topk_mean":
            sims.sort(reverse=True)
            k = max(1, min(int(top_k), len(sims)))
            return float(np.mean(sims[:k]))
        return float(max(sims))


def _clip_bbox(bbox: BBox, frame_shape: Tuple[int, int, int]) -> BBox:
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = bbox
    x1 = max(0, min(w - 1, int(x1)))
    x2 = max(0, min(w - 1, int(x2)))
    y1 = max(0, min(h - 1, int(y1)))
    y2 = max(0, min(h - 1, int(y2)))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return x1, y1, x2, y2


def person_color_histogram(frame_bgr: np.ndarray, bbox: BBox) -> Optional[np.ndarray]:
    """Return concatenated upper/lower HSV histograms for one person bbox."""
    x1, y1, x2, y2 = _clip_bbox(bbox, frame_bgr.shape)
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    if bw < 8 or bh < 20:
        return None

    # Use body center, avoid head/feet/background.
    cx1 = x1 + int(0.18 * bw)
    cx2 = x2 - int(0.18 * bw)
    upper_y1 = y1 + int(0.20 * bh)
    upper_y2 = y1 + int(0.55 * bh)
    lower_y1 = y1 + int(0.55 * bh)
    lower_y2 = y1 + int(0.88 * bh)

    parts = [(cx1, upper_y1, cx2, upper_y2), (cx1, lower_y1, cx2, lower_y2)]
    hists: List[np.ndarray] = []
    for px1, py1, px2, py2 in parts:
        if px2 <= px1 + 4 or py2 <= py1 + 4:
            return None
        crop = frame_bgr[py1:py2, px1:px2]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [24, 16], [0, 180, 0, 256]).astype(np.float32)
        s = float(hist.sum())
        if s <= 1e-12:
            return None
        hists.append((hist / s).reshape(-1))
    # Equal weight upper and lower body.
    return np.concatenate(hists, axis=0).astype(np.float32)
