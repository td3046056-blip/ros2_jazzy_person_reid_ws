from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np


def normalize_feature(feature: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Return an L2-normalized 1D feature vector."""
    arr = np.asarray(feature, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(arr))
    if norm < eps:
        return arr
    return arr / norm


class ReIDGallery:
    """Small rolling memory of ReID embeddings for one target person."""

    def __init__(self, max_size: int = 30) -> None:
        self.max_size = int(max_size)
        self._features: deque[np.ndarray] = deque(maxlen=max(1, self.max_size))

    def clear(self) -> None:
        self._features.clear()

    def add(self, feature: np.ndarray) -> None:
        if feature is None:
            return
        f = normalize_feature(feature)
        if f.size == 0:
            return
        self._features.append(f)

    def __len__(self) -> int:
        return len(self._features)

    @property
    def ready(self) -> bool:
        return len(self._features) > 0

    def mean(self) -> Optional[np.ndarray]:
        if not self._features:
            return None
        mean_feature = np.mean(np.stack(list(self._features), axis=0), axis=0)
        return normalize_feature(mean_feature)

    def similarity(self, feature: np.ndarray, mode: str = "max") -> float:
        """Cosine similarity between a candidate feature and the target memory.

        mode="max" is more tolerant when the target has changed viewpoint
        because it compares against the best stored target appearance.
        mode="mean" is stricter and compares against the average target feature.
        """
        if not self._features or feature is None:
            return -1.0
        cand = normalize_feature(feature)
        if cand.size == 0:
            return -1.0
        if mode == "mean":
            ref = self.mean()
            if ref is None:
                return -1.0
            return float(np.dot(ref, cand))
        sims = [float(np.dot(ref, cand)) for ref in self._features]
        return max(sims) if sims else -1.0


class ColorGallery:
    """Small rolling memory of HSV color histograms for one target person."""

    def __init__(self, max_size: int = 20) -> None:
        self.max_size = int(max_size)
        self._features: deque[np.ndarray] = deque(maxlen=max(1, self.max_size))

    def clear(self) -> None:
        self._features.clear()

    @property
    def ready(self) -> bool:
        return len(self._features) > 0

    def add(self, hist: Optional[np.ndarray]) -> None:
        if hist is None:
            return
        h = np.asarray(hist, dtype=np.float32).reshape(-1)
        s = float(np.sum(h))
        if h.size == 0 or s <= 1e-12:
            return
        self._features.append(h / s)

    def similarity(self, hist: Optional[np.ndarray]) -> float:
        if not self._features or hist is None:
            return -1.0
        cand = np.asarray(hist, dtype=np.float32).reshape(-1)
        s = float(np.sum(cand))
        if cand.size == 0 or s <= 1e-12:
            return -1.0
        cand = cand / s
        sims = []
        for ref in self._features:
            # Histogram intersection, range [0, 1] for normalized histograms.
            sims.append(float(np.minimum(ref, cand).sum()))
        return max(sims) if sims else -1.0
