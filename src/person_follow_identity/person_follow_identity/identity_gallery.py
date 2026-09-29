from __future__ import annotations

from collections import OrderedDict, deque
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

    Moi mau co mot "tag" = kieu khung (xem view_bucket): toan than / bi cat dau /
    bi cat chan. Nguoi dung gan xe bi cat khung, ma so anh nua duoi voi gallery
    toan than thi nguoi that chi con ~0.79 (lan vao vung nguoi la ~0.72); so voi
    mau cung kieu cat thi ~0.89 (do tren Market-1501). Khi tag du mau, so trong tag.

    Mau enroll duoc "neo" (freeze_anchor) de mau hoc online khong day chung ra.
    """

    def __init__(self, max_size: int = 120) -> None:
        self.max_size = max(1, int(max_size))
        self._features: deque[np.ndarray] = deque(maxlen=self.max_size)
        self._tags: deque[int] = deque(maxlen=self.max_size)
        self._anchor: List[np.ndarray] = []
        self._anchor_tags: List[int] = []

    def clear(self) -> None:
        self._features.clear()
        self._tags.clear()
        self._anchor.clear()
        self._anchor_tags.clear()

    def __len__(self) -> int:
        return len(self._anchor) + len(self._features)

    @property
    def ready(self) -> bool:
        return len(self) > 0

    def as_list(self) -> List[np.ndarray]:
        return self._anchor + list(self._features)

    def freeze_anchor(self) -> None:
        """Chuyen toan bo mau hien co thanh mau neo (khong bi mau online day ra)."""
        self._anchor.extend(self._features)
        self._anchor_tags.extend(self._tags)
        self._features.clear()
        self._tags.clear()

    def tag_count(self, tag: int) -> int:
        return sum(1 for t in self._anchor_tags if t == tag) + sum(1 for t in self._tags if t == tag)

    def _select(self, tag: Optional[int], min_tag_samples: int) -> List[np.ndarray]:
        feats = self.as_list()
        if tag is None or min_tag_samples <= 0:
            return feats
        tags = self._anchor_tags + list(self._tags)
        sel = [f for f, t in zip(feats, tags) if t == tag]
        return sel if len(sel) >= min_tag_samples else feats

    def add(self, feature: Optional[np.ndarray], tag: int = 0) -> bool:
        if feature is None:
            return False
        f = normalize_feature(feature)
        if f.size == 0:
            return False
        self._features.append(f)
        self._tags.append(int(tag))
        return True

    def add_diverse(
        self,
        feature: Optional[np.ndarray],
        min_similarity_to_accept: float = -1.0,
        max_similarity_to_last: float = 0.985,
        tag: int = 0,
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
        if len(self) > 0:
            if min_similarity_to_accept > -1.0:
                sim = self.similarity(f, mode="topk_mean", top_k=5)
                if sim < min_similarity_to_accept:
                    return False
            last = self._features[-1] if self._features else self._anchor[-1]
            last_sim = float(np.dot(last, f))
            if last_sim > max_similarity_to_last:
                return False
        self._features.append(f)
        self._tags.append(int(tag))
        return True

    def mean(self) -> Optional[np.ndarray]:
        feats = self.as_list()
        if not feats:
            return None
        mean_feature = np.mean(np.stack(feats, axis=0), axis=0)
        return normalize_feature(mean_feature)

    def similarities(self, feature: Optional[np.ndarray], tag: Optional[int] = None, min_tag_samples: int = 0) -> List[float]:
        feats = self._select(tag, min_tag_samples)
        if not feats or feature is None:
            return []
        cand = normalize_feature(feature)
        if cand.size == 0:
            return []
        # Nhan tung phan tu + cong, KHONG dung "@": tich ma tran goi OpenBLAS da luong, cac
        # luong do quay cho ban sau moi lan goi va tranh CPU voi torch -> YOLO cham ~3 lan.
        return (np.stack(feats, axis=0) * cand[None, :]).sum(axis=1).astype(float).tolist()

    def similarity(
        self,
        feature: Optional[np.ndarray],
        mode: str = "topk_mean",
        top_k: int = 5,
        tag: Optional[int] = None,
        min_tag_samples: int = 0,
    ) -> float:
        sims = self.similarities(feature, tag=tag, min_tag_samples=min_tag_samples)
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


class NegativeMemory:
    """Bo nho nguoi KHONG phai muc tieu, tach rieng TUNG nguoi (theo track id).

    Diem = max qua tung nguoi cua top-k mean trong mau cua nguoi do. Gop chung
    vao mot deque nhu truoc thi mot nguoi dung lau trong khung lap day bo nho, va
    "max qua moi mau" lech cao hon top-k mean cua gallery muc tieu -> muc tieu
    that bi loai vi "giong nguoi da biet".
    """

    def __init__(self, max_size: int = 160, per_person: int = 24, max_people: int = 16) -> None:
        self.max_size = max(1, int(max_size))
        self.per_person = max(2, int(per_person))
        self.max_people = max(1, int(max_people))
        self._people: "OrderedDict[int, FeatureGallery]" = OrderedDict()

    def clear(self) -> None:
        self._people.clear()

    def __len__(self) -> int:
        return sum(len(g) for g in self._people.values())

    def add(self, feature: Optional[np.ndarray], source_id: int, tag: int = 0, max_similarity_to_last: float = 0.985) -> bool:
        if feature is None:
            return False
        sid = int(source_id)
        gal = self._people.get(sid)
        if gal is None:
            gal = FeatureGallery(max_size=self.per_person)
            self._people[sid] = gal
        self._people.move_to_end(sid)
        ok = gal.add_diverse(feature, max_similarity_to_last=max_similarity_to_last, tag=tag)
        while len(self._people) > self.max_people or (len(self) > self.max_size and len(self._people) > 1):
            self._people.popitem(last=False)
        return ok

    def similarity(self, feature: Optional[np.ndarray], tag: Optional[int] = None, top_k: int = 3, min_tag_samples: int = 2) -> float:
        best = -1.0
        for gal in self._people.values():
            s = gal.similarity(feature, mode="topk_mean", top_k=top_k, tag=tag, min_tag_samples=min_tag_samples)
            best = max(best, s)
        return best


class ColorGallery:
    """Rolling HSV color histograms.

    The target memory stores two histograms: upper body and lower body. This is
    more robust than one full-body histogram when background or floor pixels leak
    into the crop.

    Co tag kieu khung + mau neo giong FeatureGallery: vung "than tren" cua anh bi cat
    dau thuc ra la hong/dui, so voi mau toan than la so ao voi quan.
    """

    def __init__(self, max_size: int = 80) -> None:
        self.max_size = max(1, int(max_size))
        self._features: deque[np.ndarray] = deque(maxlen=self.max_size)
        self._tags: deque[int] = deque(maxlen=self.max_size)
        self._anchor: List[np.ndarray] = []
        self._anchor_tags: List[int] = []

    def clear(self) -> None:
        self._features.clear()
        self._tags.clear()
        self._anchor.clear()
        self._anchor_tags.clear()

    def __len__(self) -> int:
        return len(self._anchor) + len(self._features)

    @property
    def ready(self) -> bool:
        return len(self) > 0

    def freeze_anchor(self) -> None:
        self._anchor.extend(self._features)
        self._anchor_tags.extend(self._tags)
        self._features.clear()
        self._tags.clear()

    def add(self, hist: Optional[np.ndarray], tag: int = 0) -> bool:
        if hist is None:
            return False
        h = np.asarray(hist, dtype=np.float32).reshape(-1)
        s = float(np.sum(h))
        if h.size == 0 or s <= 1e-12:
            return False
        self._features.append(h / s)
        self._tags.append(int(tag))
        return True

    def similarity(
        self,
        hist: Optional[np.ndarray],
        mode: str = "max",
        top_k: int = 5,
        tag: Optional[int] = None,
        min_tag_samples: int = 0,
    ) -> float:
        refs = self._anchor + list(self._features)
        if tag is not None and min_tag_samples > 0:
            tags = self._anchor_tags + list(self._tags)
            sel = [r for r, t in zip(refs, tags) if t == tag]
            if len(sel) >= min_tag_samples:
                refs = sel
        if not refs or hist is None:
            return -1.0
        cand = np.asarray(hist, dtype=np.float32).reshape(-1)
        s = float(np.sum(cand))
        if cand.size == 0 or s <= 1e-12:
            return -1.0
        cand = cand / s
        sims = np.minimum(np.stack(refs, axis=0), cand[None, :]).sum(axis=1).astype(float).tolist()
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


def view_bucket(bbox: BBox, frame_shape: Tuple[int, int, int], margin_px: int = 6) -> int:
    """Kieu khung cua nguoi: 0 toan than, 1 bi cat dau, 2 bi cat chan, 3 cat ca hai.

    Nguoi dung gan camera (~1 m) khong vua khung doc nen luon bi cat mot hoac hai dau.
    """
    h = int(frame_shape[0])
    top_cut = int(bbox[1]) <= margin_px
    bottom_cut = int(bbox[3]) >= h - 1 - margin_px
    return (1 if top_cut else 0) + (2 if bottom_cut else 0)


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
