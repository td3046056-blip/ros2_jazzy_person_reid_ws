from __future__ import annotations

from typing import Iterable

import cv2
import numpy as np

from person_reid_tracker.types import TrackCandidate

from .identity_lock_manager import IdentityResult


def _bar(out: np.ndarray, x: int, y: int, w: int, h: int, value: float) -> None:
    value = max(0.0, min(1.0, float(value)))
    cv2.rectangle(out, (x, y), (x + w, y + h), (60, 60, 60), 1)
    cv2.rectangle(out, (x, y), (x + int(w * value), y + h), (0, 180, 255), -1)


def draw_identity_debug_frame(
    frame: np.ndarray,
    tracks: Iterable[TrackCandidate],
    result: IdentityResult,
    draw_all_tracks: bool = True,
) -> np.ndarray:
    out = frame.copy()

    if draw_all_tracks:
        for t in tracks:
            x1, y1, x2, y2 = t.bbox
            cv2.rectangle(out, (x1, y1), (x2, y2), (95, 95, 95), 1)
            cv2.putText(out, f"ID {t.track_id} a={t.area}", (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1)

    if result.target is not None:
        t = result.target
        x1, y1, x2, y2 = t.bbox
        if result.status.startswith("ENROLL"):
            color = (0, 180, 255)
            label = f"ENROLL ID {t.track_id} {result.enrolled_samples} samples"
        elif result.recovered:
            color = (0, 255, 255)
            label = f"RECOVERED ID {t.track_id}"
        elif result.verified:
            color = (0, 255, 0)
            label = f"TARGET ID {t.track_id}"
        else:
            color = (0, 0, 255)
            label = f"SUSPECT ID {t.track_id}"
        if result.combined_similarity >= 0:
            label += f" c={result.combined_similarity:.2f} r={result.reid_similarity:.2f}"
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(out, label, (x1, max(25, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cx, cy = t.center
        cv2.circle(out, (int(cx), int(cy)), 4, color, -1)

    cv2.rectangle(out, (0, 0), (out.shape[1], 90), (0, 0, 0), -1)
    cv2.putText(out, f"{result.status}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    msg = (
        f"pos={result.reid_similarity:.2f} color={result.color_similarity:.2f} "
        f"gait={result.gait_similarity:.2f} neg={result.negative_similarity:.2f} "
        f"margin={result.identity_margin:.2f}"
    )
    cv2.putText(out, msg, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    gait_txt = "gait:ready" if result.gait_ready else "gait:collecting"
    cv2.putText(out, f"samples={result.enrolled_samples} neg={result.negative_samples} {gait_txt}  e:start  f:finish  r:reset  q:quit", (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    if result.status.startswith("ENROLL"):
        _bar(out, 430, 60, 180, 14, result.enrollment_progress)
    if result.target is None and result.reason:
        cv2.putText(out, result.reason[:90], (10, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return out
