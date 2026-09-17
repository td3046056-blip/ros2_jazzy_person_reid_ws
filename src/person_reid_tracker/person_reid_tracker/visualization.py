from __future__ import annotations

from typing import Iterable, Optional

import cv2
import numpy as np

from .target_lock import TargetResult
from .types import TrackCandidate


def draw_debug_frame(
    frame: np.ndarray,
    tracks: Iterable[TrackCandidate],
    result: TargetResult,
    draw_all_tracks: bool = False,
) -> np.ndarray:
    out = frame.copy()

    if draw_all_tracks:
        for t in tracks:
            x1, y1, x2, y2 = t.bbox
            cv2.rectangle(out, (x1, y1), (x2, y2), (90, 90, 90), 1)
            cv2.putText(out, f"ID {t.track_id}", (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

    if result.target is not None:
        t = result.target
        x1, y1, x2, y2 = t.bbox
        color = (0, 255, 0) if not result.recovered else (0, 255, 255)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f"TARGET ID {t.track_id} {result.status}"
        if result.combined_similarity >= 0:
            label += f" score={result.combined_similarity:.2f}"
        elif result.reid_similarity >= 0:
            label += f" reid={result.reid_similarity:.2f}"
        cv2.putText(out, label, (x1, max(25, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cx, cy = t.center
        cv2.circle(out, (int(cx), int(cy)), 4, color, -1)
    else:
        cv2.putText(out, f"NO TARGET: {result.status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if result.reason:
            cv2.putText(out, result.reason[:80], (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    return out
