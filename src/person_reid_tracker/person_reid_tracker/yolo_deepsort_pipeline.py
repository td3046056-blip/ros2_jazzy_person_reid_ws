from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional, Sequence

import cv2
import numpy as np
import torch

from .types import BBox, TrackCandidate

# The bundled YOLOv5 code uses absolute imports such as `from models.common import ...`.
# `models` and `utils` are installed as top-level packages by this ROS package.
from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_coords, xyxy2xywh
from tracking_module.deep_sort import DeepSort


class YoloDeepSortPipeline:
    """YOLOv5 person detection + DeepSORT tracking + ReID feature extraction."""

    def __init__(
        self,
        model_weights: str,
        deepsort_ckpt: str,
        device: str = "cpu",
        use_cuda: bool = False,
        img_size: int = 640,
        conf_thres: float = 0.5,
        iou_thres: float = 0.5,
        person_class_id: int = 0,
        deepsort_max_age: int = 70,
        deepsort_n_init: int = 3,
        deepsort_max_dist: float = 0.2,
    ) -> None:
        self.model_weights = str(Path(model_weights).expanduser())
        self.deepsort_ckpt = str(Path(deepsort_ckpt).expanduser())
        self.img_size = int(img_size)
        self.conf_thres = float(conf_thres)
        self.iou_thres = float(iou_thres)
        self.person_class_id = int(person_class_id)

        if device == "cuda" and not torch.cuda.is_available():
            device = "cpu"
        self.device = torch.device(device)
        self.use_cuda = bool(use_cuda and self.device.type == "cuda")

        self.model = DetectMultiBackend(weights=self.model_weights, device=self.device)
        self.deepsort = DeepSort(
            model_path=self.deepsort_ckpt,
            use_cuda=self.use_cuda,
            max_age=int(deepsort_max_age),
            n_init=int(deepsort_n_init),
            max_dist=float(deepsort_max_dist),
        )

    @staticmethod
    def letterbox(img: np.ndarray, new_shape=(640, 640), color=(114, 114, 114)) -> np.ndarray:
        shape = img.shape[:2]
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        dw /= 2
        dh /= 2
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))
        return cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    def _preprocess(self, frame_bgr: np.ndarray) -> torch.Tensor:
        img = self.letterbox(frame_bgr, (self.img_size, self.img_size))
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR -> RGB, HWC -> CHW
        img = np.ascontiguousarray(img)
        tensor = torch.from_numpy(img).to(self.device).float() / 255.0
        if tensor.ndimension() == 3:
            tensor = tensor.unsqueeze(0)
        return tensor

    @staticmethod
    def _clip_bbox(bbox: Sequence[float], width: int, height: int) -> BBox:
        x1, y1, x2, y2 = [int(round(float(v))) for v in bbox]
        x1 = max(0, min(width - 1, x1))
        y1 = max(0, min(height - 1, y1))
        x2 = max(0, min(width - 1, x2))
        y2 = max(0, min(height - 1, y2))
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        return (x1, y1, x2, y2)

    @staticmethod
    def _iou_xyxy(a: Sequence[float], b: Sequence[float]) -> float:
        ax1, ay1, ax2, ay2 = [float(v) for v in a]
        bx1, by1, bx2, by2 = [float(v) for v in b]
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        denom = area_a + area_b - inter
        return 0.0 if denom <= 0 else inter / denom

    def _match_track_conf(self, bbox: BBox, det_boxes: List[np.ndarray], det_confs: List[float]) -> float:
        best_iou = 0.0
        best_conf = 0.0
        for det_box, det_conf in zip(det_boxes, det_confs):
            iou = self._iou_xyxy(bbox, det_box)
            if iou > best_iou:
                best_iou = iou
                best_conf = float(det_conf)
        return best_conf

    def detect_and_track(self, frame_bgr: np.ndarray) -> List[TrackCandidate]:
        """Return current DeepSORT tracks for person detections."""
        h, w = frame_bgr.shape[:2]
        img_tensor = self._preprocess(frame_bgr)
        pred = self.model(img_tensor)
        pred = non_max_suppression(
            pred,
            self.conf_thres,
            self.iou_thres,
            classes=self.person_class_id,
        )

        tracks: List[TrackCandidate] = []
        for det in pred:
            if det is None or len(det) == 0:
                continue
            det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], frame_bgr.shape).round()
            xywhs = xyxy2xywh(det[:, 0:4])
            confs = det[:, 4]
            clss = det[:, 5]
            det_boxes_xyxy = [box.detach().cpu().numpy() for box in det[:, 0:4]]
            det_confs = [float(c) for c in confs.detach().cpu().numpy()]

            outputs = self.deepsort.update(xywhs.detach().cpu(), confs.detach().cpu(), clss.detach().cpu(), frame_bgr)
            if outputs is None or len(outputs) == 0:
                continue

            for output in outputs:
                x1, y1, x2, y2, track_id, cls = output[:6]
                bbox = self._clip_bbox((x1, y1, x2, y2), w, h)
                conf = self._match_track_conf(bbox, det_boxes_xyxy, det_confs)
                tracks.append(TrackCandidate(track_id=int(track_id), bbox=bbox, cls=int(cls), conf=conf))

        return tracks

    def extract_features(self, frame_bgr: np.ndarray, bboxes: List[BBox]) -> List[Optional[np.ndarray]]:
        """Extract DeepSORT ReID features for image crops in xyxy format."""
        h, w = frame_bgr.shape[:2]
        crops = []
        valid_map = []
        for bbox in bboxes:
            x1, y1, x2, y2 = self._clip_bbox(bbox, w, h)
            if x2 <= x1 + 2 or y2 <= y1 + 2:
                crops.append(None)
                valid_map.append(False)
                continue
            crops.append(frame_bgr[y1:y2, x1:x2])
            valid_map.append(True)

        result: List[Optional[np.ndarray]] = [None] * len(bboxes)
        valid_crops = [c for c, ok in zip(crops, valid_map) if ok and c is not None]
        if not valid_crops:
            return result
        features = self.deepsort.extractor(valid_crops)
        feature_iter = iter(features)
        for i, ok in enumerate(valid_map):
            if ok:
                result[i] = np.asarray(next(feature_iter), dtype=np.float32)
        return result
