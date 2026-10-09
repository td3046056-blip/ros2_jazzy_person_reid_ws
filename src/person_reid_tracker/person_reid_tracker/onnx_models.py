"""Detector YOLO + trich dac trung ReID OSNet chay bang ONNX Runtime (09/10).

Ly do (do tren may Ryzen 9, ghim 2 loi de gia lap mini PC i5-5200U 2 loi / 4 luong):
  - YOLOv5n 640x480: PyTorch 22 ms -> onnxruntime 14 ms; 416x320: 6 ms
  - ReID: mang cua DeepSORT (ckpt.t7, PyTorch) 13 / 32 / 67 ms cho 1 / 3 / 6 nguoi
          OSNet x0.25 (onnxruntime) 1.2 / 2.6 / 4.7 ms — va tach nguoi tot hon (xem eval_reid_data.py)
OpenCV DNN khong doc duoc ONNX cua YOLOv5n / OSNet va chay YOLO26n cham gap 3 -> dung onnxruntime.
onnxruntime 1.31 chay duoc voi numpy 1.26 cua ROS (cai: pip install --user --no-deps onnxruntime).
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

try:
    import onnxruntime as ort
except ImportError:          # bao loi ro rang khi cau hinh chon ONNX ma chua cai
    ort = None


def _session(path: str, threads: int):
    if ort is None:
        raise RuntimeError("Chua cai onnxruntime: pip install --user --no-deps onnxruntime "
                           "(hoac de trong detector_onnx / reid_onnx de dung PyTorch nhu cu)")
    o = ort.SessionOptions()
    o.intra_op_num_threads = max(1, int(threads))
    o.inter_op_num_threads = 1
    o.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    # Khong cho luong cho 'quay ban' (spin): detector + ReID la 2 phien, moi phien mot bo luong; tren 2 loi
    # (mini PC) chung tranh nhau -> YOLOv5n 14 ms (do rieng) thanh ~36 ms trong pipeline (do 09/10).
    o.add_session_config_entry("session.intra_op.allow_spinning", "0")
    return ort.InferenceSession(path, o, providers=["CPUExecutionProvider"])


class OnnxReidExtractor:
    """Thay the Extractor (ckpt.t7) cua DeepSORT: cung giao dien __call__(list crop BGR) -> (N, D).

    OSNet (torchreid) hoc tren anh RGB 256x128, chuan hoa ImageNet."""

    MEAN = np.array([0.485, 0.456, 0.406], np.float32)
    STD = np.array([0.229, 0.224, 0.225], np.float32)

    def __init__(self, model_path: str, threads: int = 2, batch: int = 16) -> None:
        self.sess = _session(model_path, threads)
        inp = self.sess.get_inputs()[0]
        self.inp = inp.name
        self.h, self.w = int(inp.shape[2]), int(inp.shape[3])
        self.batch = max(1, int(batch))

    def __call__(self, im_crops: List[Optional[np.ndarray]]) -> np.ndarray:
        crops = [c for c in im_crops if c is not None and c.size > 0]
        if not crops:
            return np.array([])
        x = np.empty((len(crops), 3, self.h, self.w), np.float32)
        for i, c in enumerate(crops):
            im = cv2.resize(c, (self.w, self.h), interpolation=cv2.INTER_LINEAR)[:, :, ::-1].astype(np.float32) / 255.0
            x[i] = ((im - self.MEAN) / self.STD).transpose(2, 0, 1)
        out = [self.sess.run(None, {self.inp: x[i:i + self.batch]})[0] for i in range(0, len(x), self.batch)]
        return np.concatenate(out).astype(np.float32)


class OnnxYoloDetector:
    """YOLO xuat ONNX, kich thuoc vao CO DINH (vd. 640x480, 416x320). Tu nhan dang dau ra:
      YOLOv5 (1, N, 85): cx cy w h obj cls...   diem = obj * cls
      YOLOv8/11/26 (1, 84, N): cx cy w h cls... diem = cls
    Tra ve (xyxy trong toa do anh goc, conf) cua lop nguoi, da NMS."""

    def __init__(self, model_path: str, conf_thres: float = 0.4, iou_thres: float = 0.5,
                 class_id: int = 0, threads: int = 2) -> None:
        self.sess = _session(model_path, threads)
        inp = self.sess.get_inputs()[0]
        self.inp = inp.name
        self.h, self.w = int(inp.shape[2]), int(inp.shape[3])
        self.conf = float(conf_thres)
        self.iou = float(iou_thres)
        self.cls = int(class_id)

    def _letterbox(self, img: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
        s = min(self.w / img.shape[1], self.h / img.shape[0])
        nw, nh = int(round(img.shape[1] * s)), int(round(img.shape[0] * s))
        px, py = (self.w - nw) // 2, (self.h - nh) // 2
        if (nw, nh) == (self.w, self.h):
            canvas = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR) if s != 1.0 else img
        else:
            canvas = np.full((self.h, self.w, 3), 114, np.uint8)
            canvas[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = canvas[:, :, ::-1].astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        return np.ascontiguousarray(x), s, px, py

    def __call__(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x, s, px, py = self._letterbox(frame_bgr)
        y = self.sess.run(None, {self.inp: x})[0][0]
        if y.shape[0] < y.shape[1] and y.shape[0] <= 128:        # (84, N) kieu YOLOv8/11/26
            y = y.T
            score = y[:, 4 + self.cls]
        else:                                                    # (N, 85) kieu YOLOv5
            score = y[:, 4] * y[:, 5 + self.cls]
        keep = score >= self.conf
        if not keep.any():
            return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)
        b, score = y[keep, :4], score[keep]
        x1 = (b[:, 0] - b[:, 2] / 2 - px) / s
        y1 = (b[:, 1] - b[:, 3] / 2 - py) / s
        bw, bh = b[:, 2] / s, b[:, 3] / s
        idx = np.array(cv2.dnn.NMSBoxes(np.stack([x1, y1, bw, bh], 1).tolist(), score.tolist(), self.conf, self.iou)).reshape(-1)
        H, W = frame_bgr.shape[:2]
        xyxy = np.stack([x1[idx], y1[idx], x1[idx] + bw[idx], y1[idx] + bh[idx]], 1)
        xyxy[:, [0, 2]] = xyxy[:, [0, 2]].clip(0, W - 1)
        xyxy[:, [1, 3]] = xyxy[:, [1, 3]].clip(0, H - 1)
        return xyxy.astype(np.float32), score[idx].astype(np.float32)
