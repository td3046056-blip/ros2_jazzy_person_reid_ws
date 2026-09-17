from __future__ import annotations

import json
import math
import statistics
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


def _finite(value: Any) -> Optional[float]:
    try:
        v = float(value)
    except Exception:
        return None
    return v if math.isfinite(v) else None


class FollowDistanceCalibrator(Node):
    """Collect target bbox height and sonar raw readings while the target stands at a known distance."""

    def __init__(self) -> None:
        super().__init__("follow_distance_calibrator")
        self.declare_parameter("target_topic", "/person_reid/target")
        self.declare_parameter("sonar_topic", "/bw_dr03/sonar")
        self.declare_parameter("target_distance_m", 1.0)
        self.declare_parameter("sample_seconds", 10.0)
        self.declare_parameter("print_period_sec", 1.0)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.sonar_topic = str(self.get_parameter("sonar_topic").value)
        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.sample_seconds = float(self.get_parameter("sample_seconds").value)
        self.print_period_sec = float(self.get_parameter("print_period_sec").value)

        self.bbox_heights: List[float] = []
        self.sonar_raw: List[float] = []
        self.started = time.time()
        self.last_print = 0.0

        self.target_sub = self.create_subscription(String, self.target_topic, self._target_cb, 10)
        self.sonar_sub = self.create_subscription(Float32MultiArray, self.sonar_topic, self._sonar_cb, 10)
        self.timer = self.create_timer(0.2, self._timer_cb)

        self.get_logger().info(
            "Calibrating %.2fm distance for %.1fs. Enroll and track person A, then keep A at exactly this distance."
            % (self.target_distance_m, self.sample_seconds)
        )

    def _target_cb(self, msg: String) -> None:
        try:
            payload: Dict[str, Any] = json.loads(msg.data)
        except Exception:
            return
        if not payload.get("target_found", False):
            return
        bbox = payload.get("bbox")
        if not isinstance(bbox, list) or len(bbox) != 4:
            return
        y1 = _finite(bbox[1])
        y2 = _finite(bbox[3])
        if y1 is None or y2 is None:
            return
        h = abs(y2 - y1)
        if h > 10.0:
            self.bbox_heights.append(h)

    def _sonar_cb(self, msg: Float32MultiArray) -> None:
        vals = [_finite(x) for x in msg.data]
        vals = [v for v in vals if v is not None and v > 0.0]
        if vals:
            self.sonar_raw.append(min(vals))

    def _timer_cb(self) -> None:
        now = time.time()
        if now - self.last_print < self.print_period_sec:
            return
        self.last_print = now
        elapsed = now - self.started
        bbox_median = statistics.median(self.bbox_heights) if self.bbox_heights else None
        sonar_median = statistics.median(self.sonar_raw) if self.sonar_raw else None
        msg = "samples: bbox=%d sonar=%d" % (len(self.bbox_heights), len(self.sonar_raw))
        if bbox_median is not None:
            msg += ", set bbox_height_at_target_distance_px: %.1f" % bbox_median
        if sonar_median is not None and sonar_median > 0.0:
            msg += ", set sonar_scale_to_m: %.6f (if target is %.2fm)" % (self.target_distance_m / sonar_median, self.target_distance_m)
        self.get_logger().info(msg)
        if elapsed >= self.sample_seconds:
            self.get_logger().info("Calibration window finished. Copy the suggested parameters into config/follow_controller.yaml")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowDistanceCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
