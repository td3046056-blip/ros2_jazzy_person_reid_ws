from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _finite_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return default
    return out if math.isfinite(out) else default


@dataclass
class TargetState:
    seen_time: float = 0.0
    payload: Optional[Dict[str, Any]] = None


class PersonFollowController(Node):
    """Convert /person_reid/target identity output into safe /cmd_vel.

    The node intentionally prefers stopping over following an uncertain target.
    It should be used after the identity node has enrolled person A.
    """

    def __init__(self) -> None:
        super().__init__("person_follow_controller")
        self._declare_parameters()
        self._load_parameters()

        self.target = TargetState()
        self.last_sonar_time = 0.0
        self.last_sonar_raw: List[float] = []
        self.enabled = bool(self.start_enabled)
        self._was_enabled = bool(self.start_enabled)
        self.last_cmd = Twist()
        self.last_status_log = 0.0
        self.last_status_publish = 0.0

        # Robust follow state. These make the robot tolerant to brief YOLO/DeepSORT
        # dropouts and noisy bbox height changes. Without this, target_found can
        # blink false/true and the robot will move-stop-move repeatedly.
        self.last_valid_payload: Optional[Dict[str, Any]] = None
        self.last_valid_time = 0.0
        self.valid_frame_streak = 0
        self.filtered_distance_m: Optional[float] = None
        self.filtered_distance_source = "none"
        self.filtered_angle_deg: Optional[float] = None
        self.moving_forward = False

        self.target_sub = self.create_subscription(String, self.target_topic, self._target_cb, 10)
        self.sonar_sub = self.create_subscription(Float32MultiArray, self.sonar_topic, self._sonar_cb, 10)
        self.command_sub = self.create_subscription(String, self.command_topic, self._command_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.enable_srv = self.create_service(Trigger, self.enable_service, self._enable_cb)
        self.disable_srv = self.create_service(Trigger, self.disable_service, self._disable_cb)
        self.stop_srv = self.create_service(Trigger, self.stop_service, self._stop_cb)

        self.timer = self.create_timer(1.0 / max(1.0, self.control_hz), self._control_timer)

        self.get_logger().info(
            "Person follow controller ready: cmd_vel_topic=%s, target_distance=%.2fm, enabled=%s"
            % (self.cmd_vel_topic, self.target_distance_m, self.enabled)
        )
        self.get_logger().info(
            "Distance source=%s, sonar_scale_to_m=%.5f, bbox_height_at_%.2fm=%.1fpx, forward_only=%s"
            % (
                self.distance_source,
                self.sonar_scale_to_m,
                self.target_distance_m,
                self.bbox_height_at_target_distance_px,
                bool(getattr(self, "forward_only", True)),
            )
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "target_topic": "/person_reid/target",
            "sonar_topic": "/bw_dr03/sonar",
            "cmd_vel_topic": "/cmd_vel",
            "status_topic": "/person_follow/status",
            "command_topic": "/person_follow/command",
            "enable_service": "/person_follow/enable",
            "disable_service": "/person_follow/disable",
            "stop_service": "/person_follow/stop",
            "start_enabled": False,
            "require_identity_ready": True,
            "require_verified": False,
            "allow_recovered_target": True,
            "target_timeout_sec": 0.45,
            "lost_stop_after_sec": 0.25,
            "target_hold_sec": 0.90,
            "target_hold_drive_sec": 0.25,
            "hold_speed_scale": 0.35,
            "require_valid_frames": 2,
            "control_hz": 12.0,
            "target_distance_m": 1.0,
            "distance_deadband_m": 0.10,
            "distance_source": "auto",  # auto, sonar, bbox
            "sonar_scale_to_m": 0.01,  # BW distance often appears in cm; set 1.0 if echo is already meters
            "sonar_valid_min_m": 0.15,
            "sonar_valid_max_m": 3.5,
            "sonar_timeout_sec": 0.35,
            "sonar_angle_gate_deg": 18.0,
            "emergency_stop_m": 0.45,
            "camera_distance_enabled": True,
            "bbox_height_at_target_distance_px": 250.0,
            "bbox_distance_valid_min_m": 0.35,
            "bbox_distance_valid_max_m": 5.0,
            "distance_filter_alpha": 0.18,
            "angle_filter_alpha": 0.22,
            "max_distance_step_m": 0.22,
            "max_angle_step_deg": 8.0,
            "start_forward_error_m": 0.22,
            "stop_forward_error_m": 0.08,
            "cmd_epsilon_linear": 0.012,
            "cmd_epsilon_angular": 0.035,
            "kp_linear": 0.45,
            "kp_angular": 1.65,
            "angular_sign": -1.0,
            "max_linear_forward": 0.22,
            "max_linear_backward": 0.0,
            "forward_only": True,
            "max_angular": 0.85,
            "min_forward_speed": 0.035,
            "max_linear_accel": 0.25,
            "max_angular_accel": 1.20,
            "angle_deadband_deg": 3.0,
            "align_only_angle_deg": 26.0,
            "slow_down_angle_deg": 14.0,
            "search_when_lost": False,
            "search_angular": 0.25,
            "publish_stop_when_disabled": True,
            "log_period_sec": 0.8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        names = [p.name for p in self._parameters.values()]
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        self.target_topic = str(self.target_topic)
        self.sonar_topic = str(self.sonar_topic)
        self.cmd_vel_topic = str(self.cmd_vel_topic)
        self.status_topic = str(self.status_topic)
        self.command_topic = str(self.command_topic)
        self.enable_service = str(self.enable_service)
        self.disable_service = str(self.disable_service)
        self.stop_service = str(self.stop_service)
        self.distance_source = str(self.distance_source).lower().strip()
        if self.distance_source not in {"auto", "sonar", "bbox"}:
            self.get_logger().warning("Invalid distance_source=%s; using auto" % self.distance_source)
            self.distance_source = "auto"

    def _target_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning("Invalid target JSON: %s" % exc)
            return
        self.target.payload = payload
        self.target.seen_time = time.time()

    def _sonar_cb(self, msg: Float32MultiArray) -> None:
        vals: List[float] = []
        for x in msg.data:
            v = _finite_float(x)
            if v is not None and v > 0.0:
                vals.append(v)
        if vals:
            self.last_sonar_raw = vals
            self.last_sonar_time = time.time()

    def _command_cb(self, msg: String) -> None:
        cmd = str(msg.data).strip().lower()
        if cmd in {"enable", "on", "start", "follow"}:
            self.enabled = True
            self.get_logger().info("Follow controller ENABLED by command topic")
        elif cmd in {"disable", "off", "stop", "safe"}:
            self.enabled = False
            self._publish_stop(force=True)
            self.get_logger().info("Follow controller DISABLED by command topic")

    def _enable_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.enabled = True
        response.success = True
        response.message = "Person follow enabled"
        self.get_logger().info(response.message)
        return response

    def _disable_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.enabled = False
        self._publish_stop(force=True)
        response.success = True
        response.message = "Person follow disabled and robot stopped"
        self.get_logger().info(response.message)
        return response

    def _stop_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.enabled = False
        self._publish_stop(force=True)
        response.success = True
        response.message = "Emergency stop sent; follow disabled"
        self.get_logger().warn(response.message)
        return response

    def _valid_target_payload(self, now: float) -> Tuple[bool, str, Dict[str, Any]]:
        payload = self.target.payload or {}
        age = now - self.target.seen_time
        if not payload or age > float(self.target_timeout_sec):
            return False, "target_msg_timeout", payload
        if bool(self.require_identity_ready) and not bool(payload.get("identity_ready", False)):
            return False, "identity_not_ready", payload
        if not bool(payload.get("target_found", False)):
            return False, str(payload.get("status", "target_not_found")), payload
        if bool(self.require_verified) and not bool(payload.get("verified", False)):
            # Some strict identity states are recovered but not marked verified every frame.
            if not (bool(self.allow_recovered_target) and bool(payload.get("recovered", False))):
                return False, "target_not_verified", payload
        bbox = payload.get("bbox")
        if not isinstance(bbox, list) or len(bbox) != 4:
            return False, "no_bbox", payload
        return True, "ok", payload

    def _sonar_distance_m(self, now: float) -> Optional[float]:
        if now - self.last_sonar_time > float(self.sonar_timeout_sec):
            return None
        scaled = []
        for raw in self.last_sonar_raw:
            d = raw * float(self.sonar_scale_to_m)
            if float(self.sonar_valid_min_m) <= d <= float(self.sonar_valid_max_m):
                scaled.append(d)
        if not scaled:
            return None
        # Use the closest front reading for safety.
        return min(scaled)

    def _bbox_height_px(self, payload: Dict[str, Any]) -> Optional[float]:
        bbox = payload.get("bbox")
        if not isinstance(bbox, list) or len(bbox) != 4:
            return None
        y1 = _finite_float(bbox[1])
        y2 = _finite_float(bbox[3])
        if y1 is None or y2 is None:
            return None
        h = abs(y2 - y1)
        return h if h > 1.0 else None

    def _bbox_distance_m(self, payload: Dict[str, Any]) -> Optional[float]:
        if not bool(self.camera_distance_enabled):
            return None
        h = self._bbox_height_px(payload)
        if h is None:
            return None
        ref = float(self.bbox_height_at_target_distance_px)
        if ref <= 1.0:
            return None
        d = float(self.target_distance_m) * ref / h
        if float(self.bbox_distance_valid_min_m) <= d <= float(self.bbox_distance_valid_max_m):
            return d
        return None

    def _choose_distance(self, payload: Dict[str, Any], angle_deg: float, now: float) -> Tuple[Optional[float], str]:
        sonar = self._sonar_distance_m(now)
        bbox = self._bbox_distance_m(payload)

        if self.distance_source == "sonar":
            return sonar, "sonar" if sonar is not None else "none"
        if self.distance_source == "bbox":
            return bbox, "bbox" if bbox is not None else "none"

        # auto: sonar is only trusted when the target is reasonably centered.
        if sonar is not None and abs(angle_deg) <= float(self.sonar_angle_gate_deg):
            return sonar, "sonar"
        if bbox is not None:
            return bbox, "bbox"
        if sonar is not None:
            return sonar, "sonar_off_axis"
        return None, "none"

    def _smooth_value(self, previous: Optional[float], raw: Optional[float], alpha: float, max_step: float) -> Optional[float]:
        if raw is None:
            return previous
        raw = float(raw)
        if previous is None or not math.isfinite(float(previous)):
            return raw
        prev = float(previous)
        step = abs(float(max_step))
        if step > 0.0:
            delta = raw - prev
            if abs(delta) > step:
                raw = prev + math.copysign(step, delta)
        a = _clamp(float(alpha), 0.01, 1.0)
        return prev + a * (raw - prev)

    def _reset_smoothers_if_new_track(self, payload: Dict[str, Any]) -> None:
        tid = payload.get("target_track_id")
        last_tid = getattr(self, "_last_filter_track_id", None)
        if tid is not None and last_tid is not None and tid != last_tid:
            self.filtered_distance_m = None
            self.filtered_distance_source = "none"
            self.filtered_angle_deg = None
            self.moving_forward = False
        if tid is not None:
            self._last_filter_track_id = tid

    def _make_twist(self, linear_x: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        return msg

    def _rate_limit(self, desired: Twist, dt: float) -> Twist:
        dt = max(1e-3, dt)
        lin_step = float(self.max_linear_accel) * dt
        ang_step = float(self.max_angular_accel) * dt
        out = Twist()
        out.linear.x = self.last_cmd.linear.x + _clamp(desired.linear.x - self.last_cmd.linear.x, -lin_step, lin_step)
        out.angular.z = self.last_cmd.angular.z + _clamp(desired.angular.z - self.last_cmd.angular.z, -ang_step, ang_step)
        return out

    def _publish_stop(self, force: bool = False) -> None:
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.last_cmd = stop
        if force:
            # Send a few stops to quickly refresh motor watchdog/muxes.
            for _ in range(2):
                self.cmd_pub.publish(stop)

    def _publish_status(self, status: Dict[str, Any], now: float) -> None:
        if now - self.last_status_publish < 0.1:
            return
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.last_status_publish = now

    def _control_timer(self) -> None:
        now = time.time()
        dt = 1.0 / max(1.0, float(self.control_hz))

        raw_valid, raw_reason, raw_payload = self._valid_target_payload(now)
        held_target = False
        hold_age = 999.0

        if raw_valid:
            self.valid_frame_streak += 1
            self.last_valid_payload = raw_payload
            self.last_valid_time = now
            payload = raw_payload
            reason = raw_reason
            valid = self.valid_frame_streak >= max(1, int(self.require_valid_frames))
            if not valid:
                reason = "warming_valid_target"
        else:
            self.valid_frame_streak = 0
            hold_age = now - self.last_valid_time
            if self.last_valid_payload is not None and hold_age <= float(self.target_hold_sec):
                # Brief detector/tracker blink: keep the last known target instead of
                # immediately sending stop/start/stop commands.
                payload = self.last_valid_payload
                valid = True
                held_target = True
                reason = "target_hold_%s" % raw_reason
            else:
                payload = raw_payload
                valid = False
                reason = raw_reason
                self.filtered_distance_m = None
                self.filtered_distance_source = "none"
                self.filtered_angle_deg = None
                self.moving_forward = False

        self._reset_smoothers_if_new_track(payload or {})

        raw_angle_deg = _finite_float(payload.get("camera_angle_deg"), 0.0) or 0.0
        self.filtered_angle_deg = self._smooth_value(
            self.filtered_angle_deg,
            raw_angle_deg,
            float(self.angle_filter_alpha),
            float(self.max_angle_step_deg),
        )
        angle_deg = self.filtered_angle_deg if self.filtered_angle_deg is not None else raw_angle_deg

        raw_distance_m, distance_source = self._choose_distance(payload, raw_angle_deg, now) if payload else (None, "none")
        if raw_distance_m is not None:
            if distance_source != self.filtered_distance_source:
                self.filtered_distance_m = float(raw_distance_m)
                self.filtered_distance_source = distance_source
            else:
                self.filtered_distance_m = self._smooth_value(
                    self.filtered_distance_m,
                    raw_distance_m,
                    float(self.distance_filter_alpha),
                    float(self.max_distance_step_m),
                )
        distance_m = self.filtered_distance_m
        sonar_m = self._sonar_distance_m(now)
        bbox_h = self._bbox_height_px(payload) if payload else None

        status: Dict[str, Any] = {
            "enabled": bool(self.enabled),
            "target_valid": bool(valid),
            "held_target": bool(held_target),
            "hold_age_sec": None if not held_target else round(float(hold_age), 3),
            "valid_frame_streak": int(self.valid_frame_streak),
            "reason": reason,
            "cmd_vel_topic": self.cmd_vel_topic,
            "target_distance_m": round(float(self.target_distance_m), 3),
            "distance_m": None if distance_m is None else round(float(distance_m), 3),
            "raw_distance_m": None if raw_distance_m is None else round(float(raw_distance_m), 3),
            "distance_source": distance_source,
            "sonar_m": None if sonar_m is None else round(float(sonar_m), 3),
            "bbox_height_px": None if bbox_h is None else round(float(bbox_h), 1),
            "angle_deg": round(float(angle_deg), 3),
            "raw_angle_deg": round(float(raw_angle_deg), 3),
            "target_status": payload.get("status") if payload else None,
            "target_track_id": payload.get("target_track_id") if payload else None,
            "identity_ready": bool(payload.get("identity_ready", False)) if payload else False,
        }

        if not self.enabled:
            # v1.9: publish the stop only on the enabled->disabled edge, not every
            # tick. lidar_follow_enhancer also publishes to cmd_vel_topic while it
            # is disabling us to run an avoidance maneuver; if we keep publishing
            # Twist(0) here at control_hz on top of that, the two publishers race
            # on the same topic and the avoidance motion comes out jerky/stuttering.
            if bool(self.publish_stop_when_disabled) and self._was_enabled:
                self._publish_stop(force=True)
            status.update({"cmd_linear_x": 0.0, "cmd_angular_z": 0.0, "controller_state": "disabled"})
            self._publish_status(status, now)
            self._was_enabled = False
            return
        self._was_enabled = True

        if sonar_m is not None and sonar_m < float(self.emergency_stop_m):
            self._publish_stop(force=True)
            status.update({"cmd_linear_x": 0.0, "cmd_angular_z": 0.0, "controller_state": "emergency_stop_sonar"})
            self._publish_status(status, now)
            if now - self.last_status_log > float(self.log_period_sec):
                self.get_logger().warn("Emergency stop: sonar %.2fm < %.2fm" % (sonar_m, self.emergency_stop_m))
                self.last_status_log = now
            return

        if not valid:
            if bool(self.search_when_lost) and reason not in {"identity_not_ready"}:
                desired = self._make_twist(0.0, float(self.search_angular))
                cmd = self._rate_limit(desired, dt)
                self.cmd_pub.publish(cmd)
                self.last_cmd = cmd
                state = "searching_lost_target"
            else:
                self._publish_stop()
                cmd = Twist()
                state = "stopped_no_valid_target"
            status.update({"cmd_linear_x": round(cmd.linear.x, 4), "cmd_angular_z": round(cmd.angular.z, 4), "controller_state": state})
            self._publish_status(status, now)
            return

        # Angular control. Positive camera angle means target is on image right;
        # ROS positive angular.z is left, so default sign is -1.0.
        angle_rad = math.radians(angle_deg)
        if abs(angle_deg) < float(self.angle_deadband_deg):
            angular_z = 0.0
        else:
            angular_z = float(self.angular_sign) * float(self.kp_angular) * angle_rad
        angular_z = _clamp(angular_z, -float(self.max_angular), float(self.max_angular))

        linear_x = 0.0
        controller_state = "tracking"
        if distance_m is None:
            # Without distance, only rotate to center the target; never drive forward blindly.
            linear_x = 0.0
            controller_state = "tracking_angle_only_no_distance"
        else:
            error = float(distance_m) - float(self.target_distance_m)
            if bool(getattr(self, "forward_only", True)):
                # Hysteresis: do not start creeping forward for tiny distance noise.
                # Start moving only when the person is clearly farther than the target,
                # and stop only after reaching the tighter stop threshold.
                if self.moving_forward:
                    if error <= float(self.stop_forward_error_m):
                        self.moving_forward = False
                        linear_x = 0.0
                        controller_state = "hold_distance"
                    else:
                        linear_x = float(self.kp_linear) * error
                else:
                    if error <= float(self.start_forward_error_m):
                        linear_x = 0.0
                        controller_state = "hold_distance" if error >= 0.0 else "too_close_stop_forward_only"
                    else:
                        self.moving_forward = True
                        linear_x = float(self.kp_linear) * error
                linear_x = _clamp(linear_x, 0.0, float(self.max_linear_forward))
            else:
                if abs(error) <= float(self.distance_deadband_m):
                    linear_x = 0.0
                    controller_state = "hold_distance"
                else:
                    linear_x = float(self.kp_linear) * error
                    linear_x = _clamp(linear_x, -float(self.max_linear_backward), float(self.max_linear_forward))

            if 0.0 < linear_x < float(self.min_forward_speed):
                linear_x = float(self.min_forward_speed)
            elif -float(self.min_forward_speed) < linear_x < 0.0:
                linear_x = -float(self.min_forward_speed)

            if held_target:
                if hold_age <= float(self.target_hold_drive_sec):
                    linear_x *= float(self.hold_speed_scale)
                    angular_z *= float(self.hold_speed_scale)
                    controller_state = "tracking_hold_short"
                else:
                    linear_x = 0.0
                    angular_z *= float(self.hold_speed_scale)
                    controller_state = "tracking_hold_angle_only"

            if abs(angle_deg) >= float(self.align_only_angle_deg):
                linear_x = 0.0
                controller_state = "align_only"
            elif abs(angle_deg) >= float(self.slow_down_angle_deg):
                # Smoothly reduce linear speed when target is off-center.
                over = abs(angle_deg) - float(self.slow_down_angle_deg)
                span = max(1.0, float(self.align_only_angle_deg) - float(self.slow_down_angle_deg))
                scale = _clamp(1.0 - over / span, 0.0, 1.0)
                linear_x *= scale
                controller_state = "tracking_slow_turn"

        if abs(linear_x) < float(self.cmd_epsilon_linear):
            linear_x = 0.0
        if abs(angular_z) < float(self.cmd_epsilon_angular):
            angular_z = 0.0

        desired = self._make_twist(linear_x, angular_z)
        cmd = self._rate_limit(desired, dt)
        # Important: do NOT zero small rate-limited ramp values while the desired
        # command is non-zero. If max_linear_accel/control_hz is smaller than
        # cmd_epsilon_linear, zeroing here makes the robot permanently stuck at 0.
        # Only apply the dead-zone when the desired command itself is essentially zero.
        if abs(desired.linear.x) < float(self.cmd_epsilon_linear) and abs(cmd.linear.x) < float(self.cmd_epsilon_linear):
            cmd.linear.x = 0.0
        if abs(desired.angular.z) < float(self.cmd_epsilon_angular) and abs(cmd.angular.z) < float(self.cmd_epsilon_angular):
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

        status.update({
            "cmd_linear_x": round(float(cmd.linear.x), 4),
            "cmd_angular_z": round(float(cmd.angular.z), 4),
            "controller_state": controller_state,
            "distance_error_m": None if distance_m is None else round(float(distance_m) - float(self.target_distance_m), 3),
        })
        self._publish_status(status, now)

        if now - self.last_status_log > float(self.log_period_sec):
            self.get_logger().info(
                "state=%s id=%s dist=%s(%s) angle=%.1f cmd=(%.3f, %.3f) reason=%s"
                % (
                    controller_state,
                    status.get("target_track_id"),
                    "None" if distance_m is None else "%.2f" % distance_m,
                    distance_source,
                    angle_deg,
                    cmd.linear.x,
                    cmd.angular.z,
                    reason,
                )
            )
            self.last_status_log = now

    def destroy_node(self) -> bool:
        try:
            self._publish_stop(force=True)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonFollowController()
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
