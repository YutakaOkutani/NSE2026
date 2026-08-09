import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    CAMERA_FRAME_STALE_STOP_SEC,
    CAMERA_HORIZONTAL_FOV_DEG,
    CAMERA_WEAK_CONFIRM_FRAMES,
    CAMERA_WEAK_MAX_MISSED_FRAMES,
    CAMERA_WEAK_MIN_CANDIDATE_PROBABILITY,
    CAMERA_WEAK_MIN_HUE_SCORE,
    CAMERA_WEAK_MIN_SHAPE_SCORE,
    CAMERA_WEAK_MIN_SV_SCORE,
    CAMERA_WEAK_RELAXED_SV_SCORE,
    CAMERA_WEAK_STRONG_HUE_SCORE,
    CONE_CENTER_POSITION,
    CONE_PHASE4_BBOX_CENTER_Y_TOLERANCE,
    CONE_PHASE4_BBOX_SIZE_RATIO_MAX,
    CONE_PHASE4_BBOX_VERTICAL_OVERLAP_MIN,
    CONE_PHASE4_BEARING_CONSISTENCY_TOLERANCE_DEG,
    CONE_PHASE4_CENTER_TOLERANCE,
    CONE_PHASE4_CONFIRM_FRAMES,
    CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE,
    CONE_PHASE4_REACHED_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_PHASE45_MAX_DISTANCE,
    Phase,
    TIMEOUT_PHASE_4,
)
from mission.nav import calc_distance_and_azimuth
from mission.phases.base import BasePhaseHandler


class Phase4Handler(BasePhaseHandler):
    @staticmethod
    def _float_value(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _fresh_camera_frame(self, snapshot, now):
        sequence = int(self._float_value(snapshot.get("cone_sequence", 0), 0.0))
        updated_at = self._float_value(snapshot.get("cone_updated_at", 0.0), 0.0)
        age = float("inf") if updated_at <= 0.0 else max(0.0, now - updated_at)
        return (
            bool(snapshot.get("cone_valid", False))
            and sequence > 0
            and age <= float(CAMERA_FRAME_STALE_STOP_SEC)
        ), sequence, age

    def _weak_candidate(self, snapshot, cone_dir):
        debug = dict(snapshot.get("cone_debug", {}) or {})
        candidate_prob = self._float_value(debug.get("candidate_probability", 0.0))
        shape = self._float_value(debug.get("cone_shape_score", 0.0))
        hue = self._float_value(debug.get("hue_redness_score", 0.0))
        sv = self._float_value(debug.get("sv_score", 0.0))
        ground_penalty = self._float_value(debug.get("ground_penalty", 1.0), 1.0)
        bbox_height = self._float_value(debug.get("bbox_height", 0.0))
        bbox_bottom = self._float_value(debug.get("bbox_bottom_frac", 0.0))
        penalty_flags = str(debug.get("penalty_flags", ""))
        centered = abs(cone_dir - CONE_CENTER_POSITION) <= CONE_PHASE4_CENTER_TOLERANCE
        color_quality = (
            hue >= CAMERA_WEAK_MIN_HUE_SCORE and sv >= CAMERA_WEAK_MIN_SV_SCORE
        ) or (
            hue >= CAMERA_WEAK_STRONG_HUE_SCORE
            and sv >= CAMERA_WEAK_RELAXED_SV_SCORE
        )
        return bool(
            centered
            and candidate_prob >= CAMERA_WEAK_MIN_CANDIDATE_PROBABILITY
            and shape >= CAMERA_WEAK_MIN_SHAPE_SCORE
            and color_quality
            and ground_penalty >= 0.5
            and "upper_sky" not in penalty_flags
            and (bbox_height <= 0.0 or bbox_bottom >= 0.35)
        )

    @staticmethod
    def _angle_diff_deg(target, current):
        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

    def _candidate_signature(self, snapshot, cone_dir):
        debug = dict(snapshot.get("cone_debug", {}) or {})
        heading = None
        bearing = None
        if bool(snapshot.get("angle_valid", False)):
            try:
                heading = float(snapshot.get("angle")) % 360.0
                bearing = (
                    heading
                    + (float(cone_dir) - CONE_CENTER_POSITION)
                    * float(CAMERA_HORIZONTAL_FOV_DEG)
                ) % 360.0
            except (TypeError, ValueError):
                heading = None
                bearing = None

        bbox_x = self._float_value(debug.get("bbox_x", 0.0))
        bbox_y = self._float_value(debug.get("bbox_y", 0.0))
        bbox_w = self._float_value(debug.get("bbox_width", 0.0))
        bbox_h = self._float_value(debug.get("bbox_height", 0.0))
        reported_height_frac = self._float_value(
            debug.get("bbox_height_frac", 0.0)
        )
        reported_bottom_frac = self._float_value(
            debug.get("bbox_bottom_frac", 0.0)
        )
        bbox_valid = (
            bbox_w > 0.0
            and bbox_h > 0.0
            and (reported_height_frac > 0.0 or reported_bottom_frac > 0.0)
        )
        if bbox_valid:
            if reported_height_frac > 0.0:
                frame_height = bbox_h / reported_height_frac
            else:
                frame_height = (bbox_y + bbox_h) / reported_bottom_frac
            center_y = (bbox_y + 0.5 * bbox_h) / max(frame_height, 1.0)
            height_frac = bbox_h / max(frame_height, 1.0)
            bottom_frac = (bbox_y + bbox_h) / max(frame_height, 1.0)
        else:
            center_y = 0.0
            height_frac = 0.0
            bottom_frac = 0.0

        return {
            "direction": float(cone_dir),
            "heading": heading,
            "bearing": bearing,
            "bbox_valid": bbox_valid,
            "bbox_x": bbox_x,
            "bbox_y": bbox_y,
            "bbox_width": bbox_w,
            "bbox_height": bbox_h,
            "center_y": center_y,
            "height_frac": height_frac,
            "bottom_frac": bottom_frac,
        }

    def _candidate_consistency(self, previous, current):
        if not previous:
            return True, "new"

        previous_bearing = previous.get("bearing")
        current_bearing = current.get("bearing")
        if previous_bearing is not None and current_bearing is not None:
            bearing_error = abs(
                self._angle_diff_deg(current_bearing, previous_bearing)
            )
            if bearing_error > float(
                CONE_PHASE4_BEARING_CONSISTENCY_TOLERANCE_DEG
            ):
                return False, "bearing"
        elif abs(
            float(current["direction"]) - float(previous.get("direction", 0.5))
        ) > float(CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE):
            return False, "direction"

        if bool(previous.get("bbox_valid")) and bool(current.get("bbox_valid")):
            if abs(
                float(current["center_y"]) - float(previous.get("center_y", 0.0))
            ) > float(CONE_PHASE4_BBOX_CENTER_Y_TOLERANCE):
                return False, "bbox_y"

            previous_h = max(float(previous.get("bbox_height", 0.0)), 1.0)
            current_h = max(float(current.get("bbox_height", 0.0)), 1.0)
            previous_w = max(float(previous.get("bbox_width", 0.0)), 1.0)
            current_w = max(float(current.get("bbox_width", 0.0)), 1.0)
            size_ratio = max(
                previous_h / current_h,
                current_h / previous_h,
                previous_w / current_w,
                current_w / previous_w,
            )
            if size_ratio > float(CONE_PHASE4_BBOX_SIZE_RATIO_MAX):
                return False, "bbox_size"

            previous_top = float(previous.get("center_y", 0.0)) - 0.5 * float(
                previous.get("height_frac", 0.0)
            )
            previous_bottom = float(previous.get("bottom_frac", 0.0))
            current_top = float(current.get("center_y", 0.0)) - 0.5 * float(
                current.get("height_frac", 0.0)
            )
            current_bottom = float(current.get("bottom_frac", 0.0))
            overlap = max(
                0.0,
                min(previous_bottom, current_bottom)
                - max(previous_top, current_top),
            )
            union = max(previous_bottom, current_bottom) - min(
                previous_top,
                current_top,
            )
            vertical_overlap = overlap / max(union, 1e-6)
            if vertical_overlap < float(CONE_PHASE4_BBOX_VERTICAL_OVERLAP_MIN):
                return False, "bbox_overlap"

        return True, "consistent"

    @staticmethod
    def _reset_strong_confirmation(controller):
        controller.phase4_detect_confirm_count = 0
        controller.phase4_detect_confirm_marker = None
        controller.phase4_detect_track_signature = None

    @staticmethod
    def _reset_weak_confirmation(controller):
        controller.phase4_weak_confirm_count = 0
        controller.phase4_weak_confirm_marker = None
        controller.phase4_weak_missed_frames = 0
        controller.phase4_weak_track_signature = None

    def _fallback_to_p3(self, controller, current_snapshot, reason):
        fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
        print(reason)
        controller.cone_phase_decision = "p4_fallback_to_p3"
        controller.st.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
        controller.searching_flag = False
        self._reset_strong_confirmation(controller)
        self._reset_weak_confirmation(controller)
        controller.time_phase3_start = time.time()

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        entry_marker = getattr(controller, "phase_entry_time", None)
        if (
            entry_marker is not None
            and getattr(controller, "phase4_camera_recovery_marker", None) != entry_marker
        ):
            controller.phase4_camera_recovery_marker = entry_marker
            controller.phase4_last_processed_cone_seq = 0
            self._reset_strong_confirmation(controller)
            self._reset_weak_confirmation(controller)
            recovery_started = getattr(controller, "camera_recovery_started_at", None)
            # Preserve a campaign started by the camera thread just after this
            # P4 entry, but discard exhausted/stale state inherited from P3/P5.
            if recovery_started is None or float(recovery_started) < float(entry_marker):
                controller.reset_camera_recovery_window()
        current_snapshot = controller.st.snapshot()
        cone_prob = current_snapshot["cone_probability"]
        cone_dir = current_snapshot.get("cone_direction", CONE_CENTER_POSITION)
        cone_reached = current_snapshot.get("cone_is_reached", False)
        now = time.time()
        camera_fresh, cone_sequence, _camera_age = self._fresh_camera_frame(current_snapshot, now)
        cone_reached_effective = camera_fresh and bool(cone_reached) and (
            cone_prob
            > max(
                CONE_PROBABILITY_THRESHOLD_PHASE4,
                CONE_PHASE4_REACHED_PROBABILITY_THRESHOLD,
            )
        )
        controller.cone_phase_decision = "p4_precheck"
        controller.cone_phase_threshold = float(CONE_PROBABILITY_THRESHOLD_PHASE4)
        controller.cone_phase_reached_probability_threshold = float(
            max(
                CONE_PROBABILITY_THRESHOLD_PHASE4,
                CONE_PHASE4_REACHED_PROBABILITY_THRESHOLD,
            )
        )
        controller.cone_phase_center_tolerance = float(CONE_PHASE4_CENTER_TOLERANCE)
        controller.cone_phase_direction_tolerance = float(
            CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE
        )
        controller.cone_phase_required_confirm_frames = int(CONE_PHASE4_CONFIRM_FRAMES)
        controller.cone_phase_detected = False
        controller.cone_phase_reached_effective = bool(cone_reached_effective)
        controller.cone_phase_centered = False
        controller.cone_phase_direction_consistent = False
        controller.cone_phase_confirm_count = int(getattr(controller, "phase4_detect_confirm_count", 0))
        print("p4 : camera searching")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if (
            hasattr(controller, "target_lat")
            and hasattr(controller, "target_lng")
            and current_snapshot.get("gps_detect") == GPS_ACTIVE_DETECT
        ):
            dist_m, azimuth = calc_distance_and_azimuth(
                current_snapshot["lat"],
                current_snapshot["lng"],
                controller.target_lat,
                controller.target_lng,
            )
            controller.st.update_navigation(distance=dist_m, azimuth=azimuth)
            if dist_m > GPS_PHASE45_MAX_DISTANCE and not getattr(controller, "phase3_arrived_latched", False):
                self._fallback_to_p3(
                    controller,
                    current_snapshot,
                    f"Phase4 GPS fallback: target is {dist_m:.1f}m away (> {GPS_PHASE45_MAX_DISTANCE:.1f}m)",
                )
                return
        if not controller.searching_flag:
            controller.searching_flag = True
            controller.time_start_searching_cone = now
            controller.camera_phase4_attempts += 1
            controller.camera_phase4_start = controller.time_start_searching_cone
        else:
            if now - controller.time_start_searching_cone >= TIMEOUT_PHASE_4:
                print("Phase4 TIMEOUT: stopping camera phases and giving up")
                controller.cone_phase_decision = "p4_timeout_to_p7_give_up"
                controller.searching_flag = False
                controller.transition_to_give_up("PHASE4_TIMEOUT_GIVE_UP")
                return
        if bool(getattr(controller, "camera_recovery_exhausted", False)):
            attempts = int(getattr(controller, "camera_reinit_attempt_count", 0))
            print(f"Camera DEAD after {attempts} reinit attempts: giving up")
            controller.cone_phase_decision = "p4_camera_dead_to_p7_give_up"
            controller.searching_flag = False
            controller.transition_to_give_up("PHASE4_CAMERA_DEAD_GIVE_UP")
            return
        if not camera_fresh:
            controller.cone_phase_decision = (
                "p4_wait_first_camera_frame"
                if cone_sequence <= 0
                else "p4_camera_stale_wait"
            )
            return
        if cone_sequence == int(getattr(controller, "phase4_last_processed_cone_seq", 0)):
            if int(getattr(controller, "phase4_weak_confirm_count", 0)) > 0:
                controller.cone_phase_required_confirm_frames = int(
                    CAMERA_WEAK_CONFIRM_FRAMES
                )
            controller.cone_phase_decision = "p4_wait_new_frame"
            return
        controller.phase4_last_processed_cone_seq = cone_sequence
        # Phase4開始時は比較的近距離(数m)のため、誤検知低減のために
        # Phase5より厳しめのprobability + 数フレーム連続確認を要求する。
        try:
            cone_dir_val = float(cone_dir)
        except (TypeError, ValueError):
            cone_dir_val = CONE_CENTER_POSITION
        centered = abs(cone_dir_val - CONE_CENTER_POSITION) <= CONE_PHASE4_CENTER_TOLERANCE
        cone_debug = dict(current_snapshot.get("cone_debug", {}) or {})
        strict_red_ok = bool(int(self._float_value(cone_debug.get("strict_red_ok", 0))))
        strict_detect = (
            cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE4
            and centered
            and strict_red_ok
        )
        loose_detect = cone_prob > CONE_PROBABILITY_THRESHOLD
        weak_detect = self._weak_candidate(current_snapshot, cone_dir_val)
        signature = self._candidate_signature(current_snapshot, cone_dir_val)
        dir_marker = getattr(controller, "phase4_detect_confirm_marker", None)
        dir_consistent, strong_consistency_reason = self._candidate_consistency(
            getattr(controller, "phase4_detect_track_signature", None),
            signature,
        )

        controller.cone_phase_detected = bool(strict_detect or weak_detect)
        controller.cone_phase_centered = bool(centered)
        controller.cone_phase_direction_consistent = bool(strict_detect and dir_consistent)

        if cone_reached_effective or strict_detect:
            if cone_reached_effective:
                controller.cone_phase_decision = "p4_reached_confirm"
                controller.phase4_detect_confirm_count = getattr(controller, "phase4_detect_confirm_count", 0) + 1
                controller.phase4_detect_confirm_marker = cone_dir_val
            elif dir_consistent:
                controller.cone_phase_decision = "p4_strict_confirm"
                controller.phase4_detect_confirm_count = getattr(controller, "phase4_detect_confirm_count", 0) + 1
                controller.phase4_detect_track_signature = signature
                if dir_marker is None:
                    controller.phase4_detect_confirm_marker = cone_dir_val
                else:
                    controller.phase4_detect_confirm_marker = 0.7 * float(dir_marker) + 0.3 * cone_dir_val
            else:
                controller.cone_phase_decision = (
                    f"p4_track_reset_{strong_consistency_reason}"
                )
                controller.phase4_detect_confirm_count = 1
                controller.phase4_detect_confirm_marker = cone_dir_val
                controller.phase4_detect_track_signature = signature
        elif loose_detect:
            controller.cone_phase_decision = "p4_low_confidence_reset"
            self._reset_strong_confirmation(controller)
        else:
            controller.cone_phase_decision = "p4_no_detection"
            self._reset_strong_confirmation(controller)

        weak_marker = getattr(controller, "phase4_weak_confirm_marker", None)
        if weak_detect:
            weak_consistent, weak_consistency_reason = self._candidate_consistency(
                getattr(controller, "phase4_weak_track_signature", None),
                signature,
            )
            if weak_consistent:
                controller.phase4_weak_confirm_count = int(
                    getattr(controller, "phase4_weak_confirm_count", 0)
                ) + 1
                controller.phase4_weak_track_signature = signature
                if weak_marker is None:
                    controller.phase4_weak_confirm_marker = cone_dir_val
                else:
                    controller.phase4_weak_confirm_marker = (
                        0.7 * self._float_value(weak_marker, cone_dir_val) + 0.3 * cone_dir_val
                    )
            else:
                controller.phase4_weak_confirm_count = 1
                controller.phase4_weak_confirm_marker = cone_dir_val
                controller.phase4_weak_track_signature = signature
                if not strict_detect:
                    controller.cone_phase_decision = (
                        f"p4_weak_track_reset_{weak_consistency_reason}"
                    )
            controller.phase4_weak_missed_frames = 0
            if not strict_detect and weak_consistent:
                controller.cone_phase_decision = "p4_weak_confirm"
            if not strict_detect:
                controller.cone_phase_direction_consistent = bool(weak_consistent)
        elif int(getattr(controller, "phase4_weak_confirm_count", 0)) > 0:
            controller.phase4_weak_missed_frames = int(
                getattr(controller, "phase4_weak_missed_frames", 0)
            ) + 1
            if controller.phase4_weak_missed_frames > CAMERA_WEAK_MAX_MISSED_FRAMES:
                self._reset_weak_confirmation(controller)

        controller.cone_phase_confirm_count = max(
            int(controller.phase4_detect_confirm_count),
            int(getattr(controller, "phase4_weak_confirm_count", 0)),
        )
        if weak_detect and not strict_detect:
            controller.cone_phase_required_confirm_frames = int(CAMERA_WEAK_CONFIRM_FRAMES)

        # 近距離ではコーンが画面からはみ出してprobが落ちても、到達判定が立てば進める。
        weak_confirmed = (
            int(getattr(controller, "phase4_weak_confirm_count", 0))
            >= CAMERA_WEAK_CONFIRM_FRAMES
        )
        if (
            cone_reached_effective
            or controller.phase4_detect_confirm_count >= CONE_PHASE4_CONFIRM_FRAMES
            or weak_confirmed
        ):
            if cone_reached_effective:
                controller.cone_phase_decision = "p4_reached_to_p5"
                print("Phase4 -> Phase5: close-range visual reached detected")
            elif weak_confirmed and controller.phase4_detect_confirm_count < CONE_PHASE4_CONFIRM_FRAMES:
                controller.cone_phase_decision = "p4_weak_confirmed_to_p5"
                print(
                    "Phase4 -> Phase5: temporally stable weak cone candidate "
                    f"(confirm={controller.phase4_weak_confirm_count})"
                )
            else:
                controller.cone_phase_decision = "p4_confirmed_to_p5"
                print(
                    "Phase4 -> Phase5: cone detected "
                    f"(prob={cone_prob:.2f}, confirm={controller.phase4_detect_confirm_count})"
                )
            controller.searching_flag = False
            self._reset_strong_confirmation(controller)
            self._reset_weak_confirmation(controller)
            controller.phase5_last_processed_cone_seq = 0
            controller.phase5_entry_reason = "phase4_detected"
            controller.st.update_navigation(phase=int(Phase.PHASE5))
            return

        if loose_detect and controller.led_blink_timer % 10 == 0:
            print(
                f"Phase4: low-confidence cone ignored (prob={cone_prob:.2f}, dir={cone_dir_val:.2f})"
            )


def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE4)


if __name__ == "__main__":
    run_standalone()
