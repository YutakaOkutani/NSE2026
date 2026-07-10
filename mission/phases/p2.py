import math
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
    BNO_CALIB_MAG_MIN,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_CLOSE_DISTANCE,
    LED_INTERVAL_PHASE2,
    PHASE2_CALIB_MAX_TIME,
    PHASE2_CALIB_MIN_TIME,
    PHASE2_CALIB_STABLE_SEC,
    PHASE2_ESCAPE_TIME,
    PHASE2_OFFSET_CONTROL_SETTLE_TIME,
    PHASE2_OFFSET_LEG_MAX_TIME,
    PHASE2_OFFSET_MAX_BNO_SPREAD_DEG,
    PHASE2_OFFSET_MAX_SUBSEGMENT_DIFF_DEG,
    PHASE2_OFFSET_MIN_DISTANCE_M,
    PHASE2_OFFSET_MIN_PATH_EFFICIENCY,
    PHASE2_OFFSET_MIN_SAMPLES,
    PHASE2_OFFSET_MODE_COLLECT,
    PHASE2_OFFSET_MODE_TURNAROUND,
    PHASE2_OFFSET_TURN_CONFIRM_COUNT,
    PHASE2_OFFSET_TURN_DEADBAND_DEG,
    PHASE2_STAGE_CALIBRATION,
    PHASE2_STAGE_ESCAPE,
    PHASE2_STAGE_OFFSET,
    PHASE_LOG_INTERVAL,
    PHASE3_ARRIVAL_CONFIRM_COUNT,
    PHASE3_ARRIVAL_CONFIRM_SEC,
    Phase,
)
from mission.nav import calc_distance_and_azimuth
from mission.phases.base import BasePhaseHandler


class Phase2Handler(BasePhaseHandler):
    @staticmethod
    def _normalize_heading(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value % 360.0

    @staticmethod
    def _angle_diff(target, current):
        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

    @classmethod
    def _circular_mean(cls, values):
        values = [cls._normalize_heading(value) for value in values]
        values = [value for value in values if value is not None]
        if not values:
            return None
        sx = sum(math.cos(math.radians(value)) for value in values)
        sy = sum(math.sin(math.radians(value)) for value in values)
        if abs(sx) < 1e-9 and abs(sy) < 1e-9:
            return None
        return math.degrees(math.atan2(sy, sx)) % 360.0

    @staticmethod
    def _centroid(samples):
        count = float(len(samples))
        return (
            sum(sample["lat"] for sample in samples) / count,
            sum(sample["lng"] for sample in samples) / count,
        )

    @classmethod
    def _segment_course(cls, samples):
        if len(samples) < 2:
            return 0.0, None
        edge_count = max(1, min(3, len(samples) // 3))
        start_lat, start_lng = cls._centroid(samples[:edge_count])
        end_lat, end_lng = cls._centroid(samples[-edge_count:])
        return calc_distance_and_azimuth(start_lat, start_lng, end_lat, end_lng)

    @classmethod
    def _estimate_offset_segment(cls, samples):
        result = {
            "valid": False,
            "reason": "insufficient_samples",
            "distance_m": 0.0,
            "path_efficiency": 0.0,
            "course_deg": 0.0,
            "bno_mean_deg": 0.0,
            "bno_spread_deg": 180.0,
            "subsegment_diff_deg": 180.0,
            "offset_deg": 0.0,
        }
        if len(samples) < int(PHASE2_OFFSET_MIN_SAMPLES):
            return result

        distance_m, _ = calc_distance_and_azimuth(
            samples[0]["lat"],
            samples[0]["lng"],
            samples[-1]["lat"],
            samples[-1]["lng"],
        )
        _, course_deg = cls._segment_course(samples)
        path_length_m = 0.0
        for previous, current in zip(samples, samples[1:]):
            step_m, _ = calc_distance_and_azimuth(
                previous["lat"], previous["lng"], current["lat"], current["lng"]
            )
            path_length_m += step_m
        path_efficiency = distance_m / path_length_m if path_length_m > 0.0 else 0.0
        bno_mean = cls._circular_mean(sample["bno_heading"] for sample in samples)
        if course_deg is None or bno_mean is None:
            result["reason"] = "invalid_heading_mean"
            return result
        bno_spread = max(
            abs(cls._angle_diff(sample["bno_heading"], bno_mean)) for sample in samples
        )

        midpoint = len(samples) // 2
        first_half = samples[: midpoint + 1]
        second_half = samples[midpoint:]
        _, first_course = cls._segment_course(first_half)
        _, second_course = cls._segment_course(second_half)
        first_bno = cls._circular_mean(sample["bno_heading"] for sample in first_half)
        second_bno = cls._circular_mean(sample["bno_heading"] for sample in second_half)
        if None in (first_course, second_course, first_bno, second_bno):
            result["reason"] = "invalid_subsegment"
            return result
        first_offset = (first_course - first_bno) % 360.0
        second_offset = (second_course - second_bno) % 360.0
        subsegment_diff = abs(cls._angle_diff(first_offset, second_offset))
        offset_deg = (course_deg - bno_mean) % 360.0

        result.update(
            distance_m=distance_m,
            path_efficiency=path_efficiency,
            course_deg=course_deg,
            bno_mean_deg=bno_mean,
            bno_spread_deg=bno_spread,
            subsegment_diff_deg=subsegment_diff,
            offset_deg=offset_deg,
        )
        if distance_m < float(PHASE2_OFFSET_MIN_DISTANCE_M):
            result["reason"] = "distance_short"
        elif path_efficiency < float(PHASE2_OFFSET_MIN_PATH_EFFICIENCY):
            result["reason"] = "path_curved"
        elif bno_spread > float(PHASE2_OFFSET_MAX_BNO_SPREAD_DEG):
            result["reason"] = "bno_heading_unstable"
        elif subsegment_diff > float(PHASE2_OFFSET_MAX_SUBSEGMENT_DIFF_DEG):
            result["reason"] = "offset_inconsistent"
        else:
            result["valid"] = True
            result["reason"] = ""
        return result

    @classmethod
    def _reset_offset_collection(cls, controller, snapshot, now, reason):
        heading = (
            cls._normalize_heading(snapshot.get("angle"))
            if snapshot.get("angle_valid", False)
            else None
        )
        controller.phase2_offset_mode = PHASE2_OFFSET_MODE_COLLECT
        controller.phase2_offset_turn_target_deg = None
        controller.phase2_offset_turn_confirm_count = 0
        controller.phase2_offset_reference_bno_deg = heading
        controller.phase2_offset_heading_error_deg = 0.0
        controller.phase2_offset_samples = []
        controller.phase2_offset_distance_m = 0.0
        controller.phase2_offset_path_efficiency = 0.0
        controller.phase2_offset_course_deg = 0.0
        controller.phase2_offset_bno_mean_deg = 0.0
        controller.phase2_offset_bno_spread_deg = 0.0
        controller.phase2_offset_subsegment_diff_deg = 0.0
        controller.bno_heading_offset_candidate_deg = None
        controller.bno_heading_offset_candidate_count = 0
        controller.phase2_offset_settle_until = now + float(PHASE2_OFFSET_CONTROL_SETTLE_TIME)
        controller.phase2_offset_leg_start_time = now
        controller.phase2_offset_reject_reason = reason
        try:
            gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
        except (TypeError, ValueError):
            gps_fix_seq = 0
        controller.phase2_offset_last_gps_fix_seq = gps_fix_seq
        controller._heading_offset_last_gps_fix_seq = gps_fix_seq
        controller.phase2_offset_observed_bno_recovery_seq = int(
            getattr(controller, "bno_heading_recovery_seq", 0)
        )

    @classmethod
    def _begin_offset_turnaround(cls, controller, snapshot, reason):
        heading = (
            cls._normalize_heading(snapshot.get("angle"))
            if snapshot.get("angle_valid", False)
            else None
        )
        controller.phase2_offset_mode = PHASE2_OFFSET_MODE_TURNAROUND
        controller.phase2_offset_turn_target_deg = (
            (heading + 180.0) % 360.0 if heading is not None else None
        )
        controller.phase2_offset_turn_confirm_count = 0
        controller.phase2_offset_samples = []
        controller.phase2_offset_distance_m = 0.0
        controller.bno_heading_offset_candidate_count = 0
        controller.phase2_offset_reject_reason = f"turnaround:{reason}"

    @classmethod
    def _update_phase2_goal(cls, controller, snapshot, now):
        if getattr(controller, "phase3_arrived_latched", False):
            return True
        if snapshot.get("gps_detect") != GPS_ACTIVE_DETECT:
            return False
        try:
            lat = float(snapshot.get("lat", 0.0))
            lng = float(snapshot.get("lng", 0.0))
        except (TypeError, ValueError):
            return False
        if not math.isfinite(lat) or not math.isfinite(lng) or (lat == 0.0 and lng == 0.0):
            return False

        distance_m, azimuth = calc_distance_and_azimuth(
            lat, lng, controller.target_lat, controller.target_lng
        )
        inside = distance_m <= float(GPS_CLOSE_DISTANCE)
        controller.st.update_navigation(distance=distance_m, azimuth=azimuth, direction=azimuth)
        try:
            gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
        except (TypeError, ValueError):
            gps_fix_seq = 0
        if inside:
            if getattr(controller, "phase3_arrival_inside_since", None) is None:
                controller.phase3_arrival_inside_since = now
            if gps_fix_seq != int(getattr(controller, "phase2_arrival_last_gps_fix_seq", -1)):
                controller.phase3_arrival_confirm_count += 1
                controller.phase2_arrival_last_gps_fix_seq = gps_fix_seq
        else:
            controller.phase3_arrival_confirm_count = 0
            controller.phase3_arrival_inside_since = None
            controller.phase2_arrival_last_gps_fix_seq = gps_fix_seq

        inside_since = getattr(controller, "phase3_arrival_inside_since", None)
        inside_elapsed = now - inside_since if inside_since is not None else 0.0
        confirmed = inside and (
            controller.phase3_arrival_confirm_count >= int(PHASE3_ARRIVAL_CONFIRM_COUNT)
            or inside_elapsed >= float(PHASE3_ARRIVAL_CONFIRM_SEC)
        )
        if confirmed:
            controller.phase3_arrived_latched = True
        controller.st.update_navigation(
            arrival_inside=inside,
            arrival_confirm_count=controller.phase3_arrival_confirm_count,
            phase3_arrived_latched=controller.phase3_arrived_latched,
        )
        return controller.phase3_arrived_latched

    @staticmethod
    def _transition_arrived_to_phase4(controller, now):
        print("Phase2: GPS goal region confirmed -> Phase4")
        controller.st.update_navigation(
            phase=int(Phase.PHASE4),
            arrival_inside=True,
            phase3_arrived_latched=True,
        )
        controller.time_phase4_start = now

    @classmethod
    def _update_offset_segment(cls, controller, snapshot):
        reference = cls._normalize_heading(getattr(controller, "phase2_offset_reference_bno_deg", None))
        if reference is None and snapshot.get("angle_valid", False):
            reference = cls._normalize_heading(snapshot.get("angle"))
            controller.phase2_offset_reference_bno_deg = reference
        if reference is None or not snapshot.get("angle_valid", False):
            controller.phase2_offset_reject_reason = "bno_unavailable"
            return

        try:
            gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
            lat = float(snapshot.get("lat", 0.0))
            lng = float(snapshot.get("lng", 0.0))
        except (TypeError, ValueError):
            return
        bno_heading = cls._normalize_heading(snapshot.get("angle"))
        if (
            gps_fix_seq <= int(getattr(controller, "phase2_offset_last_gps_fix_seq", 0))
            or gps_fix_seq <= 0
            or bno_heading is None
            or not math.isfinite(lat)
            or not math.isfinite(lng)
            or (lat == 0.0 and lng == 0.0)
        ):
            return

        controller.phase2_offset_last_gps_fix_seq = gps_fix_seq
        sample = {
            "gps_fix_seq": gps_fix_seq,
            "lat": lat,
            "lng": lng,
            "bno_heading": bno_heading,
        }
        samples = getattr(controller, "phase2_offset_samples", None)
        if not isinstance(samples, list):
            samples = []
            controller.phase2_offset_samples = samples
        samples.append(sample)
        if len(samples) >= 2:
            distance_m, _ = calc_distance_and_azimuth(
                samples[0]["lat"], samples[0]["lng"], lat, lng
            )
            controller.phase2_offset_distance_m = distance_m
        controller.bno_heading_offset_candidate_count = len(samples)

        if (
            len(samples) < int(PHASE2_OFFSET_MIN_SAMPLES)
            or controller.phase2_offset_distance_m < float(PHASE2_OFFSET_MIN_DISTANCE_M)
        ):
            return

        estimate = cls._estimate_offset_segment(samples)
        controller.phase2_offset_distance_m = estimate["distance_m"]
        controller.phase2_offset_path_efficiency = estimate["path_efficiency"]
        controller.phase2_offset_course_deg = estimate["course_deg"]
        controller.phase2_offset_bno_mean_deg = estimate["bno_mean_deg"]
        controller.phase2_offset_bno_spread_deg = estimate["bno_spread_deg"]
        controller.phase2_offset_subsegment_diff_deg = estimate["subsegment_diff_deg"]
        controller.bno_heading_offset_candidate_deg = estimate["offset_deg"]
        controller.phase2_offset_reject_reason = estimate["reason"]
        if estimate["valid"]:
            controller.bno_heading_offset_deg = estimate["offset_deg"]
            controller.bno_heading_offset_valid = True
            return

        controller.phase2_offset_attempt_count += 1
        cls._begin_offset_turnaround(controller, snapshot, estimate["reason"])

    @staticmethod
    def _enter_stage(controller, stage, now, snapshot=None):
        controller.phase2_stage = stage
        controller.phase2_stage_start = now
        if stage == PHASE2_STAGE_CALIBRATION:
            controller.phase2_calib_ready_since = None
        if stage != PHASE2_STAGE_OFFSET:
            return

        controller.bno_heading_offset_deg = 0.0
        controller.bno_heading_offset_valid = False
        controller.bno_heading_offset_candidate_deg = None
        controller.bno_heading_offset_candidate_count = 0
        if snapshot is None:
            snapshot = {}
        controller.phase2_offset_attempt_count = 0
        controller.phase2_offset_stage_retry_count = 0
        Phase2Handler._reset_offset_collection(controller, snapshot, now, "settling")

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if controller.phase2_start_time is None:
            controller.phase2_start_time = time.time()
            controller.phase2_stage = PHASE2_STAGE_ESCAPE
            controller.phase2_stage_start = controller.phase2_start_time
        now = time.time()
        stage_elapsed = now - controller.phase2_stage_start
        controller.led_blink_timer += 1
        controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE2)

        calib = controller.bno_calib
        calib_ok = calib["valid"] and calib["value"][3] >= BNO_CALIB_MAG_MIN
        goal_latched = self._update_phase2_goal(controller, snapshot, now)
        if controller.led_blink_timer % PHASE_LOG_INTERVAL == 0 and calib["valid"]:
            sys_st, gyro_st, accel_st, mag_st = calib["value"]
            print(
                f"Phase2 {controller.phase2_stage}: "
                f"Sys={sys_st} Gyro={gyro_st} Acc={accel_st} Mag={mag_st}"
            )

        if controller.phase2_stage == PHASE2_STAGE_ESCAPE:
            if stage_elapsed >= PHASE2_ESCAPE_TIME:
                if goal_latched:
                    self._transition_arrived_to_phase4(controller, now)
                else:
                    print("Phase2: parachute escape complete -> magnetic calibration")
                    self._enter_stage(controller, PHASE2_STAGE_CALIBRATION, now)
            return

        if goal_latched:
            self._transition_arrived_to_phase4(controller, now)
            return

        if controller.phase2_stage == PHASE2_STAGE_CALIBRATION:
            if calib_ok:
                if getattr(controller, "phase2_calib_ready_since", None) is None:
                    controller.phase2_calib_ready_since = now
            else:
                controller.phase2_calib_ready_since = None
            calib_ready_since = getattr(controller, "phase2_calib_ready_since", None)
            calib_stable_sec = now - calib_ready_since if calib_ready_since is not None else 0.0
            calibration_complete = (
                stage_elapsed >= PHASE2_CALIB_MIN_TIME
                and calib_stable_sec >= PHASE2_CALIB_STABLE_SEC
            )
            calibration_timed_out = stage_elapsed >= PHASE2_CALIB_MAX_TIME
            if calibration_complete or calibration_timed_out:
                reason = "calibration complete" if calibration_complete else "calibration time limit"
                print(f"Phase2: {reason} -> BNO/GPS offset straight")
                self._enter_stage(controller, PHASE2_STAGE_OFFSET, now, snapshot)
            return

        if controller.phase2_stage == PHASE2_STAGE_OFFSET:
            recovery_seq = int(getattr(controller, "bno_heading_recovery_seq", 0))
            observed_recovery_seq = int(
                getattr(controller, "phase2_offset_observed_bno_recovery_seq", 0)
            )
            if recovery_seq != observed_recovery_seq:
                print("Phase2: BNO heading recovered; restarting offset segment")
                self._reset_offset_collection(controller, snapshot, now, "bno_recovered_reset")

            if controller.phase2_offset_mode == PHASE2_OFFSET_MODE_TURNAROUND:
                target = self._normalize_heading(controller.phase2_offset_turn_target_deg)
                angle = (
                    self._normalize_heading(snapshot.get("angle"))
                    if snapshot.get("angle_valid", False)
                    else None
                )
                if target is None and angle is not None:
                    target = (angle + 180.0) % 360.0
                    controller.phase2_offset_turn_target_deg = target
                if target is not None and angle is not None:
                    turn_error = self._angle_diff(target, angle)
                    controller.phase2_offset_heading_error_deg = turn_error
                    if abs(turn_error) <= float(PHASE2_OFFSET_TURN_DEADBAND_DEG):
                        controller.phase2_offset_turn_confirm_count += 1
                    else:
                        controller.phase2_offset_turn_confirm_count = 0
                    if controller.phase2_offset_turn_confirm_count >= int(
                        PHASE2_OFFSET_TURN_CONFIRM_COUNT
                    ):
                        self._reset_offset_collection(
                            controller, snapshot, now, "settling_after_turnaround"
                        )

            if (
                getattr(controller, "phase2_offset_reference_bno_deg", None) is None
                and snapshot.get("angle_valid", False)
            ):
                controller.phase2_offset_reference_bno_deg = self._normalize_heading(snapshot.get("angle"))
            if (
                controller.phase2_offset_mode == PHASE2_OFFSET_MODE_COLLECT
                and now >= float(getattr(controller, "phase2_offset_settle_until", 0.0))
            ):
                if controller.phase2_offset_reject_reason.startswith("settling"):
                    controller.phase2_offset_reject_reason = ""
                self._update_offset_segment(controller, snapshot)
            controller.st.update_navigation(
                bno_offset_deg=float(controller.bno_heading_offset_deg),
                bno_offset_valid=bool(controller.bno_heading_offset_valid),
            )
            offset_complete = controller.bno_heading_offset_valid
            offset_leg_elapsed = now - float(
                getattr(controller, "phase2_offset_leg_start_time", now)
            )
            offset_timed_out = (
                controller.phase2_offset_mode == PHASE2_OFFSET_MODE_COLLECT
                and offset_leg_elapsed >= float(PHASE2_OFFSET_LEG_MAX_TIME)
            )
            if offset_complete or offset_timed_out:
                if offset_complete:
                    print(
                        "Phase2: BNO/GPS offset learned "
                        f"({controller.bno_heading_offset_deg:.1f} deg) -> Phase3"
                    )
                else:
                    controller.phase2_offset_stage_retry_count += 1
                    self._begin_offset_turnaround(controller, snapshot, "leg_timeout")
                    print(
                        "Phase2: offset leg time limit -> compact turnaround retry "
                        f"#{controller.phase2_offset_stage_retry_count}"
                    )
                    return
                controller.st.update_navigation(phase=int(Phase.PHASE3))
                controller.time_phase3_start = now
            return

        print(f"Phase2: unknown stage {controller.phase2_stage!r}; restarting escape stage")
        self._enter_stage(controller, PHASE2_STAGE_ESCAPE, now)


def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE2)


if __name__ == "__main__":
    run_standalone()
