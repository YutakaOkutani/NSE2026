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
    LED_INTERVAL_PHASE2,
    PHASE2_CALIB_MAX_TIME,
    PHASE2_CALIB_MIN_TIME,
    PHASE2_ESCAPE_TIME,
    PHASE2_OFFSET_MAX_TIME,
    PHASE2_OFFSET_HEADING_SETTLE_TIME,
    PHASE2_OFFSET_MIN_TIME,
    PHASE2_STAGE_CALIBRATION,
    PHASE2_STAGE_ESCAPE,
    PHASE2_STAGE_OFFSET,
    PHASE_LOG_INTERVAL,
    Phase,
)
from mission.phases.base import BasePhaseHandler


class Phase2Handler(BasePhaseHandler):
    @staticmethod
    def _enter_stage(controller, stage, now, snapshot=None):
        controller.phase2_stage = stage
        controller.phase2_stage_start = now
        if stage != PHASE2_STAGE_OFFSET:
            return

        controller.bno_heading_offset_deg = 0.0
        controller.bno_heading_offset_valid = False
        controller.bno_heading_offset_candidate_deg = None
        controller.bno_heading_offset_candidate_count = 0
        if snapshot is None:
            snapshot = {}
        try:
            controller._heading_offset_last_gps_fix_seq = int(snapshot.get("gps_fix_seq", 0))
        except (TypeError, ValueError):
            controller._heading_offset_last_gps_fix_seq = 0

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
        if controller.led_blink_timer % PHASE_LOG_INTERVAL == 0 and calib["valid"]:
            sys_st, gyro_st, accel_st, mag_st = calib["value"]
            print(
                f"Phase2 {controller.phase2_stage}: "
                f"Sys={sys_st} Gyro={gyro_st} Acc={accel_st} Mag={mag_st}"
            )

        if controller.phase2_stage == PHASE2_STAGE_ESCAPE:
            if stage_elapsed >= PHASE2_ESCAPE_TIME:
                print("Phase2: parachute escape complete -> magnetic calibration")
                self._enter_stage(controller, PHASE2_STAGE_CALIBRATION, now)
            return

        if controller.phase2_stage == PHASE2_STAGE_CALIBRATION:
            calibration_complete = stage_elapsed >= PHASE2_CALIB_MIN_TIME and calib_ok
            calibration_timed_out = stage_elapsed >= PHASE2_CALIB_MAX_TIME
            if calibration_complete or calibration_timed_out:
                reason = "calibration complete" if calibration_complete else "calibration time limit"
                print(f"Phase2: {reason} -> BNO/GPS offset straight")
                self._enter_stage(controller, PHASE2_STAGE_OFFSET, now, snapshot)
            return

        if controller.phase2_stage == PHASE2_STAGE_OFFSET:
            if stage_elapsed >= PHASE2_OFFSET_HEADING_SETTLE_TIME:
                controller._update_bno_heading_offset_from_gps(snapshot)
            controller.st.update_navigation(
                bno_offset_deg=float(controller.bno_heading_offset_deg),
                bno_offset_valid=bool(controller.bno_heading_offset_valid),
            )
            offset_complete = stage_elapsed >= PHASE2_OFFSET_MIN_TIME and controller.bno_heading_offset_valid
            offset_timed_out = stage_elapsed >= PHASE2_OFFSET_MAX_TIME
            if offset_complete or offset_timed_out:
                if offset_complete:
                    print(
                        "Phase2: BNO/GPS offset learned "
                        f"({controller.bno_heading_offset_deg:.1f} deg) -> Phase3"
                    )
                else:
                    print("Phase2: offset learning time limit; BNO remains untrusted -> Phase3")
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
