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
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    PARACHUTE_DIRECTION,
    PHASE2_STAGE_ESCAPE,
    Phase,
    TIMEOUT_PHASE_1,
)
from mission.phases.base import BasePhaseHandler


class Phase1Handler(BasePhaseHandler):
    @staticmethod
    def _collect_offset_candidate(controller, snapshot):
        if not snapshot.get("angle_valid", False):
            return
        try:
            seq = int(snapshot.get("gps_fix_seq", 0))
            lat = float(snapshot.get("lat", 0.0))
            lng = float(snapshot.get("lng", 0.0))
            heading = float(snapshot.get("angle"))
        except (TypeError, ValueError):
            return
        if seq <= int(getattr(controller, "phase1_offset_last_gps_fix_seq", 0)):
            return
        if seq <= 0 or (lat == 0.0 and lng == 0.0):
            return
        controller.phase1_offset_last_gps_fix_seq = seq
        samples = getattr(controller, "phase1_offset_samples", None)
        if not isinstance(samples, list):
            samples = []
            controller.phase1_offset_samples = samples
        samples.append(
            {"gps_fix_seq": seq, "lat": lat, "lng": lng, "bno_heading": heading}
        )
        # Diagnostic candidate only: Phase1 parachute drag can bias course vs body heading.
        from mission.phases.p2 import Phase2Handler

        estimate = Phase2Handler._estimate_offset_segment(samples)
        controller.phase1_offset_distance_m = estimate["distance_m"]
        controller.phase1_offset_path_efficiency = estimate["path_efficiency"]
        controller.phase1_offset_bno_spread_deg = estimate["bno_spread_deg"]
        controller.phase1_offset_subsegment_diff_deg = estimate["subsegment_diff_deg"]
        controller.phase1_offset_reject_reason = estimate["reason"]
        if estimate["valid"]:
            controller.phase1_offset_candidate_valid = True
            controller.phase1_offset_candidate_deg = estimate["offset_deg"]

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.on()
        if led_green:
            led_green.off()
        print("PH1: Start Parachute Separation")
        if controller.time_phase1_start is None:
            controller.time_phase1_start = time.time()
        elapsed = time.time() - controller.time_phase1_start
        if elapsed < TIMEOUT_PHASE_1:
            self._collect_offset_candidate(controller, snapshot)
            controller.st.update_navigation(direction=PARACHUTE_DIRECTION, phase=int(Phase.PHASE1))
            return
        print("PH1: Parachute Separation TIMEOUT -> switching to Phase2")
        controller.st.update_navigation(phase=int(Phase.PHASE2))
        controller.phase2_start_time = time.time()
        controller.phase2_stage = PHASE2_STAGE_ESCAPE
        controller.phase2_stage_start = controller.phase2_start_time
        controller.time_phase1_start = None


def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE1)


if __name__ == "__main__":
    run_standalone()
