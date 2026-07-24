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
    GPS_ACTIVE_DETECT,
    GPS_CLOSE_DISTANCE,
    GPS_LOST_LOG_INTERVAL,
    LED_INTERVAL_PHASE3,
    PHASE_LOG_INTERVAL,
    PHASE3_ARRIVAL_CONFIRM_COUNT,
    PHASE3_ARRIVAL_CONFIRM_SEC,
    Phase,
    TIMEOUT_PHASE_3,
)
from mission.nav import calc_distance_and_azimuth
from mission.phases.base import BasePhaseHandler


class Phase3Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.off()
        if not bool(getattr(controller, "phase3_heading_entry_ready", False)):
            print("Phase3 entry warning: BNO_ALIGNED quality is not ready -> continuing with fallback heading")
            controller.phase3_heading_entry_ready = True
            if getattr(controller, "bno_heading_offset_deg", None) is None:
                controller.bno_heading_offset_deg = 0.0
            controller.bno_heading_offset_valid = True
        controller.toggle_led(led_green, controller.led_blink_timer, interval=LED_INTERVAL_PHASE3)
        if time.time() - controller.time_phase3_start > TIMEOUT_PHASE_3:
            print("Phase3 TIMEOUT: switching to Phase4")
            controller.st.update_navigation(phase=int(Phase.PHASE4))
            controller.time_phase4_start = time.time()
            return
        if snapshot["gps_detect"] == GPS_ACTIVE_DETECT:
            dist, azimuth = calc_distance_and_azimuth(snapshot["lat"], snapshot["lng"], controller.target_lat, controller.target_lng)
            controller.st.update_navigation(distance=dist, azimuth=azimuth, direction=azimuth)
            inside_arrival = dist <= GPS_CLOSE_DISTANCE
            now = time.time()
            if getattr(controller, "phase3_arrived_latched", False):
                controller.st.update_navigation(
                    arrival_inside=True,
                    arrival_confirm_count=getattr(controller, "phase3_arrival_confirm_count", 0),
                    phase3_arrived_latched=True,
                )
                controller.st.update_navigation(phase=int(Phase.PHASE4))
                controller.time_phase4_start = now
                return
            if inside_arrival:
                controller.phase3_arrival_confirm_count = int(
                    getattr(controller, "phase3_arrival_confirm_count", 0)
                ) + 1
                if getattr(controller, "phase3_arrival_inside_since", None) is None:
                    controller.phase3_arrival_inside_since = now
            else:
                controller.phase3_arrival_confirm_count = 0
                controller.phase3_arrival_inside_since = None
            inside_since = getattr(controller, "phase3_arrival_inside_since", None)
            inside_elapsed = (now - inside_since) if inside_since is not None else 0.0
            arrival_confirmed = (
                inside_arrival
                and (
                    controller.phase3_arrival_confirm_count >= int(PHASE3_ARRIVAL_CONFIRM_COUNT)
                    or inside_elapsed >= float(PHASE3_ARRIVAL_CONFIRM_SEC)
                )
            )
            nav_heading = None
            nav_heading_source = "INVALID"
            if hasattr(controller, "_phase3_heading"):
                nav_heading, nav_heading_source = controller._phase3_heading(snapshot)
            elif hasattr(controller, "_phase45_bno_heading"):
                nav_heading = controller._phase45_bno_heading(snapshot)
                nav_heading_source = "BNO" if nav_heading is not None else "INVALID"
            heading_diff = None
            if nav_heading is not None:
                heading_diff = controller._angle_diff_deg(azimuth, nav_heading)
            controller.st.update_navigation(
                nav_heading=nav_heading if nav_heading is not None else 0.0,
                nav_heading_source=nav_heading_source,
                heading_diff=heading_diff if heading_diff is not None else 0.0,
                heading_trust=getattr(controller, "_phase3_last_heading_trust", 0.0),
                bno_trusted=bool(getattr(controller, "_phase3_last_bno_trusted", False)),
                bno_offset_deg=float(getattr(controller, "bno_heading_offset_deg", 0.0)),
                bno_offset_valid=bool(getattr(controller, "bno_heading_offset_valid", False)),
                arrival_inside=inside_arrival,
                arrival_confirm_count=controller.phase3_arrival_confirm_count,
                phase3_arrived_latched=bool(getattr(controller, "phase3_arrived_latched", False)),
            )
            if controller.led_blink_timer % PHASE_LOG_INTERVAL == 0:
                if nav_heading is not None and heading_diff is not None:
                    print(
                        f"GPS Nav: Dist={dist:.1f}m, TargetDir={azimuth:.1f}, "
                        f"Head({nav_heading_source})={nav_heading:.1f}, Diff={heading_diff:+.1f}"
                    )
                else:
                    print(
                        f"GPS Nav: Dist={dist:.1f}m, TargetDir={azimuth:.1f}, "
                        f"Head(INVALID:{nav_heading_source}), "
                        f"AngleValid={int(bool(snapshot.get('angle_valid', False)))}, "
                        f"GPSHeadingValid={int(bool(snapshot.get('gps_heading_valid', False)))}, "
                        f"GPSSpeed={float(snapshot.get('gps_speed_mps', 0.0)):.2f}m/s, "
                        f"BNOStale={float(getattr(controller, 'bno_stale_sec', 0.0)):.2f}s"
                    )
            if arrival_confirmed:
                controller.phase3_arrived_latched = True
                controller.st.update_navigation(phase3_arrived_latched=True)
                print(
                    f"Close enough ({dist:.1f}m, confirm={controller.phase3_arrival_confirm_count}, "
                    f"inside={inside_elapsed:.1f}s): switching to Phase4"
                )
                controller.st.update_navigation(phase=int(Phase.PHASE4))
                controller.time_phase4_start = now
        else:
            if controller.led_blink_timer % GPS_LOST_LOG_INTERVAL == 0:
                print("GPS Lost: Keep going...")


def run_standalone():
    from mission.run import run_single_phase

    run_single_phase(Phase.PHASE3)


if __name__ == "__main__":
    run_standalone()
