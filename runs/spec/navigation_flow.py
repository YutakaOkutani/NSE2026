import math
import sys
import time
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import GPS_ACTIVE_DETECT, PHASE3_ARRIVAL_CONFIRM_COUNT, Phase
from mission.nav import calc_distance_and_azimuth
from mission.phases.p3 import Phase3Handler


class _State:
    def __init__(self, **values):
        self.values = dict(values)

    def snapshot(self):
        return dict(self.values)

    def update_navigation(self, **values):
        self.values.update(values)


class _Phase3Controller:
    def __init__(self, lat, lng, target_lat, target_lng, gps_detect=GPS_ACTIVE_DETECT):
        self.devices = {}
        self.target_lat = target_lat
        self.target_lng = target_lng
        self.time_phase3_start = time.time()
        self.time_phase4_start = 0.0
        self.led_blink_timer = 1
        self.phase3_arrived_latched = False
        self.phase3_arrival_confirm_count = 0
        self.phase3_arrival_inside_since = None
        self.st = _State(
            phase=int(Phase.PHASE3),
            lat=lat,
            lng=lng,
            gps_detect=gps_detect,
            angle_valid=False,
            gps_heading_valid=False,
            gps_speed_mps=0.0,
            angle=0.0,
        )

    def toggle_led(self, *_args, **_kwargs):
        return None

    @staticmethod
    def _angle_diff_deg(target, current):
        return (target - current + 180.0) % 360.0 - 180.0


class NavigationMathTest(unittest.TestCase):
    def test_identical_coordinates_have_zero_distance(self):
        distance, _ = calc_distance_and_azimuth(35.0, 139.0, 35.0, 139.0)
        self.assertEqual(distance, 0.0)

    def test_distance_is_symmetric_and_finite_at_antipodes(self):
        distance_ab, _ = calc_distance_and_azimuth(35.0, 139.0, -35.0, -41.0)
        distance_ba, _ = calc_distance_and_azimuth(-35.0, -41.0, 35.0, 139.0)
        self.assertTrue(math.isfinite(distance_ab))
        self.assertAlmostEqual(distance_ab, distance_ba, places=6)

    def test_cardinal_azimuths(self):
        _, north = calc_distance_and_azimuth(0.0, 0.0, 1.0, 0.0)
        _, east = calc_distance_and_azimuth(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(north, 0.0, places=6)
        self.assertAlmostEqual(east, 90.0, places=6)


class Phase3ArrivalTest(unittest.TestCase):
    def test_single_near_fix_does_not_transition(self):
        controller = _Phase3Controller(35.0, 139.0, 35.0, 139.0)
        Phase3Handler().execute(controller, controller.st.snapshot())
        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE3))
        self.assertEqual(controller.phase3_arrival_confirm_count, 1)

    def test_required_consecutive_near_fixes_transition_to_phase4(self):
        controller = _Phase3Controller(35.0, 139.0, 35.0, 139.0)
        handler = Phase3Handler()
        for _ in range(int(PHASE3_ARRIVAL_CONFIRM_COUNT)):
            handler.execute(controller, controller.st.snapshot())
        self.assertTrue(controller.phase3_arrived_latched)
        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE4))

    def test_inactive_gps_does_not_transition(self):
        controller = _Phase3Controller(35.0, 139.0, 35.0, 139.0, gps_detect=0)
        Phase3Handler().execute(controller, controller.st.snapshot())
        self.assertEqual(controller.st.snapshot()["phase"], int(Phase.PHASE3))


if __name__ == "__main__":
    unittest.main()
