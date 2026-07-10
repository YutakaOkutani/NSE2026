import sys
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    PHASE2_CALIB_MIN_TIME,
    PHASE2_OFFSET_CONTROL_SETTLE_TIME,
    PHASE2_STAGE_CALIBRATION,
    PHASE2_STAGE_ESCAPE,
    PHASE2_STAGE_OFFSET,
    Phase,
)
from mission.phases.p2 import Phase2Handler


class _State:
    def __init__(self):
        self.values = {"phase": int(Phase.PHASE2)}

    def update_navigation(self, **kwargs):
        self.values.update(kwargs)


class _Controller:
    def __init__(self):
        self.devices = {}
        self.st = _State()
        self.phase2_start_time = 0.0
        self.phase2_stage = PHASE2_STAGE_ESCAPE
        self.phase2_stage_start = 0.0
        self.led_blink_timer = 0
        self.bno_calib = {"valid": True, "value": (3, 3, 3, 2)}
        self.bno_heading_offset_deg = 0.0
        self.bno_heading_offset_valid = False
        self.bno_heading_offset_candidate_deg = None
        self.bno_heading_offset_candidate_count = 0
        self._heading_offset_last_gps_fix_seq = 0
        self.phase2_offset_reference_bno_deg = None
        self.phase2_offset_heading_error_deg = 0.0
        self.phase2_offset_samples = []
        self.phase2_offset_last_gps_fix_seq = 0
        self.phase2_offset_distance_m = 0.0
        self.phase2_offset_path_efficiency = 0.0
        self.phase2_offset_course_deg = 0.0
        self.phase2_offset_bno_mean_deg = 0.0
        self.phase2_offset_bno_spread_deg = 0.0
        self.phase2_offset_subsegment_diff_deg = 0.0
        self.phase2_offset_attempt_count = 0
        self.phase2_offset_reject_reason = ""
        self.time_phase3_start = None

    def toggle_led(self, *_args, **_kwargs):
        return None

class Phase2FlowTest(unittest.TestCase):
    def test_escape_transitions_to_calibration(self):
        ctrl = _Controller()

        with patch("mission.phases.p2.time.time", return_value=7.0):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 3})

        self.assertEqual(ctrl.phase2_stage, PHASE2_STAGE_CALIBRATION)
        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE2))

    def test_calibration_does_not_exit_before_minimum_time(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_CALIBRATION

        with patch("mission.phases.p2.time.time", return_value=PHASE2_CALIB_MIN_TIME - 0.1):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 3})

        self.assertEqual(ctrl.phase2_stage, PHASE2_STAGE_CALIBRATION)

    def test_calibration_transitions_to_fresh_offset_learning(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_CALIBRATION
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_candidate_count = 9

        with patch("mission.phases.p2.time.time", return_value=PHASE2_CALIB_MIN_TIME + 0.1):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 12})

        self.assertEqual(ctrl.phase2_stage, PHASE2_STAGE_OFFSET)
        self.assertFalse(ctrl.bno_heading_offset_valid)
        self.assertEqual(ctrl.bno_heading_offset_candidate_count, 0)
        self.assertEqual(ctrl._heading_offset_last_gps_fix_seq, 12)

    def test_valid_offset_completes_phase2(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.bno_heading_offset_deg = 172.0
        ctrl.bno_heading_offset_valid = True
        with patch("mission.phases.p2.time.time", return_value=PHASE2_OFFSET_CONTROL_SETTLE_TIME + 0.1):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 20})

        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE3))
        self.assertTrue(ctrl.st.values["bno_offset_valid"])

    def test_offset_learning_waits_for_heading_hold_to_settle(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET

        with patch(
            "mission.phases.p2.time.time",
            return_value=PHASE2_OFFSET_CONTROL_SETTLE_TIME - 0.1,
        ):
            Phase2Handler().execute(
                ctrl,
                {"gps_fix_seq": 20, "angle_valid": True, "angle": 270.0, "lat": 35.0, "lng": 139.0},
            )

        self.assertEqual(ctrl.phase2_offset_samples, [])
        self.assertEqual(ctrl.phase2_offset_reference_bno_deg, 270.0)

    def test_interval_estimator_accepts_straight_stable_run(self):
        samples = [
            {
                "gps_fix_seq": index + 1,
                "lat": 35.0 + index * 0.000012,
                "lng": 139.0,
                "bno_heading": 270.0,
            }
            for index in range(8)
        ]

        estimate = Phase2Handler._estimate_offset_segment(samples)

        self.assertTrue(estimate["valid"])
        self.assertAlmostEqual(estimate["course_deg"], 0.0, delta=0.5)
        self.assertAlmostEqual(estimate["offset_deg"], 90.0, delta=0.5)

    def test_interval_estimator_rejects_unstable_bno_heading(self):
        samples = [
            {
                "gps_fix_seq": index + 1,
                "lat": 35.0 + index * 0.000012,
                "lng": 139.0,
                "bno_heading": 250.0 if index % 2 == 0 else 290.0,
            }
            for index in range(8)
        ]

        estimate = Phase2Handler._estimate_offset_segment(samples)

        self.assertFalse(estimate["valid"])
        self.assertEqual(estimate["reason"], "bno_heading_unstable")

    def test_interval_collector_latches_valid_offset(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_reference_bno_deg = 270.0

        for index in range(8):
            Phase2Handler._update_offset_segment(
                ctrl,
                {
                    "gps_fix_seq": index + 1,
                    "lat": 35.0 + index * 0.000012,
                    "lng": 139.0,
                    "angle_valid": True,
                    "angle": 270.0,
                },
            )

        self.assertTrue(ctrl.bno_heading_offset_valid)
        self.assertAlmostEqual(ctrl.bno_heading_offset_deg, 90.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()
