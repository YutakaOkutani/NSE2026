import sys
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    MISSION_PHASE3_CUMULATIVE_BUDGET,
    MISSION_PHASE3_MIN_RESERVE_SEC,
    PHASE2_CALIB_MIN_TIME,
    PHASE2_CALIB_STABLE_SEC,
    PHASE2_OFFSET_CONTROL_SETTLE_TIME,
    PHASE2_OFFSET_LEG_MAX_TIME,
    PHASE2_OFFSET_MAX_DISTANCE_M,
    PHASE2_OFFSET_MAX_SUBSEGMENT_DIFF_DEG,
    PHASE2_OFFSET_NEAR_GOAL_GRACE_SEC,
    PHASE2_OFFSET_MODE_COLLECT,
    PHASE2_OFFSET_MODE_READINESS,
    PHASE2_OFFSET_MODE_TURNAROUND,
    PHASE2_OFFSET_MAX_RETRIES,
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
        self.phase2_calib_ready_since = None
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
        self.phase2_offset_mode = PHASE2_OFFSET_MODE_COLLECT
        self.phase2_offset_turn_target_deg = None
        self.phase2_offset_turn_confirm_count = 0
        self.phase2_offset_stage_retry_count = 0
        self.phase2_offset_settle_until = 0.0
        self.phase2_offset_leg_start_time = 0.0
        self.phase2_offset_progress_anchor_time = 0.0
        self.phase2_offset_progress_anchor_distance_m = 0.0
        self.phase2_offset_near_goal_active = False
        self.phase2_offset_leg_time_limit_sec = PHASE2_OFFSET_LEG_MAX_TIME
        self.phase2_offset_observed_bno_recovery_seq = 0
        self.phase2_offset_reject_reason = ""
        self.phase2_offset_mode_start_time = 0.0
        self.phase2_entry_ready_count = 0
        self.phase2_heading_quality_valid = False
        self.phase2_last_bno_recovery_time = 0.0
        self.phase3_heading_entry_ready = False
        self.bno_stale_sec = 0.0
        self.bno_heading_recovery_seq = 0
        self.phase3_arrival_confirm_count = 0
        self.phase3_arrival_inside_since = None
        self.phase3_arrived_latched = False
        self.phase2_arrival_last_gps_fix_seq = -1
        self.target_lat = 35.0
        self.target_lng = 139.0
        self.time_phase3_start = None
        self.time_phase4_start = None
        self.mission_end_reason = "RUNNING"

    def toggle_led(self, *_args, **_kwargs):
        return None

class Phase2FlowTest(unittest.TestCase):
    def test_phase2_budget_preserves_minimum_phase3_navigation_time(self):
        self.assertGreaterEqual(
            MISSION_PHASE3_CUMULATIVE_BUDGET,
            MISSION_PHASE3_MIN_RESERVE_SEC,
        )

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
        ctrl.phase2_calib_ready_since = 0.0

        with patch(
            "mission.phases.p2.time.time",
            return_value=max(PHASE2_CALIB_MIN_TIME, PHASE2_CALIB_STABLE_SEC) + 0.1,
        ):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 12})

        self.assertEqual(ctrl.phase2_stage, PHASE2_STAGE_OFFSET)
        self.assertFalse(ctrl.bno_heading_offset_valid)
        self.assertEqual(ctrl.bno_heading_offset_candidate_count, 0)
        self.assertEqual(ctrl._heading_offset_last_gps_fix_seq, 12)

    def test_valid_offset_completes_phase2(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_settle_until = PHASE2_OFFSET_CONTROL_SETTLE_TIME
        ctrl.bno_heading_offset_deg = 172.0
        ctrl.bno_heading_offset_valid = True
        ctrl.phase2_heading_quality_valid = True
        ctrl.phase2_offset_mode = PHASE2_OFFSET_MODE_READINESS
        ctrl.phase2_entry_ready_count = 9
        with patch("mission.phases.p2.time.time", return_value=PHASE2_OFFSET_CONTROL_SETTLE_TIME + 2.1):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 20, "angle_valid": True, "angle": 90.0})

        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE3))
        self.assertTrue(ctrl.st.values["bno_offset_valid"])

    def test_offset_learning_waits_for_heading_hold_to_settle(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_settle_until = PHASE2_OFFSET_CONTROL_SETTLE_TIME

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

    def test_goal_latch_waits_for_escape_then_skips_to_phase4(self):
        ctrl = _Controller()
        handler = Phase2Handler()
        snapshots = [
            {
                "gps_detect": 1,
                "gps_fix_seq": index,
                "lat": 35.0,
                "lng": 139.0,
            }
            for index in (1, 2, 3)
        ]
        for now, snapshot in zip((0.1, 0.5, 1.2), snapshots):
            with patch("mission.phases.p2.time.time", return_value=now):
                handler.execute(ctrl, snapshot)

        self.assertTrue(ctrl.phase3_arrived_latched)
        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE2))

        with patch("mission.phases.p2.time.time", return_value=7.0):
            handler.execute(ctrl, snapshots[-1])

        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE4))

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

    def test_interval_estimator_accepts_field_realistic_bno_drift(self):
        headings = (254.0, 260.0, 266.0, 274.0, 280.0, 286.0, 286.0)
        samples = [
            {
                "gps_fix_seq": index + 1,
                "lat": 35.0 + index * 0.000005,
                "lng": 139.0,
                "bno_heading": heading,
            }
            for index, heading in enumerate(headings)
        ]

        estimate = Phase2Handler._estimate_offset_segment(samples)

        self.assertTrue(estimate["valid"])
        self.assertGreater(estimate["bno_spread_deg"], 12.0)
        self.assertLessEqual(estimate["bno_spread_deg"], 20.0)
        self.assertLessEqual(estimate["subsegment_diff_deg"], 25.0)

    def test_interval_estimator_rejects_unstable_bno_heading(self):
        samples = [
            {
                "gps_fix_seq": index + 1,
                "lat": 35.0 + index * 0.000012,
                "lng": 139.0,
                "bno_heading": 245.0 if index % 2 == 0 else 295.0,
            }
            for index in range(8)
        ]

        estimate = Phase2Handler._estimate_offset_segment(samples)

        self.assertFalse(estimate["valid"])
        self.assertEqual(estimate["reason"], "bno_heading_unstable")

    def test_rejected_segment_requests_forward_reorient(self):
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

        Phase2Handler._begin_offset_retry(ctrl, 10.0, "bno_heading_unstable")
        self.assertEqual(ctrl.phase2_offset_mode, PHASE2_OFFSET_MODE_TURNAROUND)
        self.assertIsNone(ctrl.phase2_offset_turn_target_deg)

    def test_rejected_segment_at_max_distance_falls_back_to_phase3_gps_only(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_reference_bno_deg = 270.0
        headings = (251.0, 251.0, 251.0, 289.0, 289.0, 289.0)

        with patch("mission.phases.p2.time.time", return_value=10.0):
            for index, heading in enumerate(headings):
                Phase2Handler._update_offset_segment(
                    ctrl,
                    {
                        "gps_fix_seq": index + 1,
                        "lat": 35.0 + index * 0.000012,
                        "lng": 139.0,
                        "angle_valid": True,
                        "angle": heading,
                    },
                )

        self.assertGreaterEqual(
            ctrl.phase2_offset_distance_m,
            PHASE2_OFFSET_MAX_DISTANCE_M,
        )
        self.assertGreater(
            ctrl.phase2_offset_subsegment_diff_deg,
            PHASE2_OFFSET_MAX_SUBSEGMENT_DIFF_DEG,
        )
        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE3))
        self.assertFalse(ctrl.bno_heading_offset_valid)
        self.assertFalse(ctrl.bno_heading_offset_verified)
        self.assertTrue(ctrl.phase3_heading_entry_ready)
        self.assertEqual(
            ctrl.phase2_offset_reject_reason,
            "gps_only:offset_inconsistent",
        )

    def test_repeated_offset_timeout_falls_back_to_phase3(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_stage_retry_count = int(PHASE2_OFFSET_MAX_RETRIES)

        with patch("mission.phases.p2.time.time", return_value=PHASE2_OFFSET_LEG_MAX_TIME + 0.1):
            Phase2Handler().execute(ctrl, {"gps_fix_seq": 20, "angle_valid": True, "angle": 90.0})

        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE3))
        self.assertFalse(ctrl.bno_heading_offset_valid)
        self.assertFalse(ctrl.bno_heading_offset_verified)
        self.assertTrue(ctrl.phase3_heading_entry_ready)
        self.assertIn("gps_only:", ctrl.phase2_offset_reject_reason)

    def test_first_offset_timeout_retries_with_forward_reorient(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET

        with patch("mission.phases.p2.time.time", return_value=PHASE2_OFFSET_LEG_MAX_TIME + 0.1):
            Phase2Handler().execute(
                ctrl,
                {"gps_fix_seq": 20, "angle_valid": True, "angle": 90.0},
            )

        self.assertEqual(ctrl.st.values["phase"], int(Phase.PHASE2))
        self.assertEqual(ctrl.phase2_offset_mode, PHASE2_OFFSET_MODE_TURNAROUND)
        self.assertEqual(ctrl.phase2_offset_stage_retry_count, 1)

    def test_near_goal_extends_leg_deadline_without_lowering_five_meter_requirement(self):
        ctrl = _Controller()
        ctrl.phase2_offset_distance_m = 4.1

        self.assertFalse(Phase2Handler._offset_progress_stalled(ctrl, 24.9))
        self.assertTrue(ctrl.phase2_offset_near_goal_active)
        self.assertEqual(
            ctrl.phase2_offset_leg_time_limit_sec,
            PHASE2_OFFSET_LEG_MAX_TIME + PHASE2_OFFSET_NEAR_GOAL_GRACE_SEC,
        )
        self.assertFalse(ctrl.bno_heading_offset_valid)

    def test_near_goal_small_progress_refreshes_stall_window(self):
        ctrl = _Controller()
        ctrl.phase2_offset_distance_m = 4.1
        self.assertFalse(Phase2Handler._offset_progress_stalled(ctrl, 20.0))

        ctrl.phase2_offset_distance_m = 4.21
        self.assertFalse(Phase2Handler._offset_progress_stalled(ctrl, 24.9))
        self.assertFalse(Phase2Handler._offset_progress_stalled(ctrl, 29.8))
        self.assertTrue(Phase2Handler._offset_progress_stalled(ctrl, 30.0))

    def test_near_goal_grace_has_hard_ten_second_cap(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_near_goal_active = True
        ctrl.phase2_offset_distance_m = 4.8
        ctrl.phase2_offset_leg_time_limit_sec = (
            PHASE2_OFFSET_LEG_MAX_TIME + PHASE2_OFFSET_NEAR_GOAL_GRACE_SEC
        )
        ctrl.phase2_offset_progress_anchor_time = PHASE2_OFFSET_LEG_MAX_TIME + 9.0
        ctrl.phase2_offset_progress_anchor_distance_m = 4.7

        now = PHASE2_OFFSET_LEG_MAX_TIME + PHASE2_OFFSET_NEAR_GOAL_GRACE_SEC + 0.1
        with patch("mission.phases.p2.time.time", return_value=now):
            Phase2Handler().execute(
                ctrl,
                {"gps_fix_seq": 20, "angle_valid": True, "angle": 90.0},
            )

        self.assertEqual(ctrl.phase2_offset_mode, PHASE2_OFFSET_MODE_TURNAROUND)
        self.assertEqual(ctrl.phase2_offset_stage_retry_count, 1)

    def test_reorient_time_does_not_consume_next_straight_leg(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_mode = PHASE2_OFFSET_MODE_TURNAROUND
        ctrl.phase2_offset_mode_start_time = 96.0
        ctrl.phase2_offset_stage_retry_count = 2

        with patch("mission.phases.p2.time.time", return_value=100.0):
            Phase2Handler().execute(
                ctrl,
                {"gps_fix_seq": 20, "angle_valid": True, "angle": 0.0},
            )

        self.assertEqual(ctrl.phase2_offset_mode, PHASE2_OFFSET_MODE_COLLECT)
        self.assertEqual(ctrl.phase2_offset_stage_retry_count, 2)

    def test_bno_recovery_restarts_offset_reference_and_samples(self):
        ctrl = _Controller()
        ctrl.phase2_stage = PHASE2_STAGE_OFFSET
        ctrl.phase2_offset_samples = [{"gps_fix_seq": 1}]
        ctrl.phase2_offset_reference_bno_deg = 10.0
        ctrl.bno_heading_recovery_seq = 1

        with patch("mission.phases.p2.time.time", return_value=3.0):
            Phase2Handler().execute(
                ctrl,
                {"gps_fix_seq": 20, "angle_valid": True, "angle": 220.0},
            )

        self.assertEqual(ctrl.phase2_offset_reference_bno_deg, 220.0)
        self.assertEqual(ctrl.phase2_offset_samples, [])
        self.assertEqual(ctrl.phase2_offset_reject_reason, "bno_recovered_reset")

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
