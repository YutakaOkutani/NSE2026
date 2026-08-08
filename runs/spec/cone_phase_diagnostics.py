import sys
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import Phase
from mission.phases.p4 import Phase4Handler
from mission.phases.p5 import Phase5Handler
from mission.st import CanSatState


class _VisionController:
    def __init__(self, phase):
        self.st = CanSatState()
        self.st.update_navigation(phase=int(phase))
        self.devices = {}
        self.searching_flag = True
        self.time_start_searching_cone = 99.0
        self.camera_phase4_attempts = 1
        self.camera_phase4_start = 99.0
        self.camera_phase5_attempts = 1
        self.camera_phase5_start = 99.0
        self.camera_dead_since = None
        self.phase4_detect_confirm_count = 0
        self.phase4_detect_confirm_marker = None
        self.phase5_reach_confirm_count = 0
        self.phase5_entry_marker = 1.0
        self.phase_entry_time = 1.0
        self.phase4_camera_recovery_marker = 1.0
        self.phase5_timeout_limit_sec = 45.0
        self.phase5_entry_reason = "phase4_detected"
        self.time_camera_start = 99.0
        self.count_cone_lost = 0
        self.led_blink_timer = 0
        self.camera_recovery_exhausted = False
        self.camera_reinit_attempt_count = 0
        self.mission_end_reason = "RUNNING"
        self.stop_motor_calls = 0
        self.recovery_reset_calls = 0

    def transition_to_give_up(self, reason):
        self.mission_end_reason = reason
        self.st.update_navigation(phase=int(Phase.PHASE7))
        self.stop_motor_calls += 1

    def reset_camera_recovery_window(self):
        self.camera_recovery_exhausted = False
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_started_at = None
        self.recovery_reset_calls += 1


class ConePhaseDiagnosticsTest(unittest.TestCase):
    def test_new_phase4_entry_rearms_stale_camera_recovery_before_give_up(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.phase4_camera_recovery_marker = 0.0
        ctrl.camera_recovery_started_at = 0.5
        ctrl.camera_recovery_exhausted = True
        ctrl.camera_reinit_attempt_count = 3

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE4))
        self.assertEqual(ctrl.recovery_reset_calls, 1)
        self.assertEqual(ctrl.mission_end_reason, "RUNNING")

    def test_phase4_timeout_stops_and_skips_directly_to_phase7(self):
        ctrl = _VisionController(Phase.PHASE4)

        with patch("mission.phases.p4.time.time", return_value=160.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE7))
        self.assertEqual(ctrl.mission_end_reason, "PHASE4_TIMEOUT_GIVE_UP")
        self.assertEqual(ctrl.cone_phase_decision, "p4_timeout_to_p7_give_up")
        self.assertEqual(ctrl.stop_motor_calls, 1)

    def test_phase4_camera_recovery_exhaustion_gives_up(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.camera_recovery_exhausted = True
        ctrl.camera_reinit_attempt_count = 3

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.st.snapshot()["phase"], int(Phase.PHASE7))
        self.assertEqual(ctrl.mission_end_reason, "PHASE4_CAMERA_DEAD_GIVE_UP")
        self.assertEqual(ctrl.cone_phase_decision, "p4_camera_dead_to_p7_give_up")
        self.assertEqual(ctrl.stop_motor_calls, 1)

    def test_phase4_records_direction_consistency_reset(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.phase4_detect_confirm_count = 1
        ctrl.phase4_detect_confirm_marker = 0.73
        ctrl.st.update_cone(cone_direction=0.91, cone_probability=0.30, cone_is_reached=False)

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p4_direction_reset")
        self.assertTrue(ctrl.cone_phase_detected)
        self.assertTrue(ctrl.cone_phase_centered)
        self.assertFalse(ctrl.cone_phase_direction_consistent)
        self.assertEqual(ctrl.cone_phase_confirm_count, 1)
        self.assertEqual(ctrl.cone_phase_direction_tolerance, 0.14)

    def test_phase4_records_reached_signal_rejected_by_probability_gate(self):
        ctrl = _VisionController(Phase.PHASE4)
        ctrl.st.update_cone(cone_direction=0.50, cone_probability=0.08, cone_is_reached=True)

        with patch("mission.phases.p4.time.time", return_value=100.0):
            Phase4Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p4_no_detection")
        self.assertFalse(ctrl.cone_phase_reached_effective)
        self.assertEqual(ctrl.cone_phase_reached_probability_threshold, 0.28)

    def test_phase5_records_reached_signal_rejected_by_probability_gate(self):
        ctrl = _VisionController(Phase.PHASE5)
        ctrl.st.update_cone(cone_direction=0.50, cone_probability=0.08, cone_is_reached=True)

        with patch("mission.phases.p5.time.time", return_value=100.0):
            Phase5Handler().execute(ctrl, ctrl.st.snapshot())

        self.assertEqual(ctrl.cone_phase_decision, "p5_cone_lost_counting")
        self.assertFalse(ctrl.cone_phase_reached_effective)
        self.assertEqual(ctrl.cone_phase_reached_probability_threshold, 0.30)
        self.assertEqual(ctrl.cone_phase_required_confirm_frames, 8)


if __name__ == "__main__":
    unittest.main()
