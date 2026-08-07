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
        self.phase5_timeout_limit_sec = 45.0
        self.phase5_entry_reason = "phase4_detected"
        self.time_camera_start = 99.0
        self.count_cone_lost = 0
        self.led_blink_timer = 0


class ConePhaseDiagnosticsTest(unittest.TestCase):
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
