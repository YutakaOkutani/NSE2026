import sys
import unittest
import importlib.util
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

MTR_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "mtr_mgr.py"
spec = importlib.util.spec_from_file_location("mtr_mgr_under_test", MTR_MGR_PATH)
mtr_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mtr_mgr_under_test)
MotorManager = mtr_mgr_under_test.MotorManager


class _HeadingOnlyController(MotorManager):
    def __init__(self):
        self.bno_heading_offset_valid = False
        self.bno_heading_offset_deg = 0.0
        self.mag_heading_offset_valid = False
        self.mag_heading_offset_deg = 0.0
        self.bno_stale_sec = 0.0
        self.phase2_offset_reference_bno_deg = None
        self.phase2_offset_heading_error_deg = 0.0
        self.phase2_offset_turn_target_deg = None
        self.motor_commands = []

    def set_motors(self, *args, **kwargs):
        self.motor_commands.append((args, kwargs))

    def stop_motors(self):
        self.motor_commands.append(((0.0, True, 0.0, True), {"cmd_type": "stop"}))

    def _normalize_heading_deg(self, value):
        try:
            deg = float(value)
        except (TypeError, ValueError):
            return None
        return deg % 360.0

    def _angle_diff_deg(self, target, current):
        diff = (float(target) - float(current) + 180.0) % 360.0 - 180.0
        return diff

    def _heading_offset_learning_ready(self, snapshot):
        return False

    def _update_bno_heading_offset_from_gps(self, snapshot):
        return None

    def _update_mag_heading_offset_from_gps(self, snapshot, gps_heading, mag_heading):
        return None

    def _bno_heading_aligned_to_gps(self, snapshot):
        heading = self._normalize_heading_deg(snapshot.get("angle"))
        if heading is None:
            return None
        return self._normalize_heading_deg(heading + self.bno_heading_offset_deg)


class Phase3HeadingTest(unittest.TestCase):
    def test_phase2_calibration_uses_forward_only_alternating_arcs(self):
        ctrl = _HeadingOnlyController()

        first = ctrl._phase2_calibration_pattern(0.0)
        second = ctrl._phase2_calibration_pattern(3.1)

        self.assertNotEqual(first[0], first[2])
        self.assertNotEqual(second[0], second[2])
        self.assertTrue(first[1])
        self.assertTrue(first[3])
        self.assertTrue(second[1])
        self.assertTrue(second[3])
        self.assertEqual(first[0], second[2])
        self.assertEqual(first[2], second[0])

    def test_phase2_reorient_uses_forward_only_arc(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase2_offset_stage_retry_count = 1

        ctrl._drive_phase2_forward_reorient()

        args, kwargs = ctrl.motor_commands[-1]
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertNotEqual(args[0], args[2])
        self.assertIn("forward_reorient", kwargs["cmd_type"])

    def test_phase2_offset_hold_steers_back_to_relative_bno_heading(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase2_offset_reference_bno_deg = 100.0

        speed_l, speed_r, error = ctrl._phase2_offset_hold_speeds(
            {"angle_valid": True, "angle": 90.0}
        )

        self.assertGreater(speed_l, speed_r)
        self.assertAlmostEqual(error, 10.0)

    def test_phase2_offset_hold_handles_heading_wraparound(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase2_offset_reference_bno_deg = 2.0

        speed_l, speed_r, error = ctrl._phase2_offset_hold_speeds(
            {"angle_valid": True, "angle": 358.0}
        )

        self.assertGreater(speed_l, speed_r)
        self.assertAlmostEqual(error, 4.0)

    def test_uses_bno_after_gps_alignment_is_valid(self):
        ctrl = _HeadingOnlyController()
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_deg = 10.0

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": True,
                "gps_heading": 90.0,
                "angle_valid": True,
                "angle": 30.0,
                "mag": [0.0, 0.0, 0.0],
            }
        )

        self.assertEqual(source, "BNO_ALIGNED")
        self.assertAlmostEqual(heading, 40.0)

    def test_falls_back_to_gps_before_bno_alignment_is_valid(self):
        ctrl = _HeadingOnlyController()

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": True,
                "gps_heading": 90.0,
                "angle_valid": True,
                "angle": 30.0,
                "mag": [0.0, 0.0, 0.0],
            }
        )

        self.assertEqual(source, "GPS_PRIMARY")
        self.assertAlmostEqual(heading, 90.0)

    def test_does_not_fall_back_to_mag_when_bno_and_gps_are_untrusted(self):
        ctrl = _HeadingOnlyController()
        ctrl.mag_heading_offset_valid = True
        ctrl.mag_heading_offset_deg = 25.0

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": False,
                "gps_heading": 0.0,
                "angle_valid": False,
                "angle": 30.0,
                "mag": [100.0, 25.0, 0.0],
            }
        )

        self.assertIsNone(heading)
        self.assertNotEqual(source, "MAG_ALIGNED")

    def test_phase3_large_heading_error_uses_pivot_turn(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_deg = 0.0

        # Snapshot with angle = 270 (facing West), target = 90 (East) => diff = 180 (large error)
        snapshot = {
            "gps_detect": 1,
            "lat": 35.0,
            "lng": 139.0,
            "gps_heading_valid": True,
            "gps_heading": 270.0,
            "gps_speed_mps": 0.5,
            "angle_valid": True,
            "angle": 270.0,
        }

        # Simulate motor control loop in Phase 3
        nav_heading, heading_source = ctrl._phase3_heading(snapshot)
        diff = ctrl._angle_diff_deg(90.0, nav_heading)
        turn_side = "left" if diff > 0 else "right"
        ctrl._set_forward_pivot_turn(
            turn_side,
            34,
            cmd_type="phase3_gps_pivot",
            speed_inner=0.0,
            ramp_time=0.1,
        )

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(kwargs["cmd_type"], "phase3_gps_pivot")
        # Ensure one wheel is stopped (0.0) for in-place pivot turn
        self.assertTrue(args[0] == 0.0 or args[2] == 0.0)


if __name__ == "__main__":
    unittest.main()
