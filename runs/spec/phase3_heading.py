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
        self.bno_heading_offset_verified = False
        self.bno_heading_offset_deg = 0.0
        self.mag_heading_offset_valid = False
        self.mag_heading_offset_deg = 0.0
        self.bno_stale_sec = 0.0
        self.phase2_offset_reference_bno_deg = None
        self.phase2_offset_heading_error_deg = 0.0
        self.phase2_offset_turn_target_deg = None
        self.phase3_no_heading_start = None
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
        ctrl.bno_heading_offset_verified = True
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

    def test_unverified_bno_is_not_used_when_gps_course_is_missing(self):
        ctrl = _HeadingOnlyController()
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = False
        ctrl.bno_heading_offset_deg = 120.0

        heading, source = ctrl._phase3_heading(
            {
                "gps_heading_valid": False,
                "angle_valid": True,
                "angle": 30.0,
            }
        )

        self.assertIsNone(heading)
        self.assertEqual(source, "NO_HEADING_SOURCE")

    def test_phase3_large_gps_error_uses_forward_arc_not_pivot(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True

        snapshot = {
            "gps_heading_valid": True,
            "gps_heading": 280.0,
            "gps_fix_seq": 10,
            "angle_valid": False,
            "angle": 0.0,
        }

        _, source, diff = ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(source, "GPS_PRIMARY")
        self.assertAlmostEqual(diff, 170.0)
        self.assertEqual(kwargs["cmd_type"], "phase3_gps_arc")
        self.assertTrue(args[1])
        self.assertTrue(args[3])
        self.assertGreater(args[0], 0.0)
        self.assertGreater(args[2], 0.0)
        self.assertGreater(args[0], args[2])

    def test_phase3_missing_heading_probes_straight_then_stops(self):
        ctrl = _HeadingOnlyController()
        snapshot = {
            "gps_heading_valid": False,
            "angle_valid": True,
            "angle": 30.0,
        }

        ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)
        first_args, first_kwargs = ctrl.motor_commands[-1]
        ctrl._drive_phase3_navigation(snapshot, 90.0, now=14.0)
        _, second_kwargs = ctrl.motor_commands[-1]

        self.assertEqual(first_kwargs["cmd_type"], "phase3_gps_probe")
        self.assertEqual(first_args[0], first_args[2])
        self.assertEqual(second_kwargs["cmd_type"], "stop")

    def test_phase3_gps_loss_stops_even_with_a_heading_source(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = True
        ctrl.bno_heading_offset_deg = 0.0
        snapshot = {
            "gps_detect": 0,
            "gps_heading_valid": False,
            "angle_valid": True,
            "angle": 30.0,
        }

        heading, source, diff = ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)

        _, kwargs = ctrl.motor_commands[-1]
        self.assertIsNone(heading)
        self.assertEqual(source, "GPS_LOST_STOP")
        self.assertIsNone(diff)
        self.assertEqual(kwargs["cmd_type"], "stop")

    def test_phase3_active_gps_keeps_probing_until_course_is_available(self):
        ctrl = _HeadingOnlyController()
        snapshot = {
            "gps_detect": 1,
            "gps_heading_valid": False,
            "angle_valid": False,
            "angle": 0.0,
        }

        ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)
        ctrl._drive_phase3_navigation(snapshot, 90.0, now=20.0)

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(kwargs["cmd_type"], "phase3_gps_probe")
        self.assertEqual(args[0], 70)
        self.assertEqual(args[2], 70)

    def test_phase3_large_verified_bno_error_uses_powered_forward_arc(self):
        ctrl = _HeadingOnlyController()
        ctrl.phase3_heading_entry_ready = True
        ctrl.bno_heading_offset_valid = True
        ctrl.bno_heading_offset_verified = True
        ctrl.bno_heading_offset_deg = 0.0

        snapshot = {
            "gps_heading_valid": False,
            "angle_valid": True,
            "angle": 280.0,
        }

        _, source, diff = ctrl._drive_phase3_navigation(snapshot, 90.0, now=10.0)

        args, kwargs = ctrl.motor_commands[-1]
        self.assertEqual(source, "BNO_ALIGNED")
        self.assertAlmostEqual(diff, 170.0)
        self.assertEqual(kwargs["cmd_type"], "phase3_bno_large_arc")
        # Positive compass-heading error requires a right turn: logical left
        # wheel is faster, but both remain powered to avoid dragging on grass.
        self.assertEqual(args[0], 85)
        self.assertEqual(args[2], 45)
        self.assertTrue(args[1])
        self.assertTrue(args[3])


if __name__ == "__main__":
    unittest.main()
