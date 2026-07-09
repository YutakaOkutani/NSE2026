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


if __name__ == "__main__":
    unittest.main()
