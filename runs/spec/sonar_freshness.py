import importlib.util
import sys
import types
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import DEFAULT_OBSTACLE_DIST_CM, SONAR_STALE_TIMEOUT_SEC
from mission.st import CanSatState

sys.modules.setdefault("serial", types.SimpleNamespace())
sys.modules.setdefault("lib.bno055", types.SimpleNamespace())
sys.modules.setdefault("lib.cone_detect", types.SimpleNamespace())
sys.modules.setdefault(
    "mission.gps_util",
    types.SimpleNamespace(
        coerce_gga_metrics=lambda *_args, **_kwargs: None,
        gga_quality_ok=lambda *_args, **_kwargs: False,
        open_gps_serial=lambda *_args, **_kwargs: None,
        parse_gga_sentence=lambda *_args, **_kwargs: None,
    ),
)

SNS_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "sns_mgr.py"
spec = importlib.util.spec_from_file_location("sonar_sensor_manager_under_test", SNS_MGR_PATH)
sensor_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor_module)


class _SonarController(sensor_module.SensorManager):
    def __init__(self):
        self.st = CanSatState()


class SonarFreshnessTest(unittest.TestCase):
    def test_recent_value_is_kept_only_within_stale_window(self):
        ctrl = _SonarController()
        self.assertTrue(ctrl._update_sonar_state(25.0, now=100.0))
        self.assertFalse(ctrl._update_sonar_state(None, now=100.5))

        snapshot = ctrl.st.snapshot()
        self.assertTrue(snapshot["obstacle_valid"])
        self.assertEqual(snapshot["obstacle_dist"], 25.0)
        self.assertEqual(snapshot["obstacle_stale_sec"], 0.5)

    def test_stale_value_is_invalidated_and_replaced(self):
        ctrl = _SonarController()
        ctrl._update_sonar_state(18.0, now=100.0)
        ctrl._update_sonar_state(None, now=100.0 + SONAR_STALE_TIMEOUT_SEC)

        snapshot = ctrl.st.snapshot()
        self.assertFalse(snapshot["obstacle_valid"])
        self.assertEqual(snapshot["obstacle_dist"], DEFAULT_OBSTACLE_DIST_CM)
        self.assertEqual(snapshot["obstacle_stale_sec"], SONAR_STALE_TIMEOUT_SEC)

    def test_invalid_numeric_samples_do_not_become_obstacles(self):
        ctrl = _SonarController()
        for value in (float("nan"), float("inf"), -1.0, 0.0, 9999.0):
            with self.subTest(value=value):
                self.assertFalse(ctrl._update_sonar_state(value, now=10.0))
                self.assertFalse(ctrl.st.snapshot()["obstacle_valid"])

    def test_valid_sample_recovers_after_stale_period(self):
        ctrl = _SonarController()
        ctrl._update_sonar_state(None, now=10.0)
        self.assertTrue(ctrl._update_sonar_state(42.0, now=20.0))

        snapshot = ctrl.st.snapshot()
        self.assertTrue(snapshot["obstacle_valid"])
        self.assertEqual(snapshot["obstacle_dist"], 42.0)
        self.assertEqual(snapshot["obstacle_stale_sec"], 0.0)


if __name__ == "__main__":
    unittest.main()
