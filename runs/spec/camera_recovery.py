import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

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
spec = importlib.util.spec_from_file_location("camera_recovery_under_test", SNS_MGR_PATH)
sns_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sns_mgr_under_test)
SensorManager = sns_mgr_under_test.SensorManager


class _Detector:
    def set_roi_img(self, _roi):
        pass


class _CameraRecoveryController(SensorManager):
    def __init__(self):
        self.devices = {"detector": None}
        self.roi_img = None
        self.roi_references = []
        self.camera_fail_count = 5
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_recovery_started_at = None
        self.camera_reinit_attempt_count = 0
        self.camera_recovery_exhausted = False

    def _release_camera_detector(self):
        self.devices["detector"] = None


class CameraRecoveryTest(unittest.TestCase):
    def test_fresh_phase4_window_clears_stale_exhaustion(self):
        ctrl = _CameraRecoveryController()
        ctrl.camera_fail_count = 9
        ctrl.camera_last_reinit = 110.0
        ctrl.camera_dead_since = 90.0
        ctrl.camera_recovery_started_at = 90.0
        ctrl.camera_reinit_attempt_count = 3
        ctrl.camera_recovery_exhausted = True

        ctrl.reset_camera_recovery_window()

        self.assertEqual(ctrl.camera_fail_count, 0)
        self.assertEqual(ctrl.camera_last_reinit, 0.0)
        self.assertIsNone(ctrl.camera_dead_since)
        self.assertIsNone(ctrl.camera_recovery_started_at)
        self.assertEqual(ctrl.camera_reinit_attempt_count, 0)
        self.assertFalse(ctrl.camera_recovery_exhausted)

    def test_recovery_is_bounded_to_three_attempts_and_fifteen_seconds(self):
        ctrl = _CameraRecoveryController()

        with patch.object(sns_mgr_under_test.dc, "detector", side_effect=_Detector, create=True):
            ctrl._begin_camera_recovery(100.0)
            with patch.object(sns_mgr_under_test.time, "time", return_value=100.0):
                self.assertTrue(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=104.0):
                self.assertFalse(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=105.0):
                self.assertTrue(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=110.0):
                self.assertTrue(ctrl._try_reinit_camera())
            with patch.object(sns_mgr_under_test.time, "time", return_value=115.0):
                self.assertFalse(ctrl._try_reinit_camera())

        self.assertEqual(ctrl.camera_reinit_attempt_count, 3)
        self.assertTrue(ctrl.camera_recovery_exhausted)

    def test_detector_recreation_is_not_a_recovery_until_a_valid_frame(self):
        ctrl = _CameraRecoveryController()
        ctrl._begin_camera_recovery(100.0)

        with patch.object(sns_mgr_under_test.dc, "detector", side_effect=_Detector, create=True):
            with patch.object(sns_mgr_under_test.time, "time", return_value=100.0):
                ctrl._try_reinit_camera()

        self.assertIsNotNone(ctrl.camera_dead_since)
        self.assertEqual(ctrl.camera_reinit_attempt_count, 1)

        ctrl._record_camera_recovered()

        self.assertIsNone(ctrl.camera_dead_since)
        self.assertIsNone(ctrl.camera_recovery_started_at)
        self.assertEqual(ctrl.camera_reinit_attempt_count, 0)
        self.assertFalse(ctrl.camera_recovery_exhausted)


if __name__ == "__main__":
    unittest.main()
