import sys
import types
import unittest
import importlib.util
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import LOG_HEADER
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
spec = importlib.util.spec_from_file_location("sns_mgr_under_test", SNS_MGR_PATH)
sns_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sns_mgr_under_test)
SensorManager = sns_mgr_under_test.SensorManager


class _LogOnlyController(SensorManager):
    def __init__(self):
        self.st = CanSatState()
        self.machine_name = "test"
        self.target_lat = 0.0
        self.target_lng = 0.0
        self.mission_start_time = 100.0
        self.last_motor_command = {}
        self.phase7_arrival_reason = "RUNNING"
        self.mission_end_reason = "RUNNING"
        self.mission_total_timeout_triggered = False
        self.radio_disabled = True
        self.radio_control_mode = "mission"
        self.radio_last_event = "disabled:phase0_start"
        self.radio_config_source = "process_env"
        self.radio_restore_deadline = 130.0


class LogSchemaTest(unittest.TestCase):
    def test_log_header_and_row_lengths_match(self):
        ctrl = _LogOnlyController()

        row = ctrl._build_log_row()

        self.assertEqual(len(row), len(LOG_HEADER))

    def test_radio_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["RadioDisabled"], 1)
        self.assertEqual(by_name["RadioControlMode"], "mission")
        self.assertEqual(by_name["RadioLastEvent"], "disabled:phase0_start")
        self.assertEqual(by_name["RadioConfigSource"], "process_env")
        self.assertEqual(by_name["RadioRestoreDeadlineElapsedSec"], "30.00")

    def test_sensor_update_elapsed_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()
        ctrl.bno_last_acc_time = 101.25
        ctrl.bmp_last_valid_time = 102.5

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["BNOAccUpdatedElapsedSec"], "1.25")
        self.assertEqual(by_name["BMPUpdatedElapsedSec"], "2.50")

    def test_phase0_exit_columns_are_written_to_log_row(self):
        ctrl = _LogOnlyController()
        ctrl.phase0_exit_reason = "impact"
        ctrl.phase0_exit_detail = "delta=8.20"

        row = ctrl._build_log_row()
        by_name = dict(zip(LOG_HEADER, row))

        self.assertEqual(by_name["Phase0ExitReason"], "impact")
        self.assertEqual(by_name["Phase0ExitDetail"], "delta=8.20")


if __name__ == "__main__":
    unittest.main()
