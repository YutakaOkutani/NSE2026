import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

MOTOR_DIAG_PATH = PROJECT_ROOT / "runs" / "diag" / "motor.py"
spec = importlib.util.spec_from_file_location("motor_diag_under_test", MOTOR_DIAG_PATH)
motor_diag = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = motor_diag
spec.loader.exec_module(motor_diag)


class MotorDiagnosticSafetyTest(unittest.TestCase):
    def test_quit_always_stops_motors(self):
        args = types.SimpleNamespace(machine=None, default_speed=50.0)
        with patch.object(motor_diag, "parse_args", return_value=args):
            with patch.object(motor_diag, "activate_machine_profile"):
                with patch.object(motor_diag, "setup"):
                    with patch.object(motor_diag, "_get_command", return_value="q"):
                        with patch.object(motor_diag, "stop") as stop:
                            motor_diag.main()
        stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
