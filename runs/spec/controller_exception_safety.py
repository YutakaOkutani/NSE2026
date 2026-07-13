import importlib.util
import sys
import types
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import Phase


def _load_controller_class_without_hardware_dependencies():
    manager_module = types.ModuleType("mission.mgr")
    for name in ("HardwareManager", "LedManager", "MotorManager", "RadioManager", "SensorManager"):
        setattr(manager_module, name, type(name, (), {}))

    phases_module = types.ModuleType("mission.phases")
    for name in (
        "Phase0Handler",
        "Phase1Handler",
        "Phase2Handler",
        "Phase3Handler",
        "Phase4Handler",
        "Phase5Handler",
        "Phase6Handler",
        "Phase7Handler",
    ):
        setattr(phases_module, name, type(name, (), {}))

    state_module = types.ModuleType("mission.st")
    state_module.CanSatState = type("CanSatState", (), {})

    ctrl_path = PROJECT_ROOT / "mission" / "ctrl.py"
    spec = importlib.util.spec_from_file_location("controller_under_test", ctrl_path)
    module = importlib.util.module_from_spec(spec)
    original_modules = {
        name: sys.modules.get(name)
        for name in ("mission.mgr", "mission.phases", "mission.st")
    }
    try:
        sys.modules["mission.mgr"] = manager_module
        sys.modules["mission.phases"] = phases_module
        sys.modules["mission.st"] = state_module
        spec.loader.exec_module(module)
    finally:
        for name, original in original_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original
    return module.CanSatController


CanSatController = _load_controller_class_without_hardware_dependencies()


class _RunHarness:
    run = CanSatController.run

    def __init__(self, failure_stage):
        self.failure_stage = failure_stage
        self._shutdown_requested = False
        self.shutdown_reasons = []

    def setup_hardware(self):
        if self.failure_stage == "setup":
            raise RuntimeError("setup failed")

    def signal_led(self, _count):
        if self.failure_stage == "led":
            raise RuntimeError("led failed")

    def prepare_mission_radio_control(self, _start_phase):
        if self.failure_stage == "radio":
            raise RuntimeError("radio failed")

    def initialize_phase(self, _phase):
        if self.failure_stage == "phase":
            raise RuntimeError("phase failed")

    def request_shutdown(self, reason):
        self._shutdown_requested = True
        self.shutdown_reasons.append(reason)


class ControllerExceptionSafetyTest(unittest.TestCase):
    def test_startup_failures_always_request_shutdown(self):
        for stage in ("setup", "led", "radio", "phase"):
            with self.subTest(stage=stage):
                harness = _RunHarness(stage)
                with self.assertRaisesRegex(RuntimeError, f"{stage} failed"):
                    harness.run(start_phase=Phase.PHASE0)
                self.assertEqual(harness.shutdown_reasons, ["RUN_EXIT"])


if __name__ == "__main__":
    unittest.main()
