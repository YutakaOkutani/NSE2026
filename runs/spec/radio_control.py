import os
import sys
import tempfile
import time
import unittest
import importlib.util
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    RADIO_CONTROL_ENV_KEY,
    RADIO_DRY_RUN_ENV_KEY,
    RADIO_MISSION_ENV_PATH_ENV_KEY,
    RADIO_PRE_OFF_DELAY_ENV_KEY,
    RADIO_RESTORE_TIMEOUT_ENV_KEY,
    Phase,
)
RADIO_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "radio_mgr.py"
spec = importlib.util.spec_from_file_location("radio_mgr_under_test", RADIO_MGR_PATH)
radio_mgr_under_test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(radio_mgr_under_test)
RadioManager = radio_mgr_under_test.RadioManager


class _Result:
    returncode = 0
    stdout = ""
    stderr = ""


class _FakeLed:
    def __init__(self):
        self.events = []

    def on(self):
        self.events.append("on")

    def off(self):
        self.events.append("off")


class _FakeRadioController(RadioManager):
    def __init__(self):
        self.commands = []
        self.devices = {
            DEVICE_LED_RED: _FakeLed(),
            DEVICE_LED_GREEN: _FakeLed(),
        }

    def radio_command_runner(self, command, **_kwargs):
        self.commands.append(list(command))
        return _Result()


class RadioControlTest(unittest.TestCase):
    def setUp(self):
        self.env_patch = patch.dict(os.environ, {}, clear=False)
        self.env_patch.start()
        for key in (
            RADIO_CONTROL_ENV_KEY,
            RADIO_PRE_OFF_DELAY_ENV_KEY,
            RADIO_RESTORE_TIMEOUT_ENV_KEY,
            RADIO_DRY_RUN_ENV_KEY,
            RADIO_MISSION_ENV_PATH_ENV_KEY,
            "CANSAT_RADIO_USE_SUDO",
            "INVOCATION_ID",
            "JOURNAL_STREAM",
        ):
            os.environ.pop(key, None)
        self.ctrl = _FakeRadioController()

    def tearDown(self):
        self.env_patch.stop()

    def test_disabled_by_default(self):
        self.ctrl.prepare_mission_radio_control(Phase.PHASE0)

        self.assertFalse(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands, [])

    def test_mission_mode_blocks_wifi_for_phase0(self):
        os.environ[RADIO_CONTROL_ENV_KEY] = "mission"
        os.environ[RADIO_PRE_OFF_DELAY_ENV_KEY] = "0"
        os.environ[RADIO_RESTORE_TIMEOUT_ENV_KEY] = "30"

        self.ctrl.prepare_mission_radio_control(Phase.PHASE0)

        self.assertTrue(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands, [["rfkill", "block", "wifi"]])
        self.assertIsNotNone(self.ctrl.radio_restore_deadline)

    def test_diag_style_direct_disable_uses_process_env(self):
        os.environ[RADIO_CONTROL_ENV_KEY] = "mission"

        disabled = self.ctrl.disable_mission_radio("diag_manual")

        self.assertTrue(disabled)
        self.assertTrue(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands, [["rfkill", "block", "wifi"]])

    def test_manual_run_loads_mission_env_when_process_env_missing(self):
        with tempfile.NamedTemporaryFile("w", delete=False) as file_obj:
            file_obj.write("CANSAT_RADIO_CONTROL=mission\n")
            file_obj.write("CANSAT_RADIO_PRE_OFF_DELAY_SEC=0\n")
            file_obj.write("CANSAT_RADIO_RESTORE_TIMEOUT_SEC=30\n")
            env_path = file_obj.name
        self.addCleanup(lambda: Path(env_path).unlink(missing_ok=True))
        os.environ[RADIO_MISSION_ENV_PATH_ENV_KEY] = env_path

        self.ctrl.prepare_mission_radio_control(Phase.PHASE0)

        self.assertEqual(self.ctrl.radio_control_mode, "mission")
        self.assertTrue(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands, [["rfkill", "block", "wifi"]])
        self.assertTrue(self.ctrl.radio_config_source.startswith("mission_env:"))

    def test_mission_env_off_keeps_radio_enabled(self):
        with tempfile.NamedTemporaryFile("w", delete=False) as file_obj:
            file_obj.write("CANSAT_RADIO_CONTROL=off\n")
            env_path = file_obj.name
        self.addCleanup(lambda: Path(env_path).unlink(missing_ok=True))
        os.environ[RADIO_MISSION_ENV_PATH_ENV_KEY] = env_path

        self.ctrl.prepare_mission_radio_control(Phase.PHASE0)

        self.assertFalse(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands, [])
        self.assertEqual(self.ctrl.radio_control_mode, "off")
        self.assertTrue(self.ctrl.radio_config_source.startswith("mission_env:"))

    def test_non_phase0_start_does_not_block_wifi(self):
        os.environ[RADIO_CONTROL_ENV_KEY] = "mission"

        self.ctrl.prepare_mission_radio_control(Phase.PHASE3)

        self.assertFalse(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands, [])

    def test_restore_unblocks_wifi_once(self):
        os.environ[RADIO_CONTROL_ENV_KEY] = "mission"
        os.environ[RADIO_PRE_OFF_DELAY_ENV_KEY] = "0"
        self.ctrl.prepare_mission_radio_control(Phase.PHASE0)

        restored = self.ctrl.restore_mission_radio("test")
        restored_again = self.ctrl.restore_mission_radio("test_again")

        self.assertTrue(restored)
        self.assertFalse(restored_again)
        self.assertEqual(
            self.ctrl.commands,
            [["rfkill", "block", "wifi"], ["rfkill", "unblock", "wifi"]],
        )

    def test_failsafe_restores_after_deadline(self):
        os.environ[RADIO_CONTROL_ENV_KEY] = "mission"
        os.environ[RADIO_PRE_OFF_DELAY_ENV_KEY] = "0"
        os.environ[RADIO_RESTORE_TIMEOUT_ENV_KEY] = "100"
        self.ctrl.prepare_mission_radio_control(Phase.PHASE0)
        self.ctrl.radio_restore_deadline = time.time() - 0.1

        restored = self.ctrl.check_radio_failsafe()

        self.assertTrue(restored)
        self.assertFalse(self.ctrl.radio_disabled)
        self.assertEqual(self.ctrl.commands[-1], ["rfkill", "unblock", "wifi"])


if __name__ == "__main__":
    unittest.main()
