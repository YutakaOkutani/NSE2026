import importlib.util
import sys
import time
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.config import RadioConfig
from mission.const import DEVICE_LED_GREEN, DEVICE_LED_RED, Phase

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
    def __init__(self, radio_config):
        self.radio_config = radio_config
        self.radio_config_source = "/test/mission.toml"
        self.commands = []
        self.devices = {
            DEVICE_LED_RED: _FakeLed(),
            DEVICE_LED_GREEN: _FakeLed(),
        }

    def radio_command_runner(self, command, **_kwargs):
        self.commands.append(list(command))
        return _Result()


def _radio_config(
    control="off",
    pre_off_delay_sec=0.0,
    restore_timeout_sec=30.0,
    use_sudo=False,
    dry_run=False,
):
    return RadioConfig(
        control=control,
        pre_off_delay_sec=pre_off_delay_sec,
        restore_timeout_sec=restore_timeout_sec,
        use_sudo=use_sudo,
        dry_run=dry_run,
    )


class RadioControlTest(unittest.TestCase):
    def test_off_mode_keeps_wifi_enabled(self):
        ctrl = _FakeRadioController(_radio_config())
        ctrl.prepare_mission_radio_control(Phase.PHASE0)
        self.assertFalse(ctrl.radio_disabled)
        self.assertEqual(ctrl.commands, [])

    def test_mission_mode_blocks_wifi_for_phase0(self):
        ctrl = _FakeRadioController(_radio_config(control="mission"))
        ctrl.prepare_mission_radio_control(Phase.PHASE0)
        self.assertTrue(ctrl.radio_disabled)
        self.assertEqual(ctrl.commands, [["rfkill", "block", "wifi"]])
        self.assertIsNotNone(ctrl.radio_restore_deadline)

    def test_non_phase0_start_does_not_block_wifi(self):
        ctrl = _FakeRadioController(_radio_config(control="mission"))
        ctrl.prepare_mission_radio_control(Phase.PHASE3)
        self.assertFalse(ctrl.radio_disabled)
        self.assertEqual(ctrl.commands, [])

    def test_restore_unblocks_wifi_once(self):
        ctrl = _FakeRadioController(_radio_config(control="mission"))
        ctrl.prepare_mission_radio_control(Phase.PHASE0)
        self.assertTrue(ctrl.restore_mission_radio("test"))
        self.assertFalse(ctrl.restore_mission_radio("test_again"))
        self.assertEqual(
            ctrl.commands,
            [["rfkill", "block", "wifi"], ["rfkill", "unblock", "wifi"]],
        )

    def test_failsafe_restores_after_deadline(self):
        ctrl = _FakeRadioController(_radio_config(control="mission"))
        ctrl.prepare_mission_radio_control(Phase.PHASE0)
        ctrl.radio_restore_deadline = time.time() - 0.1
        self.assertTrue(ctrl.check_radio_failsafe())
        self.assertFalse(ctrl.radio_disabled)
        self.assertEqual(ctrl.commands[-1], ["rfkill", "unblock", "wifi"])

    def test_use_sudo_prefixes_command(self):
        ctrl = _FakeRadioController(_radio_config(control="mission", use_sudo=True))
        ctrl.prepare_mission_radio_control(Phase.PHASE0)
        self.assertEqual(ctrl.commands, [["sudo", "-n", "rfkill", "block", "wifi"]])

    def test_dry_run_does_not_execute_command(self):
        ctrl = _FakeRadioController(_radio_config(control="mission", dry_run=True))
        ctrl.prepare_mission_radio_control(Phase.PHASE0)
        self.assertFalse(ctrl.radio_disabled)
        self.assertTrue(ctrl.radio_simulated_disabled)
        self.assertEqual(ctrl.commands, [])
        self.assertTrue(ctrl.restore_mission_radio("dry_run_test"))
        self.assertFalse(ctrl.radio_simulated_disabled)
        self.assertEqual(ctrl.radio_last_event, "dry_run_restored:dry_run_test")


if __name__ == "__main__":
    unittest.main()
