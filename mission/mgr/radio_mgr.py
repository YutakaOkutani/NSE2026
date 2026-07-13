import subprocess
import time

from mission.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    RADIO_COMMAND_TIMEOUT_SEC,
    Phase,
)


class RadioManager:
    def radio_control_enabled(self):
        return self.radio_config.control == "mission"

    def _radio_command_prefix(self):
        if self.radio_config.use_sudo:
            return ["sudo", "-n"]
        return []

    def _run_radio_command(self, args):
        command = self._radio_command_prefix() + list(args)
        command_text = " ".join(command)
        if self.radio_config.dry_run:
            print(f"Radio: DRY RUN {command_text}")
            return True

        runner = getattr(self, "radio_command_runner", None)
        if runner is None:
            runner = subprocess.run

        try:
            result = runner(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=RADIO_COMMAND_TIMEOUT_SEC,
            )
        except TypeError:
            result = runner(command)
        except Exception as exc:
            print(f"Radio: command failed to start ({command_text}): {exc}")
            return False

        returncode = int(getattr(result, "returncode", 0))
        if returncode == 0:
            return True
        stderr = str(getattr(result, "stderr", "") or "").strip()
        stdout = str(getattr(result, "stdout", "") or "").strip()
        detail = stderr or stdout or f"returncode={returncode}"
        print(f"Radio: command failed ({command_text}): {detail}")
        return False

    def _signal_radio_shutdown_pending(self, delay_sec):
        led_red = self.devices.get(DEVICE_LED_RED)
        led_green = self.devices.get(DEVICE_LED_GREEN)
        end_time = time.time() + max(0.0, float(delay_sec))
        tick = 0
        while time.time() < end_time:
            red_on = (tick % 2) == 0
            if led_red:
                led_red.on() if red_on else led_red.off()
            if led_green:
                led_green.off() if red_on else led_green.on()
            tick += 1
            time.sleep(0.25)
        if led_red:
            led_red.off()
        if led_green:
            led_green.off()

    def prepare_mission_radio_control(self, start_phase):
        self.radio_control_mode = self.radio_config.control
        self.radio_disabled = False
        self.radio_simulated_disabled = False
        self.radio_disable_time = None
        self.radio_restore_deadline = None
        self.radio_last_event = f"disabled_by_config:mode={self.radio_control_mode or 'unset'}"

        if not self.radio_control_enabled():
            print(
                f"Radio: mission control disabled (mode={self.radio_control_mode}; "
                f"source={getattr(self, 'radio_config_source', '')})"
            )
            return

        phase = Phase(start_phase)
        if phase != Phase.PHASE0:
            self.radio_last_event = "skipped_non_phase0_start"
            print(f"Radio: mission control skipped for {phase.name} start")
            return

        delay_sec = self.radio_config.pre_off_delay_sec
        restore_timeout = self.radio_config.restore_timeout_sec
        print(
            "Radio: mission control armed; "
            f"Wi-Fi off after {delay_sec:.1f}s, failsafe restore after {restore_timeout:.1f}s "
            f"(source={getattr(self, 'radio_config_source', '')})"
        )
        self._signal_radio_shutdown_pending(delay_sec)
        self.disable_mission_radio("phase0_start")
        if self.radio_disabled or self.radio_simulated_disabled:
            self.radio_restore_deadline = time.time() + restore_timeout

    def disable_mission_radio(self, reason):
        if not self.radio_control_enabled():
            return False
        if getattr(self, "radio_disabled", False) or getattr(self, "radio_simulated_disabled", False):
            return True
        print(f"Radio: disabling Wi-Fi ({reason})")
        if not self._run_radio_command(["rfkill", "block", "wifi"]):
            self.radio_last_event = "disable_failed"
            return False
        if self.radio_config.dry_run:
            self.radio_simulated_disabled = True
            self.radio_last_event = f"dry_run_disabled:{reason}"
            return True
        self.radio_disabled = True
        self.radio_disable_time = time.time()
        self.radio_last_event = f"disabled:{reason}"
        return True

    def restore_mission_radio(self, reason):
        if not self.radio_control_enabled():
            return False
        if not (
            getattr(self, "radio_disabled", False)
            or getattr(self, "radio_simulated_disabled", False)
        ):
            return False
        print(f"Radio: restoring Wi-Fi ({reason})")
        if not self._run_radio_command(["rfkill", "unblock", "wifi"]):
            self.radio_last_event = "restore_failed"
            return False
        was_dry_run = bool(getattr(self, "radio_simulated_disabled", False))
        self.radio_disabled = False
        self.radio_simulated_disabled = False
        self.radio_restore_deadline = None
        event = "dry_run_restored" if was_dry_run else "restored"
        self.radio_last_event = f"{event}:{reason}"
        return True

    def check_radio_failsafe(self):
        if not (
            getattr(self, "radio_disabled", False)
            or getattr(self, "radio_simulated_disabled", False)
        ):
            return False
        deadline = getattr(self, "radio_restore_deadline", None)
        if deadline is None or time.time() < deadline:
            return False
        return self.restore_mission_radio("failsafe_timeout")
