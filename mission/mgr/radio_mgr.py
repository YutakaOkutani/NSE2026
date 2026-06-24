import os
import subprocess
import time
from pathlib import Path

from mission.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    RADIO_COMMAND_TIMEOUT_SEC,
    RADIO_CONTROL_ENV_KEY,
    RADIO_CONTROL_MISSION_VALUE,
    RADIO_DRY_RUN_ENV_KEY,
    RADIO_MISSION_ENV_PATH_ENV_KEY,
    RADIO_PRE_OFF_DELAY_ENV_KEY,
    RADIO_PRE_OFF_DELAY_SEC,
    RADIO_RESTORE_TIMEOUT_ENV_KEY,
    RADIO_RESTORE_TIMEOUT_SEC,
    RADIO_USE_SUDO_ENV_KEY,
    Phase,
)


class RadioManager:
    def _project_root(self):
        return Path(__file__).resolve().parents[2]

    def _env_flag_enabled(self, key):
        return str(os.getenv(key, "")).strip().lower() in ("1", "true", "yes", "on")

    def _mission_env_path(self):
        override = os.getenv(RADIO_MISSION_ENV_PATH_ENV_KEY)
        if override and override.strip():
            return Path(override).expanduser()
        return self._project_root() / "mission.env"

    def _load_mission_env_file_if_needed(self):
        if RADIO_CONTROL_ENV_KEY in os.environ:
            self.radio_config_source = "process_env"
            return

        path = self._mission_env_path()
        if not path.exists():
            self.radio_config_source = f"missing:{path}"
            return

        loaded_keys = []
        try:
            for raw_line in path.read_text(encoding="utf-8").splitlines():
                line = raw_line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                if not key.startswith("CANSAT_RADIO_"):
                    continue
                if key not in os.environ:
                    os.environ[key] = value
                    loaded_keys.append(key)
        except Exception as exc:
            self.radio_config_source = f"load_failed:{path}:{exc}"
            return

        if loaded_keys:
            self.radio_config_source = f"mission_env:{path}"
        else:
            self.radio_config_source = f"mission_env_empty:{path}"

    def _read_env_float(self, key, default_value):
        raw = os.getenv(key)
        if raw is None or str(raw).strip() == "":
            return float(default_value)
        try:
            return max(0.0, float(raw))
        except (TypeError, ValueError):
            print(f"Radio: invalid {key}={raw!r}; using {default_value}")
            return float(default_value)

    def radio_control_enabled(self):
        mode = str(os.getenv(RADIO_CONTROL_ENV_KEY, "")).strip().lower()
        return mode == RADIO_CONTROL_MISSION_VALUE

    def _radio_command_prefix(self):
        if self._env_flag_enabled(RADIO_USE_SUDO_ENV_KEY):
            return ["sudo", "-n"]
        return []

    def _run_radio_command(self, args):
        command = self._radio_command_prefix() + list(args)
        command_text = " ".join(command)
        if self._env_flag_enabled(RADIO_DRY_RUN_ENV_KEY):
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
        self._load_mission_env_file_if_needed()
        self.radio_control_mode = str(os.getenv(RADIO_CONTROL_ENV_KEY, "")).strip().lower()
        self.radio_disabled = False
        self.radio_disable_time = None
        self.radio_restore_deadline = None
        self.radio_last_event = f"disabled_by_config:mode={self.radio_control_mode or 'unset'}"

        if not self.radio_control_enabled():
            print(
                f"Radio: mission control disabled ({RADIO_CONTROL_ENV_KEY}={self.radio_control_mode or 'unset'}; "
                f"source={getattr(self, 'radio_config_source', '')})"
            )
            return

        phase = Phase(start_phase)
        if phase != Phase.PHASE0:
            self.radio_last_event = "skipped_non_phase0_start"
            print(f"Radio: mission control skipped for {phase.name} start")
            return

        delay_sec = self._read_env_float(RADIO_PRE_OFF_DELAY_ENV_KEY, RADIO_PRE_OFF_DELAY_SEC)
        restore_timeout = self._read_env_float(RADIO_RESTORE_TIMEOUT_ENV_KEY, RADIO_RESTORE_TIMEOUT_SEC)
        print(
            "Radio: mission control armed; "
            f"Wi-Fi off after {delay_sec:.1f}s, failsafe restore after {restore_timeout:.1f}s "
            f"(source={getattr(self, 'radio_config_source', '')})"
        )
        self._signal_radio_shutdown_pending(delay_sec)
        self.disable_mission_radio("phase0_start")
        if self.radio_disabled:
            self.radio_restore_deadline = time.time() + restore_timeout

    def disable_mission_radio(self, reason):
        if not self.radio_control_enabled():
            return False
        if getattr(self, "radio_disabled", False):
            return True
        print(f"Radio: disabling Wi-Fi ({reason})")
        if not self._run_radio_command(["rfkill", "block", "wifi"]):
            self.radio_last_event = "disable_failed"
            return False
        self.radio_disabled = True
        self.radio_disable_time = time.time()
        self.radio_last_event = f"disabled:{reason}"
        return True

    def restore_mission_radio(self, reason):
        if not self.radio_control_enabled():
            return False
        if not getattr(self, "radio_disabled", False):
            return False
        print(f"Radio: restoring Wi-Fi ({reason})")
        if not self._run_radio_command(["rfkill", "unblock", "wifi"]):
            self.radio_last_event = "restore_failed"
            return False
        self.radio_disabled = False
        self.radio_restore_deadline = None
        self.radio_last_event = f"restored:{reason}"
        return True

    def check_radio_failsafe(self):
        if not getattr(self, "radio_disabled", False):
            return False
        deadline = getattr(self, "radio_restore_deadline", None)
        if deadline is None or time.time() < deadline:
            return False
        return self.restore_mission_radio("failsafe_timeout")
