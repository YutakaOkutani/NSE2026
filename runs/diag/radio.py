import argparse
import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    RADIO_CONTROL_ENV_KEY,
    RADIO_DRY_RUN_ENV_KEY,
    RADIO_RESTORE_TIMEOUT_ENV_KEY,
    RADIO_USE_SUDO_ENV_KEY,
)

RADIO_MGR_PATH = PROJECT_ROOT / "mission" / "mgr" / "radio_mgr.py"
spec = importlib.util.spec_from_file_location("radio_mgr_diag", RADIO_MGR_PATH)
radio_mgr_diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(radio_mgr_diag)
RadioManager = radio_mgr_diag.RadioManager


class RadioDiag(RadioManager):
    def __init__(self):
        self.devices = {}
        self.radio_disabled = False
        self.radio_disable_time = None
        self.radio_restore_deadline = None
        self.radio_last_event = "diag_start"
        self.radio_control_mode = ""
        self.radio_config_source = "diag_cli"


def print_rfkill_status():
    try:
        result = subprocess.run(
            ["rfkill", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except Exception as exc:
        print(f"rfkill status unavailable: {exc}")
        return
    text = (result.stdout or result.stderr or "").strip()
    if text:
        print(text)


def parse_args():
    parser = argparse.ArgumentParser(description="Wi-Fi radio off/on diagnostic")
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Seconds to keep Wi-Fi off before restoring it.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print rfkill commands without executing them.",
    )
    parser.add_argument(
        "--use-sudo",
        action="store_true",
        help="Run rfkill commands through sudo -n.",
    )
    parser.add_argument(
        "--status-only",
        action="store_true",
        help="Only print rfkill status.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.status_only:
        print_rfkill_status()
        return

    os.environ[RADIO_CONTROL_ENV_KEY] = "mission"
    os.environ[RADIO_RESTORE_TIMEOUT_ENV_KEY] = str(max(0.0, args.duration))
    if args.dry_run:
        os.environ[RADIO_DRY_RUN_ENV_KEY] = "1"
    if args.use_sudo:
        os.environ[RADIO_USE_SUDO_ENV_KEY] = "1"

    diag = RadioDiag()
    print("--- before ---")
    print_rfkill_status()

    if not diag.disable_mission_radio("diag_manual"):
        print(f"Radio disable failed: {diag.radio_last_event}")
        raise SystemExit(1)

    print(f"Wi-Fi should be off for {args.duration:.1f}s. SSH/ping should drop if this is not dry-run.")
    deadline = time.time() + max(0.0, args.duration)
    try:
        while time.time() < deadline:
            time.sleep(min(1.0, max(0.0, deadline - time.time())))
    finally:
        diag.restore_mission_radio("diag_manual_restore")

    print("--- after ---")
    print_rfkill_status()
    print(f"Radio diag finished: {diag.radio_last_event}")


if __name__ == "__main__":
    main()
