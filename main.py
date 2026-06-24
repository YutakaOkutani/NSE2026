import argparse
import os

from mission.const import (
    RADIO_CONTROL_ENV_KEY,
    RADIO_DRY_RUN_ENV_KEY,
    RADIO_PRE_OFF_DELAY_ENV_KEY,
    RADIO_RESTORE_TIMEOUT_ENV_KEY,
    RADIO_USE_SUDO_ENV_KEY,
)
from mission.run import run_full_mission


def parse_args():
    parser = argparse.ArgumentParser(description="CanSat mission runner")
    parser.add_argument(
        "--machine",
        default=None,
        help="Optional machine override (common / unit1 / unit2). Auto-detects when omitted.",
    )
    parser.add_argument("--target-lat", type=float, default=None, help="Override target latitude. Falls back to CANSAT_TARGET_LAT.")
    parser.add_argument("--target-lng", type=float, default=None, help="Override target longitude. Falls back to CANSAT_TARGET_LNG.")
    parser.add_argument("--log-dir", default=None, help="Optional log directory override. Falls back to CANSAT_LOG_DIR.")
    parser.add_argument(
        "--radio-control",
        choices=("off", "mission"),
        default=None,
        help="Override Wi-Fi mission control. 'mission' enables Phase0 radio-off behavior even outside systemd.",
    )
    parser.add_argument(
        "--radio-restore-timeout-sec",
        type=float,
        default=None,
        help="Override Wi-Fi restore failsafe seconds after radio-off.",
    )
    parser.add_argument(
        "--radio-pre-off-delay-sec",
        type=float,
        default=None,
        help="Override LED warning delay before Wi-Fi is turned off.",
    )
    parser.add_argument(
        "--radio-dry-run",
        action="store_true",
        help="Print radio commands without executing them.",
    )
    parser.add_argument(
        "--radio-use-sudo",
        action="store_true",
        help="Run rfkill commands through sudo -n.",
    )
    return parser.parse_args()


def apply_radio_arg_overrides(args):
    if args.radio_control is not None:
        os.environ[RADIO_CONTROL_ENV_KEY] = args.radio_control
    if args.radio_restore_timeout_sec is not None:
        os.environ[RADIO_RESTORE_TIMEOUT_ENV_KEY] = str(args.radio_restore_timeout_sec)
    if args.radio_pre_off_delay_sec is not None:
        os.environ[RADIO_PRE_OFF_DELAY_ENV_KEY] = str(args.radio_pre_off_delay_sec)
    if args.radio_dry_run:
        os.environ[RADIO_DRY_RUN_ENV_KEY] = "1"
    if args.radio_use_sudo:
        os.environ[RADIO_USE_SUDO_ENV_KEY] = "1"


def main():
    args = parse_args()
    apply_radio_arg_overrides(args)
    run_full_mission(
        target_lat=args.target_lat,
        target_lng=args.target_lng,
        machine_name=args.machine,
        log_dir=args.log_dir,
    )


if __name__ == "__main__":
    main()
