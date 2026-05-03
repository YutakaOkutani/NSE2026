import argparse

from csmn.run import run_full_mission


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
    return parser.parse_args()


def main():
    args = parse_args()
    run_full_mission(
        target_lat=args.target_lat,
        target_lng=args.target_lng,
        machine_name=args.machine,
        log_dir=args.log_dir,
    )


if __name__ == "__main__":
    main()
