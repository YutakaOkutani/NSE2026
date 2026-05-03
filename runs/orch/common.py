import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
TEST_LOG_ROOT = PROJECT_ROOT / "runs" / "log"
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from csmn.profile import build_debug_log_dir, list_profiles, resolve_machine_profile


def build_phase_runner_parser(description: str, default_label: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--machine", default=None, choices=list_profiles())
    parser.add_argument("--debug-scope", default="machine", choices=("machine", "shared"))
    parser.add_argument("--debug-label", default=default_label)
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--target-lat", type=float, default=None)
    parser.add_argument("--target-lng", type=float, default=None)
    return parser


def resolve_runtime_args(args):
    resolution = resolve_machine_profile(args.machine)
    if args.log_dir:
        log_dir = str(Path(args.log_dir))
    else:
        log_dir = build_debug_log_dir(
            base_root=TEST_LOG_ROOT,
            profile_name=resolution.name,
            scope=args.debug_scope,
            label=args.debug_label,
        )
    return {
        "machine_name": resolution.name,
        "target_lat": args.target_lat,
        "target_lng": args.target_lng,
        "log_dir": log_dir,
    }
