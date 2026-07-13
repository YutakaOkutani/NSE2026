import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
TEST_LOG_ROOT = Path("/home/pi/logs_nse2026")
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

def build_phase_runner_parser(description: str, default_label: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--debug-label", default=default_label)
    parser.add_argument("--log-dir", default=None)
    return parser


def resolve_runtime_args(args):
    if args.log_dir:
        log_dir = str(Path(args.log_dir))
    else:
        safe_label = str(args.debug_label or "debug").strip().replace(" ", "_")
        log_dir = str(TEST_LOG_ROOT / "debug" / safe_label)
    return {
        "log_dir": log_dir,
    }
