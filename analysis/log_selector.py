from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
LOGS_DIR = REPO_ROOT / "analysis" / "robust_logs"


def _format_size(size_bytes: int) -> str:
    if size_bytes < 1024:
        return f"{size_bytes} B"
    elif size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f} KB"
    else:
        return f"{size_bytes / (1024 * 1024):.1f} MB"


def get_robust_log_candidates() -> list[Path]:
    """Return all robust_log_*.csv files in robust_logs directory, sorted by mtime (newest first)."""
    if not LOGS_DIR.exists():
        return []
    candidates = list(LOGS_DIR.rglob("robust_log_*.csv"))
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates


def find_latest_log() -> Path:
    """Return the newest robust_log_*.csv file path."""
    candidates = get_robust_log_candidates()
    if not candidates:
        raise FileNotFoundError(f"No robust_log_*.csv found in {LOGS_DIR}")
    return candidates[0]


def resolve_log_path(file_path: str | Path | None = None) -> Path:
    """Resolve log path.

    If file_path is provided, return its resolved Path.
    If file_path is None:
      - If stdin is an interactive terminal (isatty), display interactive log selection menu.
      - If stdin is non-interactive, return the latest log.
    """
    if file_path:
        path = Path(file_path).resolve()
        if not path.exists():
            raise FileNotFoundError(f"Specified log file not found: {path}")
        return path

    candidates = get_robust_log_candidates()
    if not candidates:
        raise FileNotFoundError(f"No robust_log_*.csv found in {LOGS_DIR}")

    # Fallback to latest log in non-interactive environment (e.g. CI, pipe)
    if not sys.stdin.isatty():
        latest = candidates[0]
        print(f"[INFO] Non-interactive shell detected. Automatically selected latest log: {latest.name}")
        return latest

    # Display interactive selection menu
    total = len(candidates)
    width = len(str(total))

    print("\n" + "=" * 70)
    print("  Select a robust log file to analyze")
    print("=" * 70)

    for i, candidate in enumerate(candidates, start=1):
        stat = candidate.stat()
        mtime_str = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
        size_str = _format_size(stat.st_size)
        tag = " (Latest)" if i == 1 else ""
        print(f" [{i:{width}d}] {candidate.name:<45} ({mtime_str}, {size_str}){tag}")

    print("=" * 70)

    while True:
        try:
            user_input = input(f"Select log number [1-{total}] (default: 1): ").strip()
            if not user_input:
                selected = candidates[0]
                print(f"[INFO] Selected default (Latest): {selected.name}\n")
                return selected

            choice = int(user_input)
            if 1 <= choice <= total:
                selected = candidates[choice - 1]
                print(f"[INFO] Selected [{choice}]: {selected.name}\n")
                return selected
            else:
                print(f"[WARN] Invalid number. Please enter a value between 1 and {total}.")
        except ValueError:
            print("[WARN] Invalid input. Please enter a number or press Enter for default.")
        except (KeyboardInterrupt, EOFError):
            print("\n[INFO] Operation cancelled by user.")
            sys.exit(0)
