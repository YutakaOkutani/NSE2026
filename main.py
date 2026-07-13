import sys

from mission.config import MissionConfigError
from mission.run import run_full_mission


def main():
    if len(sys.argv) != 1:
        raise SystemExit("main.py does not accept configuration arguments; edit mission.toml instead")
    try:
        run_full_mission()
    except MissionConfigError as exc:
        raise SystemExit(f"Mission configuration error: {exc}") from exc


if __name__ == "__main__":
    main()
