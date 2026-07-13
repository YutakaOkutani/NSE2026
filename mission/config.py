import math
from dataclasses import dataclass
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 and older
    tomllib = None


class MissionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class TargetConfig:
    latitude: float
    longitude: float


@dataclass(frozen=True)
class RadioConfig:
    control: str
    pre_off_delay_sec: float
    restore_timeout_sec: float
    use_sudo: bool
    dry_run: bool


@dataclass(frozen=True)
class MissionConfig:
    target: TargetConfig
    radio: RadioConfig
    source: Path


_REPO_ROOT = Path(__file__).resolve().parents[1]
MISSION_CONFIG_PATH = _REPO_ROOT / "mission.toml"
_ROOT_KEYS = {"target", "radio"}
_TARGET_KEYS = {"latitude", "longitude"}
_RADIO_KEYS = {
    "control",
    "pre_off_delay_sec",
    "restore_timeout_sec",
    "use_sudo",
    "dry_run",
}


def _require_table(data, key, path):
    value = data.get(key)
    if not isinstance(value, dict):
        raise MissionConfigError(f"{path}: [{key}] table is required")
    return value


def _reject_unknown_keys(table, allowed, label, path):
    unknown = sorted(set(table) - allowed)
    if unknown:
        raise MissionConfigError(f"{path}: unknown key(s) in {label}: {', '.join(unknown)}")


def _require_number(table, key, label, path):
    if key not in table:
        raise MissionConfigError(f"{path}: {label}.{key} is required")
    value = table[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionConfigError(f"{path}: {label}.{key} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise MissionConfigError(f"{path}: {label}.{key} must be finite")
    return value


def _require_bool(table, key, label, path):
    if key not in table:
        raise MissionConfigError(f"{path}: {label}.{key} is required")
    value = table[key]
    if not isinstance(value, bool):
        raise MissionConfigError(f"{path}: {label}.{key} must be true or false")
    return value


def load_mission_config(path=None):
    if tomllib is None:
        raise MissionConfigError("mission.toml requires Python 3.11 or newer")
    config_path = Path(path) if path is not None else MISSION_CONFIG_PATH
    try:
        with config_path.open("rb") as file_obj:
            data = tomllib.load(file_obj)
    except FileNotFoundError as exc:
        raise MissionConfigError(
            f"Mission config not found: {config_path}. "
            "Copy mission.toml.example to mission.toml and set the target coordinates."
        ) from exc
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise MissionConfigError(f"Failed to read {config_path}: {exc}") from exc

    if not isinstance(data, dict):
        raise MissionConfigError(f"{config_path}: root must be a TOML table")
    _reject_unknown_keys(data, _ROOT_KEYS, "root", config_path)

    target = _require_table(data, "target", config_path)
    radio = _require_table(data, "radio", config_path)
    _reject_unknown_keys(target, _TARGET_KEYS, "target", config_path)
    _reject_unknown_keys(radio, _RADIO_KEYS, "radio", config_path)

    latitude = _require_number(target, "latitude", "target", config_path)
    longitude = _require_number(target, "longitude", "target", config_path)
    if not -90.0 <= latitude <= 90.0:
        raise MissionConfigError(f"{config_path}: target.latitude must be between -90 and 90")
    if not -180.0 <= longitude <= 180.0:
        raise MissionConfigError(f"{config_path}: target.longitude must be between -180 and 180")

    control = radio.get("control")
    if not isinstance(control, str) or control not in ("off", "mission"):
        raise MissionConfigError(f'{config_path}: radio.control must be "off" or "mission"')
    pre_off_delay_sec = _require_number(radio, "pre_off_delay_sec", "radio", config_path)
    restore_timeout_sec = _require_number(radio, "restore_timeout_sec", "radio", config_path)
    if pre_off_delay_sec < 0.0:
        raise MissionConfigError(f"{config_path}: radio.pre_off_delay_sec must be non-negative")
    if restore_timeout_sec < 0.0:
        raise MissionConfigError(f"{config_path}: radio.restore_timeout_sec must be non-negative")

    return MissionConfig(
        target=TargetConfig(latitude=latitude, longitude=longitude),
        radio=RadioConfig(
            control=control,
            pre_off_delay_sec=pre_off_delay_sec,
            restore_timeout_sec=restore_timeout_sec,
            use_sudo=_require_bool(radio, "use_sudo", "radio", config_path),
            dry_run=_require_bool(radio, "dry_run", "radio", config_path),
        ),
        source=config_path.resolve(),
    )
