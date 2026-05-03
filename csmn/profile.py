import os
import socket
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Iterable


@dataclass(frozen=True)
class MachineProfile:
    name: str
    description: str
    const_overrides: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class MachineProfileResolution:
    name: str
    source: str
    fallback: bool = False


_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_LOG_ROOT = _REPO_ROOT / "log"
_MARKER_PATHS = (
    _REPO_ROOT / ".cansat_machine",
    Path("/etc/cansat_machine"),
)


PROFILE_REGISTRY: dict[str, MachineProfile] = {
    "common": MachineProfile(
        name="common",
        description="共通ベース設定。機体固有のトリムをまだ入れない入口。",
        const_overrides={
            "LOG_DIR": str(_DEFAULT_LOG_ROOT / "common"),
            "CAMERA_CONTROL_INVERT_X": False,
        },
    ),
    "unit1": MachineProfile(
        name="unit1",
        description="1号機用プロファイル。機体固有の補正値はここに集約する。",
        const_overrides={
            "LOG_DIR": str(_DEFAULT_LOG_ROOT / "unit1"),
            "CAMERA_CONTROL_INVERT_X": False,
            "MOTOR_SPEED_SCALE_1": 1.00,
            "MOTOR_SPEED_SCALE_2": 1.00,
            "MOTOR_SPEED_OFFSET_1": 0.0,
            "MOTOR_SPEED_OFFSET_2": 0.0,
        },
    ),
    "unit2": MachineProfile(
        name="unit2",
        description="2号機用プロファイル。1号機との差分だけをここに置く。",
        const_overrides={
            "LOG_DIR": str(_DEFAULT_LOG_ROOT / "unit2"),
            "CAMERA_CONTROL_INVERT_X": False,
            "MOTOR_SPEED_SCALE_1": 1.00,
            "MOTOR_SPEED_SCALE_2": 1.00,
            "MOTOR_SPEED_OFFSET_1": 0.0,
            "MOTOR_SPEED_OFFSET_2": 0.0,
        },
    ),
}


HOSTNAME_PROFILE_MAP: dict[str, str] = {
    "nse2026-unit1": "unit1",
    "nse2026-unit2": "unit2",
    "cansat-unit1": "unit1",
    "cansat-unit2": "unit2",
}


PATCHABLE_MODULES = (
    "csmn.const",
    "csmn.ctrl",
    "csmn.gps_util",
    "csmn.mgr.hw_mgr",
    "csmn.mgr.mtr_mgr",
    "csmn.mgr.sns_mgr",
    "csmn.run",
)


def list_profiles() -> tuple[str, ...]:
    return tuple(PROFILE_REGISTRY.keys())


def _normalize_profile_name(raw_name: object) -> str:
    return str(raw_name).strip().lower()


def _require_known_profile(profile_name: str, source: str) -> str:
    if profile_name in PROFILE_REGISTRY:
        return profile_name
    if source.startswith("fallback"):
        return "common"
    available = ", ".join(list_profiles())
    raise ValueError(f"Unknown machine profile from {source}: {profile_name} (available: {available})")


def _read_marker_profile() -> MachineProfileResolution | None:
    for path in _MARKER_PATHS:
        try:
            if not path.exists():
                continue
            marker_value = path.read_text(encoding="utf-8").splitlines()[0]
        except (OSError, IndexError):
            continue
        profile_name = _require_known_profile(_normalize_profile_name(marker_value), f"marker:{path}")
        return MachineProfileResolution(name=profile_name, source=f"marker:{path}")
    return None


def _detect_profile_from_hostname() -> MachineProfileResolution | None:
    hostname = _normalize_profile_name(socket.gethostname())
    if not hostname:
        return None
    if hostname in PROFILE_REGISTRY:
        return MachineProfileResolution(name=hostname, source="hostname")
    mapped_name = HOSTNAME_PROFILE_MAP.get(hostname)
    if mapped_name is None:
        return None
    return MachineProfileResolution(
        name=_require_known_profile(mapped_name, f"hostname:{hostname}"),
        source="hostname",
    )


def resolve_machine_profile(profile_name: str | None = None) -> MachineProfileResolution:
    if profile_name is not None and str(profile_name).strip() != "":
        resolved_name = _require_known_profile(_normalize_profile_name(profile_name), "argument")
        return MachineProfileResolution(name=resolved_name, source="argument")

    env_name = os.getenv("CANSAT_MACHINE")
    if env_name is not None and str(env_name).strip() != "":
        resolved_name = _require_known_profile(_normalize_profile_name(env_name), "CANSAT_MACHINE")
        return MachineProfileResolution(name=resolved_name, source="CANSAT_MACHINE")

    marker_resolution = _read_marker_profile()
    if marker_resolution is not None:
        return marker_resolution

    hostname_resolution = _detect_profile_from_hostname()
    if hostname_resolution is not None:
        return hostname_resolution

    # 判別不能時は機体固有補正を含まない common に落とす。
    # 明示指定やマーカーファイルが未知値だった場合は上で ValueError にする。
    return MachineProfileResolution(name="common", source="fallback:common", fallback=True)


def get_profile(profile_name: str | None) -> MachineProfile:
    resolution = resolve_machine_profile(profile_name)
    try:
        return PROFILE_REGISTRY[resolution.name]
    except KeyError as exc:
        available = ", ".join(list_profiles())
        raise ValueError(f"Unknown machine profile: {resolution.name} (available: {available})") from exc


def build_mission_log_dir(profile_name: str | None) -> str:
    profile = get_profile(profile_name)
    return str(Path(str(profile.const_overrides.get("LOG_DIR", _DEFAULT_LOG_ROOT / profile.name))))


def build_debug_log_dir(base_root: Path, profile_name: str | None, scope: str, label: str) -> str:
    profile = get_profile(profile_name)
    safe_label = str(label or "debug").strip().replace(" ", "_")
    if scope == "shared":
        return str(base_root / "shared" / safe_label / profile.name)
    return str(base_root / "by_machine" / profile.name / safe_label)


def _iter_target_modules(extra_modules: Iterable[str | ModuleType] | None = None):
    yielded: set[str] = set()
    for module_name in PATCHABLE_MODULES:
        module = sys.modules.get(module_name)
        if module is None:
            continue
        yielded.add(module.__name__)
        yield module

    if not extra_modules:
        return

    for extra in extra_modules:
        if isinstance(extra, ModuleType):
            module = extra
        else:
            module = sys.modules.get(str(extra))
        if module is None or module.__name__ in yielded:
            continue
        yielded.add(module.__name__)
        yield module


def activate_machine_profile(
    profile_name: str | None = None,
    *,
    log_dir: str | os.PathLike[str] | None = None,
    target_lat: float | None = None,
    target_lng: float | None = None,
    extra_overrides: dict[str, object] | None = None,
    extra_modules: Iterable[str | ModuleType] | None = None,
) -> MachineProfile:
    profile = get_profile(profile_name)
    overrides = dict(profile.const_overrides)

    # 明示引数はプロファイル既定値より優先する。ログ形式は既存のまま、
    # 保存先だけを確定済みプロファイルに合わせて差し替える。
    if log_dir is not None:
        overrides["LOG_DIR"] = str(log_dir)
    if target_lat is not None:
        overrides["TARGET_LAT"] = float(target_lat)
    if target_lng is not None:
        overrides["TARGET_LNG"] = float(target_lng)
    if extra_overrides:
        overrides.update(extra_overrides)

    import csmn.const as mission_constants

    for key, value in overrides.items():
        setattr(mission_constants, key, value)

    for module in _iter_target_modules(extra_modules):
        for key, value in overrides.items():
            if hasattr(module, key):
                setattr(module, key, value)

    return profile
