import os
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


_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_LOG_ROOT = _REPO_ROOT / "log"


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


def get_profile(profile_name: str | None) -> MachineProfile:
    resolved_name = str(profile_name or os.getenv("CANSAT_MACHINE", "common")).strip().lower()
    try:
        return PROFILE_REGISTRY[resolved_name]
    except KeyError as exc:
        available = ", ".join(list_profiles())
        raise ValueError(f"Unknown machine profile: {resolved_name} (available: {available})") from exc


def build_mission_log_dir(profile_name: str) -> str:
    profile = get_profile(profile_name)
    return str(Path(str(profile.const_overrides.get("LOG_DIR", _DEFAULT_LOG_ROOT / profile.name))))


def build_debug_log_dir(base_root: Path, profile_name: str, scope: str, label: str) -> str:
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
