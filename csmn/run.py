import os

from csmn import const as mission_constants
from csmn.const import Phase
from csmn.profile import activate_machine_profile, build_mission_log_dir


def _resolve_target(target_lat, target_lng):
    if target_lat is None:
        target_lat = mission_constants.TARGET_LAT
    if target_lng is None:
        target_lng = mission_constants.TARGET_LNG
    return float(target_lat), float(target_lng)


def _read_env_float(env_key):
    raw = os.getenv(env_key)
    if raw is None or str(raw).strip() == "":
        return None
    return float(raw)


def _activate_runtime(machine_name=None, log_dir=None, target_lat=None, target_lng=None):
    resolved_machine = str(machine_name or os.getenv("CANSAT_MACHINE", "common")).strip().lower()
    resolved_log_dir = log_dir if log_dir is not None else os.getenv("CANSAT_LOG_DIR")
    if resolved_log_dir is None:
        resolved_log_dir = build_mission_log_dir(resolved_machine)

    resolved_target_lat = target_lat if target_lat is not None else _read_env_float("CANSAT_TARGET_LAT")
    resolved_target_lng = target_lng if target_lng is not None else _read_env_float("CANSAT_TARGET_LNG")
    activate_machine_profile(
        resolved_machine,
        log_dir=resolved_log_dir,
        target_lat=resolved_target_lat,
        target_lng=resolved_target_lng,
    )
    return resolved_machine


def run_full_mission(target_lat=None, target_lng=None, machine_name=None, log_dir=None):
    from csmn.ctrl import CanSatController

    machine_name = _activate_runtime(
        machine_name=machine_name,
        log_dir=log_dir,
        target_lat=target_lat,
        target_lng=target_lng,
    )
    target_lat, target_lng = _resolve_target(target_lat, target_lng)
    controller = CanSatController(target_lat, target_lng, machine_name=machine_name)
    controller.run(start_phase=Phase.PHASE0)


def run_phase_sequence(start_phase, allowed_phases, target_lat=None, target_lng=None, machine_name=None, log_dir=None):
    from csmn.ctrl import CanSatController

    machine_name = _activate_runtime(
        machine_name=machine_name,
        log_dir=log_dir,
        target_lat=target_lat,
        target_lng=target_lng,
    )
    target_lat, target_lng = _resolve_target(target_lat, target_lng)
    controller = CanSatController(target_lat, target_lng, machine_name=machine_name)
    controller.run(start_phase=start_phase, allowed_phases=allowed_phases)


def run_single_phase(phase, target_lat=None, target_lng=None, machine_name=None, log_dir=None):
    run_phase_sequence(
        start_phase=phase,
        allowed_phases=(phase,),
        target_lat=target_lat,
        target_lng=target_lng,
        machine_name=machine_name,
        log_dir=log_dir,
    )
