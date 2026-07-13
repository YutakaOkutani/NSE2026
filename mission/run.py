from mission.config import load_mission_config
from mission.const import Phase
from mission.profile import activate_machine_profile, build_mission_log_dir, resolve_machine_profile


def _activate_runtime(machine_name=None, log_dir=None):
    resolution = resolve_machine_profile(machine_name)
    resolved_machine = resolution.name
    resolved_log_dir = log_dir if log_dir is not None else build_mission_log_dir(resolved_machine)

    # 実行開始前に機体差分を既存定数へ反映する。以降の制御・ログ形式は従来経路を使う。
    activate_machine_profile(
        resolved_machine,
        log_dir=resolved_log_dir,
    )
    return resolved_machine


def _build_controller(machine_name=None, log_dir=None):
    config = load_mission_config()
    from mission.ctrl import CanSatController

    machine_name = _activate_runtime(machine_name=machine_name, log_dir=log_dir)
    print(
        f"Mission config: {config.source}; "
        f"target=({config.target.latitude:.6f}, {config.target.longitude:.6f}); "
        f"radio={config.radio.control}; dry_run={int(config.radio.dry_run)}; "
        f"use_sudo={int(config.radio.use_sudo)}"
    )
    return CanSatController(config, machine_name=machine_name)


def run_full_mission(machine_name=None, log_dir=None):
    controller = _build_controller(machine_name=machine_name, log_dir=log_dir)
    controller.run(start_phase=Phase.PHASE0)


def run_phase_sequence(start_phase, allowed_phases, machine_name=None, log_dir=None):
    controller = _build_controller(machine_name=machine_name, log_dir=log_dir)
    controller.run(start_phase=start_phase, allowed_phases=allowed_phases)


def run_single_phase(phase, machine_name=None, log_dir=None):
    run_phase_sequence(
        start_phase=phase,
        allowed_phases=(phase,),
        machine_name=machine_name,
        log_dir=log_dir,
    )
