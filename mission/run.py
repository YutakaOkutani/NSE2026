from mission.config import load_mission_config
from mission.const import Phase


def _build_controller(log_dir=None):
    config = load_mission_config()
    from mission.ctrl import CanSatController

    print(
        f"Mission config: {config.source}; "
        f"target=({config.target.latitude:.6f}, {config.target.longitude:.6f}); "
        f"radio={config.radio.control}; dry_run={int(config.radio.dry_run)}; "
        f"use_sudo={int(config.radio.use_sudo)}"
    )
    return CanSatController(config, log_dir=log_dir)


def run_full_mission(log_dir=None):
    controller = _build_controller(log_dir=log_dir)
    controller.run(start_phase=Phase.PHASE0)


def run_phase_sequence(start_phase, allowed_phases, log_dir=None):
    controller = _build_controller(log_dir=log_dir)
    controller.run(start_phase=start_phase, allowed_phases=allowed_phases)


def run_single_phase(phase, log_dir=None):
    run_phase_sequence(
        start_phase=phase,
        allowed_phases=(phase,),
        log_dir=log_dir,
    )
