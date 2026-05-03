from common import TEST_LOG_ROOT, build_phase_runner_parser, resolve_runtime_args
from csmn.const import Phase
from csmn.run import run_phase_sequence


def run_phase0_logging(log_subdir: str, machine_name: str = "common") -> None:
    log_dir = TEST_LOG_ROOT / log_subdir
    run_phase_sequence(
        start_phase=Phase.PHASE0,
        allowed_phases=(Phase.PHASE0,),
        machine_name=machine_name,
        log_dir=str(log_dir),
    )


def main():
    args = build_phase_runner_parser(
        description="Phase0 logging runner",
        default_label="orch_p0_log",
    ).parse_args()
    runtime_args = resolve_runtime_args(args)
    run_phase_sequence(
        start_phase=Phase.PHASE0,
        allowed_phases=(Phase.PHASE0,),
        **runtime_args,
    )


if __name__ == "__main__":
    main()
