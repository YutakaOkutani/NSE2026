from common import build_phase_runner_parser, resolve_runtime_args
from mission.const import Phase
from mission.run import run_phase_sequence


def main():
    args = build_phase_runner_parser(
        description="Phase0-Phase1 orchestration runner",
        default_label="orch_p0_p1",
    ).parse_args()
    runtime_args = resolve_runtime_args(args)
    run_phase_sequence(
        start_phase=Phase.PHASE0,
        allowed_phases=(Phase.PHASE0, Phase.PHASE1),
        **runtime_args,
    )


if __name__ == "__main__":
    main()
