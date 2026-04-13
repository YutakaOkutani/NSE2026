from common import build_phase_runner_parser, resolve_runtime_args
from csmn.const import Phase
from csmn.run import run_phase_sequence


def main():
    args = build_phase_runner_parser(
        description="Phase2-Phase3 orchestration runner",
        default_label="orch_p2_p3",
    ).parse_args()
    runtime_args = resolve_runtime_args(args)
    run_phase_sequence(
        start_phase=Phase.PHASE2,
        allowed_phases=(Phase.PHASE2, Phase.PHASE3),
        **runtime_args,
    )


if __name__ == "__main__":
    main()
