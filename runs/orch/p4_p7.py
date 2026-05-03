from common import build_phase_runner_parser, resolve_runtime_args
from csmn.const import Phase
from csmn.run import run_phase_sequence


def main():
    args = build_phase_runner_parser(
        description="Phase4-Phase7 orchestration runner",
        default_label="orch_p4_p7",
    ).parse_args()
    runtime_args = resolve_runtime_args(args)
    run_phase_sequence(
        start_phase=Phase.PHASE4,
        allowed_phases=(Phase.PHASE4, Phase.PHASE5, Phase.PHASE6, Phase.PHASE7),
        **runtime_args,
    )


if __name__ == "__main__":
    main()
