# 03 Design Principles

> **Audience: AI coding and debugging agents.** Use these principles to resolve tradeoffs; do not treat them as aspirational prose.

## Decision order

When requirements conflict, choose in this order:

1. Stop or remain safe.
2. Preserve enough evidence to explain the behavior.
3. Recover from transient failure.
4. Continue toward the mission goal.
5. Optimize speed, resource use, or presentation.

Do not trade a higher item for a lower item without an explicit requirement and matching safety test.

## Design invariants

- Keep one authoritative production path. Diagnostic and partial-run entries must reuse it.
- Keep one airframe configuration. Do not add unused generality for hypothetical vehicles.
- Separate observation, decision, and physical actuation.
- Treat validity, freshness, and confirmation history as part of every sensor-dependent decision.
- Prefer conservative fallback or bounded progress over unobservable continuation.
- Bound every retry, recovery loop, reverse transition, and external command.
- Make shutdown idempotent and reachable from every exception path.
- Make control decisions reproducible from structured logs.
- Keep field tuning local: fixed airframe values in `mission/const.py`, per-mission values in untracked `mission.toml`.
- Move CPU-heavy visualization and retrospective inference to `analysis/` or the PC side.

## Change acceptance

Do not accept a behavior change unless all applicable statements are true:

- The changed responsibility remains in its owning layer.
- Failure behavior is defined before the happy path is optimized.
- A new decision has a reason and sufficient logged evidence.
- A threshold change has a field-log or test hypothesis.
- A regression test locks the boundary when hardware is not required.
- Hardware behavior is promoted from unit/spec to diagnostic, partial integration, then full mission.
- Unrelated phases and terminal safety were checked.

## Failure semantics

- Distinguish unavailable, invalid, stale, and recovered sensor states.
- Do not convert exceptions or missing devices into plausible normal values.
- Do not let one optional subsystem stop the mission unless its absence makes continued motion unsafe.
- Do not use infinite retry as recovery.
- On uncertain heading or detection, prefer bounded conservative behavior and explicit logs.
- Do not label forced progress as success; preserve terminal reasons such as timeout, forced goal, or abnormal exit.

## Observability rules

- Log a decision's inputs and reason, not only its output.
- Preserve units and invalid-value semantics when adding a field.
- Use command types and event reasons stable enough for offline grouping.
- Update analysis consumers only for fields with actual diagnostic value.
- Never infer that a change is fixed from a single successful field run without checking the failure evidence.

## AI Checklist

- Did I choose safety and evidence before performance?
- Is the behavior bounded under stale data, missing hardware, and retry?
- Is the change local to one responsibility?
- Can a future agent prove why the system made this decision?
- Is the success/failure label semantically honest?
