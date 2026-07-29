# 05 Coding Rules

> **Audience: AI coding and debugging agents.** Apply these rules to implementation changes. This page intentionally omits generic Python style.

## Placement

- Put mission decisions in the owning `Phase*Handler`.
- Put cross-phase lifecycle, dispatch, cumulative timeout, and shutdown in `CanSatController`.
- Put hardware initialization, polling, retries, normalization, and actuation in the matching Manager.
- Put device-protocol or reusable detector details in `lib/`.
- Put pure navigation math in `mission/nav.py`; put GPS transport/parsing helpers in `mission/gps_util.py`.
- Put fixed airframe and control values in `mission/const.py`; put per-mission target/radio values only in `mission.toml`.
- Put hardware-free regression tests in `runs/spec/`, device diagnostics in `runs/diag/`, and partial production runs in `runs/orch/`.

## State and concurrency

- Update shared facts only through `CanSatState.update_*`.
- Read shared facts through `snapshot()`; avoid multiple snapshots within one decision unless freshness between reads is intentional.
- Keep state update methods short and lock-scoped; perform I/O and expensive calculation outside the lock.
- Add explicit validity/freshness fields with a measurement. Do not use `0`, an empty vector, or the last good value as implicit validity.
- Increment sequence values only when a new accepted observation arrives.
- Keep worker threads daemonized only with an independent safe shutdown path; thread failure must not leave motors active.
- Protect shared hardware buses with the existing lock rather than creating competing device instances.

## Phases and transitions

- Reset entry-local timers, counters, and latches when the phase entry marker changes.
- Require temporal or sample confirmation for noisy release, arrival, and vision decisions.
- Record why a transition occurred. Keep forced timeout and fallback distinct from verified success.
- Preserve cumulative time accounting across Phase re-entry.
- Use reverse transitions only with an explicit recovery condition and bounded retry/time budget.
- Do not call device-driver objects from new Phase code. Add or use a Manager method.
- Add a spec test for success, invalid input, timeout, and fallback boundaries that changed.

## Sensors and hardware

- Separate initialization failure, read failure, range/format invalidity, staleness, and recovery.
- Keep the last good value only when paired with a stale age and validity flag.
- Bound reinitialization frequency and log the reason.
- Clamp actuator inputs and apply logical-to-physical direction only in the motor layer.
- On any actuator exception or loop exit, stop motors.
- Follow [`reference/hardware-safety.md`](reference/hardware-safety.md) before changing or running physical I/O.

## Configuration and constants

- Validate all configuration before importing/constructing hardware-dependent control.
- Reject unknown keys; do not silently accept misspelled configuration.
- Keep `mission.toml.example` deliberately non-runnable until target coordinates are edited.
- Do not restore legacy target constants, environment variables, or production CLI overrides.
- When changing a timing constant, inspect both local Phase timeout and controller cumulative budget.
- When changing motor mapping or trim, update and run the motor safety specs before any bench run.

## Logs and analysis

- Treat `LOG_HEADER` and `_build_log_row()` as one ordered schema.
- Change header, row builder, `runs/spec/log_schema.py`, and affected `analysis/` consumers atomically.
- Log reason, unit, validity/freshness, and terminal semantics; avoid unbounded debug strings.
- Flush the primary mission record often enough to survive power loss; do not add heavy analysis to the log thread.
- Keep standard output concise and rate-limited inside fast loops.

## Errors and cleanup

- Catch exceptions only where a safe fallback, retry, cleanup, or clearer boundary error exists.
- Never swallow actuator, configuration, state serialization, or hardware initialization failures silently.
- Keep `request_shutdown()` idempotent.
- Preserve cleanup in `finally` for motors, serial, sockets, cameras, GPIO factories, and radio restoration.
- Ensure startup failures still request shutdown even if hardware setup only partially completed.

## Minimal implementation loop

1. Identify the owning layer and invariant.
2. Add or tighten the smallest failing spec when hardware is not essential.
3. Change production code without duplicating it in `runs/`.
4. Update logs/analysis/docs only when their contract changes.
5. Run the mapped spec suite.
6. Promote to diagnostic, partial integration, and E2E only as required by risk.

## AI Checklist

- Is the code in the layer that owns the decision or I/O?
- Are shared reads atomic and accompanied by validity/freshness?
- Are transition reasons, retries, and timeouts explicit and bounded?
- Can every new failure path clean up safely?
- Did I update schema consumers and regression tests together?
