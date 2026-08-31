# 04 Repository Rules

> **Audience: AI coding and debugging agents.** This page defines allowed change surfaces, protected surfaces, and repository-wide prohibitions.

## Change boundaries

| Area | Allowed change | Required constraint |
| --- | --- | --- |
| `main.py` | Production startup wiring only | Keep argument-free mission execution; validate before hardware import/init |
| `mission/config.py` | Local control config and run-context schemas | Reject malformed or unknown values before hardware setup; keep run metadata behavior-neutral |
| `mission/paths.py`, `mission/run_bundle.py` | Runtime data and provenance | Preserve unique run IDs, snapshots, and legacy analysis compatibility |
| `mission/const.py` | Fixed airframe values, thresholds, budgets, schema | Add no per-mission secrets or target coordinates; pair behavioral values with tests/evidence |
| `mission/st.py` | Shared state fields and atomic update/snapshot API | Update producers, consumers, logs, and tests together |
| `mission/ctrl.py` | Lifecycle, dispatch, cumulative budgets, shutdown | Keep Phase algorithms and device protocols out |
| `mission/mgr/` | Physical I/O and continuous device/actuator policy | Keep mission success decisions out |
| `mission/phases/` | Phase-local state machine decisions | Use Manager APIs; preserve reasons, entry reset, fallback, and bounds |
| `lib/` | Low-level device or vision implementation | Keep controller/Phase dependencies out |
| `runs/spec/` | Fast hardware-free contract tests | Test production behavior, not a duplicate implementation |
| `runs/diag/` | One-device bench tools | Default safe; always release/stop outputs |
| `runs/orch/` | Phase-range production-path runs | Vary only start/allowed phases and debug log destination |
| `runs/cam/` | Vision evidence and legacy relay | Do not move relay control behavior into production |
| `runs/telemetry/` | Independent telemetry prototype | Never control motors, phases, or mission termination |
| `analysis/` | Offline log consumers | Do not make runtime mission code depend on it |
| `deploy/`, `scripts/` | Host integration | Preserve production entry and configuration contract |
| `docs/` | AI-only intent, rules, and high-risk references | Do not duplicate source-visible detail |

## Protected surfaces

- Do not modify root `README.md` while maintaining AI context. It is human-facing and has a separate role.
- Do not commit `mission.toml` or `run-context.toml`; they contain active local values and are intentionally untracked.
- Do not edit `gerber/*.zip` unless the task explicitly concerns PCB fabrication.
- Do not treat generated logs, images, analysis outputs, caches, or virtual environments as source.
- Do not overwrite or normalize unrelated local changes.
- Do not change real GPIO, motor direction/trim, radio mode, or target coordinates as an incidental fix.
- Do not run full mission or actuator diagnostics on attached hardware without the safety preconditions in [`reference/hardware-safety.md`](reference/hardware-safety.md).

## Prohibited designs

- Do not add production logic to `runs/` or copy Phase logic into a diagnostic.
- Do not access hardware directly from a new Phase implementation.
- Do not let telemetry, analysis, or a UI become required for mission control.
- Do not add environment-variable or CLI overrides to production mission configuration.
- Do not add airframe names, selectors, or configuration hierarchies.
- Do not let event labels, notes, or output paths change control decisions.
- Do not scatter numeric tuning values through handlers or diagnostics.
- Do not use a stale retained measurement as valid merely because its number is nonzero.
- Do not hide sensor, command, or log failures with silent broad exception handling.
- Do not change a log header without changing row serialization, schema tests, and relevant analysis.
- Do not leave a debug bypass, forced Phase, or temporary shortcut in `main.py`.
- Do not weaken motor stop, Wi-Fi restoration, timeout, or final-log behavior.

## Documentation rules

- Every file under `docs/` is AI-facing and must declare its audience and load trigger at the top.
- Keep the numbered startup order stable unless the architecture itself changes.
- Put each rule in the file listed in [`README.md`](README.md); link instead of repeating it.
- Remove tutorials, generic tooling explanations, long command catalogs, full sample payloads, and source-recoverable inventories.
- Put numeric values in docs only when they define an external/safety contract; otherwise point to source.
- Update [`maintenance/context-audit.md`](maintenance/context-audit.md) when adding, deleting, merging, or changing a page's role.
- End every Markdown page with `## AI Checklist`.

## AI Checklist

- Am I editing an allowed surface with the required paired updates?
- Have I avoided protected runtime, hardware, and human-documentation assets?
- Did I avoid adding a second source of truth?
- Is temporary debug behavior isolated from production?
- Does this docs change preserve one owner per rule?
