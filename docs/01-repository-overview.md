# 01 Repository Overview

> **Audience: AI coding and debugging agents.** Read at the start of every repository task. For human setup or first execution, refer to [`../README.md`](../README.md).

## System identity

- Treat this repository as the production control system for one CanSat airframe.
- Do not add airframe selection, per-airframe configuration, or per-airframe log trees.
- Keep `main.py` as the minimal production entry point.
- Keep production behavior in `mission/`; use `runs/` only to exercise or diagnose it.
- Optimize for safe termination, observable decisions, repeatable field behavior, and goal completion in that order.

## Repository map

| Path | Responsibility | Load or change when |
| --- | --- | --- |
| `main.py` | Validate startup arguments and invoke the full mission | Production startup changes only |
| `mission/` | Production state, orchestration, decisions, hardware management | Any flight/drive behavior change |
| `mission/phases/` | Phase-local mission decisions and transitions | One phase's strategy changes |
| `mission/mgr/` | Hardware lifecycle, sensor acquisition, actuators, radio | Physical I/O or continuous control changes |
| `lib/` | Low-level sensor and vision adapters | Device protocol or detector implementation changes |
| `runs/spec/` | Hardware-free behavioral regression tests | Every decision or safety bug fix |
| `runs/diag/` | One-device diagnostics | Hardware isolation and bench checks |
| `runs/orch/` | Partial end-to-end runs through production controller paths | Multi-phase real-hardware verification |
| `runs/cam/` | Image capture, detector debugging, legacy relay | Vision evidence or relay debugging |
| `runs/telemetry/` | Independent Phase A telemetry prototype | Telemetry-only work |
| `analysis/` | Offline mission-log diagnosis and exploration reconstruction | Log interpretation or schema consumers |
| `deploy/systemd/` | Deployment examples | Boot/runtime integration changes |
| `scripts/` | Host operational helpers | Non-mission host automation |
| `gerber/` | Released PCB fabrication archive | Board-specific task with explicit authority only |
| `docs/` | AI decision context | Architecture/rule changes only |
| `README.md` | Human overview, setup, and execution | Human documentation task only |

Generated logs, captures, analysis outputs, local virtual environments, and `mission.toml` are runtime artifacts, not source modules.

## Task routing

| Change request | Primary code | Required context |
| --- | --- | --- |
| Phase transition or timeout | `mission/phases/`, `mission/ctrl.py`, `mission/const.py` | `02` through `07` |
| Sensor validity or freshness | `mission/mgr/sns_mgr.py`, `mission/st.py`, `lib/` | `02` through `07`, hardware reference |
| Motor behavior or direction | `mission/mgr/mtr_mgr.py`, `mission/motor_map.py`, `mission/const.py` | `02` through `07`, hardware reference |
| Configuration | `mission/config.py`, `mission.toml.example` | `02` through `06` |
| Log schema | `mission/const.py`, `mission/mgr/sns_mgr.py`, `analysis/`, `runs/spec/log_schema.py` | `02` through `07` |
| Vision | `lib/cone_detect.py`, Phase 4/5, camera thread, `runs/cam/` | `02` through `07` |
| Telemetry | `runs/telemetry/` | `02` through `06`, telemetry reference |
| Test-only tooling | matching `runs/` subtree | `02` through `06` |

## Sources of truth

- Use `mission/const.py` for current hardware pins, thresholds, timing, phase budgets, log columns, and fixed single-airframe values.
- Use `mission/config.py` and `mission.toml.example` for the local configuration contract.
- Use `mission/phases/p0.py` through `p7.py` for current phase decisions.
- Use `mission/mgr/` for actual sensor, actuator, and radio behavior.
- Use `runs/spec/` for locked behavioral expectations.
- Use this documentation for intent, boundaries, and rules that code structure alone does not reveal.

If docs and code disagree, stop relying on the doc detail, inspect the relevant tests, and update the owning documentation with the code change.

## AI Checklist

- Have I identified the production path and avoided placing it in `runs/`?
- Have I opened the source-of-truth file instead of trusting duplicated details?
- Is this still a single-airframe design?
- Have I loaded the context page that owns this change?
