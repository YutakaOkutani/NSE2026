# AI Context Audit and Refactor Record

> **Audience: AI documentation maintainers only.** Do not load for coding or debugging. This page records information-value decisions and document ownership.

## Evaluation scale

- `Read value`: value to an agent making a relevant change, from `★★★★★` to `★`.
- `Recoverable`: `YES` when source code alone makes the information easy to reconstruct.
- `Mistake prevention`: contribution to avoiding unsafe or architecturally wrong changes.
- `Worth tokens`: retain only `YES` rows in active context. Deleted material is recorded separately.

## Retained section audit

| Owner / section | Read value | Recoverable | Mistake prevention | Worth tokens | Retention reason |
| --- | --- | --- | --- | --- | --- |
| Index / required read order | ★★★★★ | NO | ★★★★★ | YES | Prevents loading all docs and fixes priority order |
| Index / document ownership | ★★★★ | NO | ★★★★★ | YES | Prevents duplication and responsibility drift |
| Overview / system identity | ★★★★★ | NO | ★★★★★ | YES | Prevents multi-airframe and experimental-path redesign |
| Overview / repository map | ★★★★ | PARTIAL | ★★★★ | YES | Directory names do not reveal production vs diagnostic authority |
| Overview / task routing | ★★★★ | NO | ★★★★ | YES | Reduces discovery tokens and missed coupled files |
| Overview / sources of truth | ★★★★★ | NO | ★★★★★ | YES | Prevents docs or examples becoming authoritative |
| Architecture / production flow | ★★★★★ | PARTIAL | ★★★★★ | YES | Concurrency and lifecycle are dispersed across modules |
| Architecture / module ownership | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents responsibilities moving between similarly named files |
| Architecture / dependency direction | ★★★★★ | NO | ★★★★★ | YES | Prevents direct hardware and reverse dependencies |
| Architecture / controller contract | ★★★★★ | PARTIAL | ★★★★★ | YES | Multiple inheritance obscures intended responsibility |
| Architecture / state contract | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents races and validity mistakes |
| Architecture / Manager contract | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents Phase/hardware responsibility leakage |
| Architecture / Phase contract | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents unbounded or opaque transitions |
| Architecture / time and shutdown | ★★★★★ | PARTIAL | ★★★★★ | YES | Safety behavior is cross-cutting |
| Architecture / observability | ★★★★ | PARTIAL | ★★★★ | YES | Prevents console-only and incompatible logging |
| Principles / decision order | ★★★★★ | NO | ★★★★★ | YES | Resolves design tradeoffs consistently |
| Principles / invariants | ★★★★★ | NO | ★★★★★ | YES | Preserves product design intent |
| Principles / acceptance and failure | ★★★★ | NO | ★★★★★ | YES | Blocks unsafe success-only changes |
| Repository rules / change boundaries | ★★★★★ | PARTIAL | ★★★★★ | YES | Defines allowed and paired edits |
| Repository rules / protected surfaces | ★★★★★ | NO | ★★★★★ | YES | Prevents local config, hardware, and human-doc damage |
| Repository rules / prohibited designs | ★★★★★ | NO | ★★★★★ | YES | Compact NG list directly prevents drift |
| Repository rules / docs rules | ★★★ | NO | ★★★★ | YES | Needed only for docs maintenance |
| Coding / placement | ★★★★ | PARTIAL | ★★★★★ | YES | Prevents implementation in the wrong layer |
| Coding / state and concurrency | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents race/stale-value defects |
| Coding / phases, sensors, hardware | ★★★★★ | PARTIAL | ★★★★★ | YES | Encodes safety and transition expectations |
| Coding / config and constants | ★★★★ | PARTIAL | ★★★★★ | YES | Prevents runtime override and budget errors |
| Coding / logs and cleanup | ★★★★★ | PARTIAL | ★★★★★ | YES | Protects schema and terminal safety |
| Debug / diagnosis order | ★★★★★ | NO | ★★★★★ | YES | Prevents threshold-first debugging |
| Debug / test tiers | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents overclaiming and wrong test placement |
| Debug / regression mapping | ★★★★ | YES | ★★★★ | YES | Small token cost materially reduces missed suites |
| Debug / E2E rules | ★★★★★ | NO | ★★★★★ | YES | Hardware tests can damage the vehicle or mislead |
| Debug / failure triage | ★★★★ | PARTIAL | ★★★★★ | YES | Encodes repository-specific first evidence |
| Mission / mission and config contracts | ★★★★★ | PARTIAL | ★★★★★ | YES | Defines state-machine semantics and config authority |
| Mission / Phase responsibilities | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents responsibility and transition drift |
| Mission / P2, P3, P4/5 invariants | ★★★★★ | PARTIAL | ★★★★★ | YES | High-risk strategies span multiple modules |
| Mission / timeout semantics | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents false-success and re-entry budget errors |
| Competition provenance | ★★★★ | NO | ★★★★ | YES | Prevents one event's rules and field tuning becoming universal defaults |
| Hardware / electrical constraint | ★★★★★ | NO | ★★★★★ | YES | Not visible in code; prevents GPIO damage |
| Hardware / motor and radio constraints | ★★★★★ | PARTIAL | ★★★★★ | YES | Prevents uncontrolled motion and lost recovery |
| Telemetry / current status and boundary | ★★★★★ | NO | ★★★★★ | YES | Prevents prototype being treated as production control |
| Telemetry / transport and video separation | ★★★★ | PARTIAL | ★★★★★ | YES | Preserves independent low-load state channel |
| Telemetry / logging and change rules | ★★★★ | PARTIAL | ★★★★ | YES | Prevents incompatible endpoints and untraceable loss |
| Legacy aliases / owner redirect | ★ | YES | ★★★ | YES | Small cost preserves links outside the authorized docs scope |

`PARTIAL` is used where code reveals mechanics but not the intended boundary. It is treated as `NO` for deletion decisions.

## Deleted content

- Removed Python/Git/Markdown/D2 installation and syntax teaching.
- Removed long command catalogs and dependency/package introductions.
- Removed full telemetry sample payload and field-by-field UI wish list.
- Removed staged future roadmaps that did not constrain current implementation.
- Removed generic quality prose, historical motivation, and repeated “purpose” sections.
- Removed duplicated directory trees, log destination tables, and source-visible constant explanations.
- Removed camera relay invocation instructions already owned by script help/runtime README.
- Removed systemd and sudoers tutorials; deployment examples and human README own setup.
- Removed four D2 flowcharts because they duplicated code, contained stale Phase 0/2 behavior, and created a second mission specification.
- Removed the D2 README because generated-diagram tooling no longer exists in `docs/`.
- Replaced old paths referenced outside `docs/` with minimal compatibility aliases; aliases contain no independent context.

## Integrated content

| Previous files | New owner | Integration decision |
| --- | --- | --- |
| `architecture/overview.md`, `development/components.md` | `01`, `02`, `04` | Kept only responsibility and dependency boundaries |
| `development/principles.md`, policy parts of `development/mission.md` | `03`, `05` | Converted prose into enforceable rules |
| `development/mission.md`, flowchart semantics | `07` | Replaced duplicated algorithm narration with Phase contracts/invariants |
| `development/testing.md` | `06`, hardware reference | Split test architecture from electrical/bench safety; old path is an anchor-preserving alias |
| `operations/radio_control.md` | hardware reference | Retained only configuration authority and recovery safety; old path is an alias |
| `operations/camera_relay.md`, telemetry docs | telemetry reference | Marked legacy relay vs current Phase A boundary; externally linked telemetry paths are aliases |
| old `index.md` | `README.md` | Added minimal routing and explicit ownership; old path is an alias |

## Newly added information

- Added a strict load order and conditional context routing.
- Added one-owner-per-rule document contracts.
- Added explicit dependency direction and identification of legacy Phase-to-device coupling.
- Added allowed/protected change surfaces and a centralized NG list.
- Added a state/concurrency contract and atomic log-schema change rule.
- Added E2E promotion gates, skipped-Phase assumptions, and evidence standards.
- Added mission terminal semantics and Phase 2/3/4/5 invariants derived from current code/tests.
- Added current telemetry integration status so agents do not assume a publisher exists.
- Added a persistent value audit so future pruning uses the same token-value criterion.
- Added run-bundle/config-metadata boundaries and an NSE2026 provenance page without restoring runtime profile selection.
- Reserved `flowchart/` with `.gitkeep` for future diagrams without restoring stale diagram content.

## Maintenance rule

For every proposed section, ask: “Would an agent plausibly make a wrong or unsafe design without this?” Delete it when the answer is no. When the answer is yes, keep the shortest rule form and assign exactly one owner.

## AI Checklist

- Did I evaluate every added section for recoverability and design-error prevention?
- Did I delete source-recoverable or human-oriented detail?
- Does each retained rule have exactly one owner?
- Did I update the deleted, integrated, and newly added inventories?
- Is the startup context smaller and more decision-dense after this change?
