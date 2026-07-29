# Telemetry Contract

> **Audience: AI agents changing `runs/telemetry/` or `runs/cam/relay_*`.** This is a subsystem reference, not startup context.

## Current status

- Treat `runs/telemetry/` as a Phase A independent prototype, not a production `main.py` integration.
- `telemetry_sbc.py` currently emits simulated state or reads a JSON state file.
- `telemetry_pc.py` receives, displays link/state health, and writes structured logs.
- Do not document or assume a local publisher from `CanSatState`; it is not implemented.
- Treat `runs/cam/relay_*` as legacy camera-debug tooling that also executes control behavior. Do not use it as the telemetry architecture.

## Boundary

- Telemetry may serialize, send, receive, display, and record state.
- Telemetry must not read hardware devices directly, execute Phase handlers, command motors, decide goal/stop, or own mission recovery.
- Mission execution must continue if telemetry or the PC disappears.
- A future mission integration must be one-way state publication from the mission process to a separate telemetry agent.
- Heavy display, plotting, archival, and retrospective analysis belong on the PC side.

## Transport contract

- Keep normal telemetry as one JSON packet per UDP datagram using schema `cansat.telemetry.v1`.
- Keep sequence numbers so the receiver can infer loss.
- Prefer the newest state over retransmitting obsolete state.
- Keep the default normal-state path lightweight (currently 5 Hz, UDP port 5001, recommended payload maximum 1200 bytes).
- Treat `sent_unix` delay as clock-dependent; use trends, link age, receive rate, and sequence gaps together.
- Report malformed/schema/type failures as receiver events instead of crashing.
- Continue sending and local spooling when the PC is absent.

Source files own the complete packet fields and CLI options; do not duplicate their full schemas here.

## Video separation

- Never put JPEG/base64 frames or large debug text in normal telemetry packets.
- Add future video on a separate process/stream/port with independent backpressure and failure.
- A lost or slow video stream must not delay normal telemetry or mission control.
- Put only small video health state in normal telemetry.

## Logging and failure semantics

- Keep PC receive packets and receiver events in separate JSONL logs.
- Keep SBC spool bounded/rotated and sampled according to its own logging interval.
- Distinguish `WAITING` (never received), `DEGRADED`, `LOST` (previous link expired), bad packet, state-source error, and payload-over-limit.
- Preserve receive timestamp, sender identity, schema/type, sequence, and error reason required to reconstruct link health.
- Do not turn packet loss into a mission control command.

## Change rules

- When changing schema, update both endpoints and compatibility/error behavior in the same change.
- Define unit, missing/invalid representation, update rate, and UI/log use before adding a field.
- Do not enlarge SBC dependencies for a PC-only feature.
- Test PC restart, no listener, malformed input, sequence gaps, stale state file, payload limit, and clean socket/log shutdown.
- Keep user-facing invocation details in `runs/telemetry/README.md`; keep design constraints here.

## AI Checklist

- Is telemetry still optional and one-way relative to mission control?
- Does normal telemetry remain small and independent of video?
- Can PC absence, malformed packets, and link loss occur without stopping the mission?
- Did both endpoints and logs change with the schema?
- Have I avoided assuming the unimplemented mission-state publisher exists?
