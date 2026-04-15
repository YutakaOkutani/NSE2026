import argparse
import json
import math
import socket
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_PC_HOST = "127.0.0.1"
DEFAULT_PC_PORT = 5001
DEFAULT_TX_HZ = 5.0
TELEMETRY_DIR = Path(__file__).resolve().parent
DEFAULT_LOG_DIR = str(TELEMETRY_DIR / "logs" / "telemetry_sbc")
SCHEMA = "cansat.telemetry.v1"
MAX_UDP_PAYLOAD_BYTES = 1200


class RotatingJsonlLogger:
    def __init__(self, path, max_bytes, backups):
        self.path = Path(path)
        self.max_bytes = int(max_bytes)
        self.backups = int(backups)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, record):
        try:
            if self.max_bytes > 0 and self.path.exists() and self.path.stat().st_size >= self.max_bytes:
                self._rotate()
            with self.path.open("a", encoding="utf-8") as file_obj:
                file_obj.write(json.dumps(record, ensure_ascii=True, separators=(",", ":")) + "\n")
        except OSError as exc:
            print(f"log_write_error={exc}", file=sys.stderr)

    def _rotate(self):
        if self.backups <= 0:
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
            return
        for idx in range(self.backups - 1, 0, -1):
            src = self.path.with_name(f"{self.path.name}.{idx}")
            dst = self.path.with_name(f"{self.path.name}.{idx + 1}")
            if src.exists():
                src.replace(dst)
        if self.path.exists():
            self.path.replace(self.path.with_name(f"{self.path.name}.1"))


def monotonic_ms(start_time):
    return int((time.monotonic() - start_time) * 1000)


def read_cpu_temp_c():
    path = Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        raw = path.read_text(encoding="utf-8").strip()
        return round(float(raw) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def compact_json(payload):
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")


def deep_update(base, incoming):
    for key, value in incoming.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_update(base[key], value)
        else:
            base[key] = value
    return base


def read_state_override(path):
    if not path:
        return {}, None
    state_path = Path(path)
    try:
        raw = state_path.read_text(encoding="utf-8")
        if not raw.strip():
            return {}, time.time() - state_path.stat().st_mtime
        return json.loads(raw), time.time() - state_path.stat().st_mtime
    except FileNotFoundError:
        return {"health": {"errors": [f"state_file_missing:{state_path}"]}}, None
    except json.JSONDecodeError as exc:
        return {"health": {"errors": [f"state_file_json_error:{exc.lineno}:{exc.colno}"]}}, None
    except OSError as exc:
        return {"health": {"errors": [f"state_file_read_error:{exc}"]}}, None


def simulated_motion(seq, boot_ms):
    elapsed = boot_ms / 1000.0
    heading = (elapsed * 8.0) % 360.0
    lat = 35.0 + math.sin(elapsed / 30.0) * 0.00005
    lng = 139.0 + math.cos(elapsed / 30.0) * 0.00005
    return {
        "mission": {
            "phase": 0,
            "mode": "TELEMETRY_PHASE_A",
            "reason": "",
        },
        "gps": {
            "lat": round(lat, 7),
            "lng": round(lng, 7),
            "fix": 1,
            "sats": 8,
            "hdop": 1.2,
            "heading_deg": round(heading, 1),
            "heading_valid": True,
            "speed_mps": 0.2,
        },
        "imu": {
            "acc": [0.0, 0.0, 9.81],
            "gyro": [0.0, 0.0, round(math.sin(elapsed) * 0.02, 4)],
            "mag": [20.0, 0.0, 35.0],
            "angle_deg": round(heading, 1),
            "angle_valid": True,
            "fall": 0.0,
        },
        "baro": {
            "alt_m": round(10.0 + math.sin(elapsed / 10.0), 2),
            "pressure_hpa": 1013.25,
        },
        "nav": {
            "distance_m": max(0.0, round(30.0 - elapsed * 0.05, 2)),
            "azimuth_deg": 90.0,
            "direction": 0.5,
        },
        "actuator": {
            "motor_left": 0.0,
            "motor_right": 0.0,
            "last_cmd_age_ms": None,
        },
        "health": {
            "source": "simulated",
            "errors": [],
            "video": "off",
        },
    }


def build_packet(seq, start_time, args, tx_error_count):
    boot_ms = monotonic_ms(start_time)
    packet = {
        "schema": SCHEMA,
        "type": "state",
        "seq": seq,
        "boot_ms": boot_ms,
        "sent_unix": time.time(),
    }

    if args.source == "file":
        override, state_age = read_state_override(args.state_file)
        packet.update(
            {
                "mission": {"phase": None, "mode": "FILE_SOURCE", "reason": ""},
                "gps": {},
                "imu": {},
                "baro": {},
                "nav": {},
                "actuator": {},
                "health": {
                    "source": "file",
                    "state_age_ms": int(state_age * 1000) if state_age is not None else None,
                    "errors": [],
                    "video": "off",
                },
            }
        )
        deep_update(packet, override)
    else:
        packet.update(simulated_motion(seq, boot_ms))

    health = packet.setdefault("health", {})
    errors = health.setdefault("errors", [])
    health["cpu_temp_c"] = read_cpu_temp_c()
    health["tx_error_count"] = tx_error_count
    health["payload_limit_bytes"] = MAX_UDP_PAYLOAD_BYTES
    health["tx_hz"] = args.tx_hz
    health["target"] = f"{args.pc_host}:{args.pc_port}"
    health["agent"] = "runs/telemetry/telemetry_sbc.py"

    body = compact_json(packet)
    if len(body) > MAX_UDP_PAYLOAD_BYTES:
        errors.append(f"payload_over_limit:{len(body)}")
        packet["health"]["payload_bytes"] = len(body)
    else:
        packet["health"]["payload_bytes"] = len(body)
    return packet


def parse_args():
    parser = argparse.ArgumentParser(description="Phase A UDP telemetry sender for CanSat.")
    parser.add_argument("--pc-host", default=DEFAULT_PC_HOST, help="PC receiver IP or hostname")
    parser.add_argument("--pc-port", type=int, default=DEFAULT_PC_PORT, help="PC receiver UDP port")
    parser.add_argument("--tx-hz", type=float, default=DEFAULT_TX_HZ, help="telemetry transmit rate")
    parser.add_argument("--source", choices=["sim", "file"], default="sim", help="state source for Phase A")
    parser.add_argument("--state-file", default=None, help="JSON file used when --source=file")
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR, help="local SBC telemetry spool directory")
    parser.add_argument("--log-every", type=int, default=5, help="write one local JSONL row every N packets")
    parser.add_argument("--max-log-bytes", type=int, default=10 * 1024 * 1024, help="rotate local log at this size")
    parser.add_argument("--log-backups", type=int, default=3, help="number of local rotated logs")
    parser.add_argument("--quiet", action="store_true", help="suppress periodic status output")
    return parser.parse_args()


def main():
    args = parse_args()
    args.tx_hz = max(0.2, min(50.0, float(args.tx_hz)))
    args.log_every = max(1, int(args.log_every))

    run_id = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = Path(args.log_dir) / f"telemetry_sbc_{run_id}.jsonl"
    logger = RotatingJsonlLogger(log_path, args.max_log_bytes, args.log_backups)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    target = (args.pc_host, int(args.pc_port))
    start_time = time.monotonic()
    seq = 0
    tx_error_count = 0
    last_print = 0.0
    interval = 1.0 / args.tx_hz

    print(f"telemetry_sbc: sending UDP to {target[0]}:{target[1]} at {args.tx_hz:.1f} Hz")
    print(f"telemetry_sbc: source={args.source} local_log={log_path}")

    try:
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(0.02, next_send - now))
                continue

            packet = build_packet(seq, start_time, args, tx_error_count)
            body = compact_json(packet)
            try:
                sock.sendto(body, target)
            except OSError as exc:
                tx_error_count += 1
                packet.setdefault("health", {}).setdefault("errors", []).append(f"send_error:{exc}")

            if seq % args.log_every == 0:
                logger.write(packet)

            if not args.quiet and now - last_print >= 1.0:
                print(
                    "telemetry_sbc: "
                    f"seq={seq} bytes={len(body)} tx_errors={tx_error_count} "
                    f"cpu_temp={packet.get('health', {}).get('cpu_temp_c')}"
                )
                last_print = now

            seq += 1
            next_send += interval
            if next_send < time.monotonic() - interval:
                next_send = time.monotonic() + interval
    except KeyboardInterrupt:
        print("\ntelemetry_sbc: stopped by Ctrl+C")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
