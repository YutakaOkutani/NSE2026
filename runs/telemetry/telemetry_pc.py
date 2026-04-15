import argparse
import json
import socket
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5001
TELEMETRY_DIR = Path(__file__).resolve().parent
DEFAULT_LOG_DIR = str(TELEMETRY_DIR / "logs" / "telemetry_pc")
EXPECTED_SCHEMA = "cansat.telemetry.v1"


class JsonlRunLogger:
    def __init__(self, root_dir):
        run_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_dir = Path(root_dir) / run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.telemetry_path = self.run_dir / "telemetry.jsonl"
        self.events_path = self.run_dir / "events.jsonl"

    def write_packet(self, packet, addr, recv_unix):
        record = {
            "recv_unix": recv_unix,
            "addr": f"{addr[0]}:{addr[1]}",
            "packet": packet,
        }
        self._append(self.telemetry_path, record)

    def write_event(self, event, **fields):
        record = {"event_unix": time.time(), "event": event}
        record.update(fields)
        self._append(self.events_path, record)

    @staticmethod
    def _append(path, record):
        try:
            with path.open("a", encoding="utf-8") as file_obj:
                file_obj.write(json.dumps(record, ensure_ascii=True, separators=(",", ":")) + "\n")
        except OSError as exc:
            print(f"log_write_error={exc}", file=sys.stderr)


class LinkStats:
    def __init__(self, window_sec):
        self.window_sec = float(window_sec)
        self.arrivals = deque()
        self.last_seq = None
        self.expected = 0
        self.missed = 0
        self.received = 0
        self.bad_packets = 0
        self.last_packet = None
        self.last_addr = None
        self.last_recv_unix = 0.0
        self.last_delay_ms = None
        self.last_event_state = "INIT"

    def update_packet(self, packet, addr, recv_unix):
        self.received += 1
        self.last_packet = packet
        self.last_addr = addr
        self.last_recv_unix = recv_unix
        sent_unix = packet.get("sent_unix")
        if isinstance(sent_unix, (int, float)):
            self.last_delay_ms = max(0.0, (recv_unix - float(sent_unix)) * 1000.0)
        else:
            self.last_delay_ms = None

        seq = packet.get("seq")
        if isinstance(seq, int):
            if self.last_seq is not None:
                gap = seq - self.last_seq
                if gap > 1:
                    self.missed += gap - 1
                self.expected += max(1, gap)
            else:
                self.expected += 1
            self.last_seq = seq

        self.arrivals.append(recv_unix)
        self._trim(recv_unix)

    def update_bad_packet(self):
        self.bad_packets += 1

    def _trim(self, now):
        while self.arrivals and now - self.arrivals[0] > self.window_sec:
            self.arrivals.popleft()

    def rx_hz(self, now):
        self._trim(now)
        if self.last_recv_unix <= 0.0 or now - self.last_recv_unix > self.window_sec:
            return 0.0
        if len(self.arrivals) < 2:
            return 0.0
        duration = max(0.001, self.arrivals[-1] - self.arrivals[0])
        return (len(self.arrivals) - 1) / duration

    def loss_percent(self):
        if self.expected <= 0:
            return 0.0
        return 100.0 * self.missed / float(self.expected)

    def link_age(self, now):
        if self.last_recv_unix <= 0.0:
            return None
        return now - self.last_recv_unix

    def link_state(self, now):
        age = self.link_age(now)
        if age is None:
            return "WAITING"
        if age < 1.0:
            return "OK"
        if age < 3.0:
            return "DEGRADED"
        return "LOST"


def clear_screen():
    sys.stdout.write("\033[2J\033[H")


def fmt_value(value, precision=2, missing="-"):
    if value is None:
        return missing
    if isinstance(value, float):
        return f"{value:.{precision}f}"
    return str(value)


def nested(packet, *keys, default=None):
    cur = packet
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def render_dashboard(stats, logger, now):
    packet = stats.last_packet or {}
    age = stats.link_age(now)
    state = stats.link_state(now)
    delay = stats.last_delay_ms
    addr = f"{stats.last_addr[0]}:{stats.last_addr[1]}" if stats.last_addr else "-"
    errors = nested(packet, "health", "errors", default=[]) or []

    lines = [
        "CanSat Telemetry Ground Station - Phase A",
        "=" * 48,
        "[LINK]",
        f"status: {state}",
        f"age: {fmt_value(age)} s",
        f"rx: {stats.rx_hz(now):.2f} Hz",
        f"loss: {stats.loss_percent():.1f} %",
        f"last_seq: {fmt_value(packet.get('seq'), 0)}",
        f"delay: {fmt_value(delay, 1)} ms",
        f"source: {addr}",
        "",
        "[MISSION]",
        f"phase: {fmt_value(nested(packet, 'mission', 'phase'))}",
        f"mode: {fmt_value(nested(packet, 'mission', 'mode'))}",
        f"reason: {fmt_value(nested(packet, 'mission', 'reason'))}",
        f"boot_ms: {fmt_value(packet.get('boot_ms'), 0)}",
        "",
        "[GPS]",
        f"fix: {fmt_value(nested(packet, 'gps', 'fix'))}",
        f"lat/lng: {fmt_value(nested(packet, 'gps', 'lat'), 7)} / {fmt_value(nested(packet, 'gps', 'lng'), 7)}",
        f"sats: {fmt_value(nested(packet, 'gps', 'sats'))}",
        f"hdop: {fmt_value(nested(packet, 'gps', 'hdop'))}",
        f"speed: {fmt_value(nested(packet, 'gps', 'speed_mps'))} m/s",
        f"heading: {fmt_value(nested(packet, 'gps', 'heading_deg'), 1)} valid={fmt_value(nested(packet, 'gps', 'heading_valid'))}",
        "",
        "[IMU]",
        f"angle: {fmt_value(nested(packet, 'imu', 'angle_deg'), 1)} valid={fmt_value(nested(packet, 'imu', 'angle_valid'))}",
        f"acc: {fmt_value(nested(packet, 'imu', 'acc'))}",
        f"gyro: {fmt_value(nested(packet, 'imu', 'gyro'))}",
        f"fall: {fmt_value(nested(packet, 'imu', 'fall'))}",
        "",
        "[NAV / ACTUATOR]",
        f"distance: {fmt_value(nested(packet, 'nav', 'distance_m'))} m",
        f"azimuth: {fmt_value(nested(packet, 'nav', 'azimuth_deg'), 1)} deg",
        f"direction: {fmt_value(nested(packet, 'nav', 'direction'))}",
        f"motor L/R: {fmt_value(nested(packet, 'actuator', 'motor_left'))} / {fmt_value(nested(packet, 'actuator', 'motor_right'))}",
        f"last_cmd_age: {fmt_value(nested(packet, 'actuator', 'last_cmd_age_ms'), 0)} ms",
        "",
        "[HEALTH]",
        f"cpu_temp: {fmt_value(nested(packet, 'health', 'cpu_temp_c'), 1)} C",
        f"state_age: {fmt_value(nested(packet, 'health', 'state_age_ms'), 0)} ms",
        f"payload: {fmt_value(nested(packet, 'health', 'payload_bytes'), 0)} bytes",
        f"video: {fmt_value(nested(packet, 'health', 'video'))}",
        f"errors: {', '.join(errors) if errors else 'none'}",
        "",
        "[LOG]",
        f"run_dir: {logger.run_dir}",
        f"bad_packets: {stats.bad_packets}",
    ]
    clear_screen()
    print("\n".join(lines), flush=True)


def validate_packet(packet):
    if not isinstance(packet, dict):
        return "not_object"
    if packet.get("schema") != EXPECTED_SCHEMA:
        return f"schema_mismatch:{packet.get('schema')}"
    if packet.get("type") != "state":
        return f"type_mismatch:{packet.get('type')}"
    if not isinstance(packet.get("seq"), int):
        return "missing_seq"
    return None


def parse_args():
    parser = argparse.ArgumentParser(description="Phase A UDP telemetry receiver for CanSat.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="UDP bind host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP bind port")
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR, help="PC telemetry log root")
    parser.add_argument("--window-sec", type=float, default=10.0, help="RX rate window")
    parser.add_argument("--ui-hz", type=float, default=1.0, help="dashboard refresh rate")
    parser.add_argument("--no-clear", action="store_true", help="do not clear terminal before dashboard redraw")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.no_clear:
        globals()["clear_screen"] = lambda: None

    logger = JsonlRunLogger(args.log_dir)
    stats = LinkStats(window_sec=args.window_sec)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, int(args.port)))
    sock.settimeout(0.2)
    logger.write_event("receiver_start", host=args.host, port=args.port, run_dir=str(logger.run_dir))
    print(f"telemetry_pc: listening on {args.host}:{args.port}")
    print(f"telemetry_pc: log_dir={logger.run_dir}")

    last_render = 0.0
    last_link_state = stats.link_state(time.time())
    render_interval = 1.0 / max(0.2, min(20.0, args.ui_hz))

    try:
        while True:
            now = time.time()
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                data = None
                addr = None

            if data is not None:
                recv_unix = time.time()
                try:
                    packet = json.loads(data.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    stats.update_bad_packet()
                    logger.write_event("bad_packet", reason=str(exc), addr=f"{addr[0]}:{addr[1]}")
                else:
                    validation_error = validate_packet(packet)
                    if validation_error:
                        stats.update_bad_packet()
                        logger.write_event("invalid_packet", reason=validation_error, addr=f"{addr[0]}:{addr[1]}")
                    else:
                        stats.update_packet(packet, addr, recv_unix)
                        logger.write_packet(packet, addr, recv_unix)

            state = stats.link_state(time.time())
            if state != last_link_state:
                logger.write_event("link_state_change", previous=last_link_state, current=state)
                last_link_state = state

            if time.time() - last_render >= render_interval:
                render_dashboard(stats, logger, time.time())
                last_render = time.time()
    except KeyboardInterrupt:
        print("\ntelemetry_pc: stopped by Ctrl+C")
        logger.write_event("receiver_stop", reason="CTRL_C")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
