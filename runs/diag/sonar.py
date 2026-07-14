import argparse
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    OBSTACLE_AVOID_DIST,
    PIN_ECHO,
    PIN_TRIG,
    SONAR_MAX_DISTANCE,
    SONAR_STALE_TIMEOUT_SEC,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Ultrasonic distance sensor diagnostic")
    parser.add_argument("--interval", type=float, default=0.2, help="Polling interval in seconds")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds; 0 runs until Ctrl+C",
    )
    return parser.parse_args()


def _valid_distance_cm(distance_m):
    try:
        distance_m = float(distance_m)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(distance_m) or not 0.0 < distance_m < float(SONAR_MAX_DISTANCE):
        return None
    return distance_m * 100.0


def main():
    args = parse_args()
    interval = max(0.05, float(args.interval))
    duration = max(0.0, float(args.duration))

    from gpiozero import DistanceSensor
    from gpiozero.pins.lgpio import LGPIOFactory

    print("Ultrasonic sensor diagnostic")
    print(f"  trigger GPIO : {PIN_TRIG}")
    print(f"  echo GPIO    : {PIN_ECHO}")
    print(f"  max distance : {SONAR_MAX_DISTANCE:.1f} m")
    print(f"  stale limit  : {SONAR_STALE_TIMEOUT_SEC:.1f} s")
    print(f"  avoid limit  : {OBSTACLE_AVOID_DIST:.1f} cm")
    print("CAUTION: standard HC-SR04 ECHO is 5 V; never connect it directly to Raspberry Pi GPIO.")
    print("CAUTION: the current PCB has no level conversion; add an external divider or level shifter.")
    print("Place a flat object in front of the sensor and move it. Ctrl+C to exit.")

    pin_factory = None
    sensor = None
    valid_count = 0
    invalid_count = 0
    consecutive_invalid = 0
    last_valid_at = None
    last_valid_cm = None
    started_at = time.monotonic()

    try:
        pin_factory = LGPIOFactory()
        sensor = DistanceSensor(
            echo=PIN_ECHO,
            trigger=PIN_TRIG,
            max_distance=SONAR_MAX_DISTANCE,
            pin_factory=pin_factory,
        )

        while duration <= 0.0 or time.monotonic() - started_at < duration:
            now = time.monotonic()
            try:
                distance_cm = _valid_distance_cm(sensor.distance)
            except Exception as exc:
                distance_cm = None
                error_detail = f"{type(exc).__name__}: {exc}"
            else:
                error_detail = "no valid echo"

            timestamp = time.strftime("%H:%M:%S")
            if distance_cm is not None:
                valid_count += 1
                consecutive_invalid = 0
                last_valid_at = now
                last_valid_cm = distance_cm
                obstacle = distance_cm < float(OBSTACLE_AVOID_DIST)
                print(
                    f"[{timestamp}] VALID distance={distance_cm:7.2f} cm "
                    f"obstacle={int(obstacle)}"
                )
            else:
                invalid_count += 1
                consecutive_invalid += 1
                stale_sec = now - last_valid_at if last_valid_at is not None else float("inf")
                production_valid = last_valid_at is not None and stale_sec < float(SONAR_STALE_TIMEOUT_SEC)
                last_text = f"{last_valid_cm:.2f} cm" if last_valid_cm is not None else "none"
                stale_text = f"{stale_sec:.2f}s" if math.isfinite(stale_sec) else "never"
                state = "HOLD" if production_valid else "STALE"
                print(
                    f"[{timestamp}] {state} last={last_text} age={stale_text} "
                    f"failures={consecutive_invalid} ({error_detail})"
                )

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nDiagnostic stopped by user.")
    finally:
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass
        if pin_factory is not None:
            try:
                pin_factory.close()
            except Exception:
                pass

    print(f"Summary: valid={valid_count}, invalid={invalid_count}")
    if valid_count == 0:
        print("No valid echo was received. Check 5V/GND, GPIO wiring, target angle, and distance.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
