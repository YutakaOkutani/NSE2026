import argparse
import csv
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import (
    CONE_PHASE4_CENTER_TOLERANCE,
    CONE_PHASE4_CONFIRM_FRAMES,
    CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE,
    CONE_PHASE4_REACHED_PROBABILITY_THRESHOLD,
    CONE_PHASE5_REACH_CONFIRM_FRAMES,
    CONE_PHASE5_REACHED_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
)
from mission.mgr.hw_mgr import HardwareManager
from lib import cone_detect as dc
from lib.cone_diagnostics import (
    CONE_DIAGNOSTIC_KEYS,
    detector_diagnostics,
    normalize_cone_diagnostics,
)

DEFAULT_OUT_BASE = Path("/home/pi/logs_nse2026")
WINDOW_NAME = "Detector Debug (Production cone_detect.py)"

DEBUG_CSV_BASE_FIELDS = [
    "frame",
    "time_sec",
    "probability",
    "is_detected_by_phase_threshold",
    "is_reached",
    "occupancy",
    "frame_red_occupancy",
    "cone_direction",
    "method",
    "shape_score",
    "cone_shape_score",
    "sv_score",
    "hue_redness_score",
    "roi_support_ratio",
]
DEBUG_CSV_DIAGNOSTIC_FIELDS = [
    "schema_version",
    "valid",
    "status",
    "sequence",
    "phase",
    "phase_threshold",
    "phase_reached_probability_threshold",
    "phase_center_tolerance",
    "phase_direction_tolerance",
    "phase_required_confirm_frames",
    "is_reached_effective_by_phase",
    "is_phase_candidate",
    "is_detected_internal",
] + [
    key
    for key in CONE_DIAGNOSTIC_KEYS
    if key
    not in {
        "schema_version",
        "valid",
        "status",
        "sequence",
        "direction",
        "probability",
        "method",
        "is_detected",
        "is_reached",
        "occupancy",
        "frame_red_occupancy",
        "shape_score",
        "cone_shape_score",
        "sv_score",
        "hue_redness_score",
        "roi_support_ratio",
    }
]
DEBUG_CSV_FIELDS = DEBUG_CSV_BASE_FIELDS + DEBUG_CSV_DIAGNOSTIC_FIELDS


class _RoiLoader(HardwareManager):
    pass


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Run production cone detector on live camera and dump debug artifacts. "
            "This imports the same detector code used in mission runtime."
        )
    )
    parser.add_argument("--phase", type=int, choices=[4, 5], default=4, help="Phase threshold preset (4 or 5).")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Number of frames to process. 0 means run until 'q' key.",
    )
    parser.add_argument("--sleep", type=float, default=0.05, help="Sleep seconds between frames.")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable GUI window. Artifacts and CSV logs are still saved.",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=1,
        help="Save image artifacts every N frames.",
    )
    parser.add_argument(
        "--outdir",
        type=str,
        default="",
        help="Output directory. Default: /home/pi/logs_nse2026/camera_debug/<timestamp>.",
    )
    return parser


def _make_run_outdir(base_dir: Path) -> Path:
    ts = time.strftime("%Y%m%d_%H%M%S")
    subsec = int((time.time() % 1.0) * 1000)
    camera_debug_dir = base_dir / "camera_debug"
    outdir = camera_debug_dir / f"{ts}_{subsec:03d}"
    suffix = 0
    while outdir.exists():
        suffix += 1
        outdir = camera_debug_dir / f"{ts}_{subsec:03d}_{suffix:02d}"
    outdir.mkdir(parents=True, exist_ok=False)
    return outdir


def _display_available() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _debug_csv_record(detector, frame_idx, elapsed_sec, phase, phase_threshold, *, valid, status):
    if detector is None:
        diag = normalize_cone_diagnostics(
            {"valid": int(bool(valid)), "status": status, "sequence": frame_idx}
        )
    else:
        diag = detector_diagnostics(
            detector,
            valid=valid,
            status=status,
            sequence=frame_idx,
        )
    record = {
        "frame": frame_idx,
        "time_sec": elapsed_sec,
        "probability": diag["probability"],
        "is_detected_by_phase_threshold": int(diag["probability"] > phase_threshold),
        "is_reached": diag["is_reached"],
        "occupancy": diag["occupancy"],
        "frame_red_occupancy": diag["frame_red_occupancy"],
        "cone_direction": diag["direction"],
        "method": diag["method"],
        "shape_score": diag["shape_score"],
        "cone_shape_score": diag["cone_shape_score"],
        "sv_score": diag["sv_score"],
        "hue_redness_score": diag["hue_redness_score"],
        "roi_support_ratio": diag["roi_support_ratio"],
        "phase": phase,
        "phase_threshold": phase_threshold,
        "phase_reached_probability_threshold": max(
            phase_threshold,
            (
                CONE_PHASE4_REACHED_PROBABILITY_THRESHOLD
                if phase == 4
                else CONE_PHASE5_REACHED_PROBABILITY_THRESHOLD
            ),
        ),
        "phase_center_tolerance": CONE_PHASE4_CENTER_TOLERANCE if phase == 4 else 0.0,
        "phase_direction_tolerance": (
            CONE_PHASE4_DIRECTION_CONSISTENCY_TOLERANCE if phase == 4 else 0.0
        ),
        "phase_required_confirm_frames": (
            CONE_PHASE4_CONFIRM_FRAMES if phase == 4 else CONE_PHASE5_REACH_CONFIRM_FRAMES
        ),
        "is_detected_internal": diag["is_detected"],
    }
    reached_threshold = record["phase_reached_probability_threshold"]
    record["is_reached_effective_by_phase"] = int(
        bool(diag["is_reached"]) and diag["probability"] > reached_threshold
    )
    centered = abs(diag["direction"] - 0.5) <= float(CONE_PHASE4_CENTER_TOLERANCE)
    record["is_phase_candidate"] = int(
        diag["probability"] > phase_threshold and (phase != 4 or centered)
    )
    for key in DEBUG_CSV_DIAGNOSTIC_FIELDS:
        if key not in record:
            record[key] = diag.get(key, "")
    return record


def _overlay(detector, prob_thresh, frame_idx):
    img = detector.input_img
    if img is None:
        return None
    vis = img.copy()
    h, w = vis.shape[:2]

    bbox = getattr(detector, "detected", None)
    if bbox is not None:
        x, y, bw, bh = [int(v) for v in bbox]
        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

    cent = getattr(detector, "centroids", None)
    if cent is not None:
        cx, cy = int(cent[0]), int(cent[1])
        cv2.circle(vis, (cx, cy), 5, (0, 255, 255), -1)

    prob = float(getattr(detector, "probability", 0.0))
    is_reached = bool(getattr(detector, "is_reached", False))
    method = str(getattr(detector, "debug_method", "unknown"))
    detected = prob > prob_thresh
    status = f"DET={int(detected)} REACH={int(is_reached)} p={prob:.3f} thr={prob_thresh:.2f} f={frame_idx}"
    cv2.putText(vis, status, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (0, 255, 0) if detected else (0, 0, 255), 2)
    cv2.putText(vis, method, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 1)

    if detector.binarized_img is not None:
        mask = detector.binarized_img.astype(np.uint8)
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_rgb, (w // 3, h // 3))
        mh, mw = mask_small.shape[:2]
        vis[0:mh, w - mw : w] = mask_small
    return vis


def main():
    args = build_parser().parse_args()
    prob_thresh = (
        float(CONE_PROBABILITY_THRESHOLD_PHASE4)
        if args.phase == 4
        else float(CONE_PROBABILITY_THRESHOLD_PHASE5)
    )

    outdir = Path(args.outdir).expanduser().resolve() if args.outdir else _make_run_outdir(DEFAULT_OUT_BASE)
    if args.outdir:
        outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / "debug.csv"

    detector = dc.detector()
    detector.capture_reached_path = str(outdir / "capture_reached.png")
    try:
        roi_loader = _RoiLoader()
        roi_images = roi_loader._load_roi_images()
        roi_input = roi_images if roi_images else None
        detector.set_roi_img(roi_input)
    except Exception as exc:
        print(f"ROI setup warning: {exc}")

    gui_enabled = (not args.headless) and _display_available()
    if (not args.headless) and (not gui_enabled):
        print("GUI disabled: DISPLAY/WAYLAND_DISPLAY is not set. Running in save-only mode.")
    if gui_enabled:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    frame_idx = 0
    detect_count = 0
    reached_count = 0
    t_start = time.time()

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=DEBUG_CSV_FIELDS)
        writer.writeheader()

        while True:
            frame_idx += 1
            ok = detector.detect_cone()
            now = time.time()
            if not ok:
                print("camera capture failed")
                writer.writerow(
                    _debug_csv_record(
                        None,
                        frame_idx,
                        now - t_start,
                        args.phase,
                        prob_thresh,
                        valid=False,
                        status="camera_capture_failed",
                    )
                )
                f.flush()
                time.sleep(max(0.0, args.sleep))
                if args.frames > 0 and frame_idx >= args.frames:
                    break
                continue

            prob = float(detector.probability)
            is_reached = bool(detector.is_reached)
            is_det = prob > prob_thresh
            detect_count += 1 if is_det else 0
            reached_count += 1 if is_reached else 0

            writer.writerow(
                _debug_csv_record(
                    detector,
                    frame_idx,
                    now - t_start,
                    args.phase,
                    prob_thresh,
                    valid=True,
                    status="ok",
                )
            )
            f.flush()

            if frame_idx % max(1, args.save_every) == 0:
                stem = f"frame_{frame_idx:06d}"
                if detector.input_img is not None:
                    cv2.imwrite(str(outdir / f"{stem}_input.png"), detector.input_img)
                if detector.projected_img is not None:
                    proj = detector.projected_img
                    if proj.ndim == 2:
                        cv2.imwrite(str(outdir / f"{stem}_projected.png"), proj.astype(np.uint8))
                    else:
                        cv2.imwrite(str(outdir / f"{stem}_projected.png"), proj)
                if detector.binarized_img is not None:
                    cv2.imwrite(str(outdir / f"{stem}_mask.png"), detector.binarized_img.astype(np.uint8))

            if gui_enabled:
                vis = _overlay(detector, prob_thresh, frame_idx)
                if vis is not None:
                    cv2.imshow(WINDOW_NAME, vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    stem = f"manual_{frame_idx:06d}"
                    if detector.input_img is not None:
                        cv2.imwrite(str(outdir / f"{stem}_input.png"), detector.input_img)
                    if detector.binarized_img is not None:
                        cv2.imwrite(str(outdir / f"{stem}_mask.png"), detector.binarized_img.astype(np.uint8))

            if args.frames > 0 and frame_idx >= args.frames:
                break
            time.sleep(max(0.0, args.sleep))

    elapsed = max(1e-6, time.time() - t_start)
    print(f"Output dir: {outdir}")
    print(f"CSV: {csv_path}")
    print(
        f"Frames={frame_idx}, Detect(phase{args.phase})={detect_count} ({100.0 * detect_count / max(1, frame_idx):.1f}%), "
        f"Reached={reached_count}, FPS={frame_idx / elapsed:.2f}"
    )

    try:
        detector.close()
    except Exception:
        pass
    if gui_enabled:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
