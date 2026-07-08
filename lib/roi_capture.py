import argparse
import shutil
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mission.const import ROI_CAPTURE_DIR, ROI_PATH_1


def build_parser():
    parser = argparse.ArgumentParser(
        description="Capture ROI reference images used by the production cone detector."
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of reference images to capture.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Seconds to wait between captures when --count is greater than 1.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="Camera warmup seconds before the first capture.",
    )
    parser.add_argument("--width", type=int, default=640, help="Capture width used by the detector.")
    parser.add_argument("--height", type=int, default=480, help="Capture height used by the detector.")
    parser.add_argument(
        "--outdir",
        default=ROI_CAPTURE_DIR,
        help=f"ROI output directory. Default: {ROI_CAPTURE_DIR}",
    )
    parser.add_argument(
        "--prefix",
        default="captured_roi_img",
        help="Archive filename prefix. Use false_ or fake_ for negative references.",
    )
    parser.add_argument(
        "--no-latest",
        action="store_true",
        help=f"Do not update the latest ROI file: {ROI_PATH_1}",
    )
    return parser


def main():
    args = build_parser().parse_args()
    if args.count <= 0:
        print("ERROR: --count must be >= 1")
        return 2
    if args.width <= 0 or args.height <= 0:
        print("ERROR: --width/--height must be >= 1")
        return 2

    outdir = Path(args.outdir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    latest_path = Path(ROI_PATH_1) if outdir == Path(ROI_CAPTURE_DIR) else outdir / "captured_roi_img.png"

    picam2 = None
    try:
        from picamera2 import Picamera2

        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (args.width, args.height), "format": "BGR888"}
        )
        picam2.configure(config)
        picam2.start()

        print(f"ROI capture started: {args.width}x{args.height}, warmup={args.warmup:.1f}s")
        print(f"ROI directory: {outdir}")
        time.sleep(max(0.0, args.warmup))

        ts = time.strftime("%Y%m%d_%H%M%S")
        saved_paths = []
        for idx in range(args.count):
            suffix = f"_{idx + 1:03d}" if args.count > 1 else ""
            archive_path = outdir / f"{args.prefix}_{ts}{suffix}.png"
            picam2.capture_file(str(archive_path))
            saved_paths.append(archive_path)
            print(f"[{idx + 1}/{args.count}] archived: {archive_path}")

            if idx + 1 < args.count and args.interval > 0:
                time.sleep(args.interval)

        prefix_lower = args.prefix.lower()
        negative_prefix = any(token in prefix_lower for token in ("false_", "fake_", "grass", "sky", "bg_"))
        if negative_prefix and not args.no_latest:
            print("Latest ROI not updated because the prefix is classified as a negative reference.")
        elif not args.no_latest and saved_paths:
            latest_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(saved_paths[-1], latest_path)
            print(f"Latest ROI updated: {latest_path}")

        print("Done.")
        return 0
    except Exception as exc:
        print(f"ROI capture failed: {exc}")
        return 1
    finally:
        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
            try:
                picam2.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
