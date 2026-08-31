from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def default_data_root() -> Path:
    """Return the per-user runtime data root used by production and tools."""
    return Path.home() / "cansat-data"


DEFAULT_DATA_ROOT = default_data_root()
DEFAULT_RUNS_ROOT = DEFAULT_DATA_ROOT / "runs"
DEFAULT_DEBUG_ROOT = DEFAULT_DATA_ROOT / "debug"
DEFAULT_CAPTURE_ROOT = DEFAULT_DATA_ROOT / "captures"
DEFAULT_TELEMETRY_ROOT = DEFAULT_DATA_ROOT / "telemetry"

# Detector references are versioned control inputs. Generated captures belong
# under the runtime data root and must never be loaded implicitly as references.
ROI_REFERENCE_DIR = REPO_ROOT / "assets" / "roi"
ROI_CAPTURE_DIR = DEFAULT_CAPTURE_ROOT / "roi"
ROI_PRIMARY_REFERENCE = ROI_REFERENCE_DIR / "captured_roi_img.png"

RUN_CONTEXT_PATH = REPO_ROOT / "run-context.toml"
RUN_CONTEXT_EXAMPLE_PATH = REPO_ROOT / "run-context.toml.example"

