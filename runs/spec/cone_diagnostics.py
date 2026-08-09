import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lib.cone_diagnostics import (
    CONE_DIAGNOSTIC_KEYS,
    CONE_DIAGNOSTIC_LOG_COLUMNS,
    CONE_DIAGNOSTIC_SCHEMA_VERSION,
    detector_diagnostics,
    mission_log_values,
    normalize_cone_diagnostics,
)


class _FakeDetector:
    cone_direction = 0.73
    probability = 0.08
    debug_method = "as_is:close_red_region"
    is_detected = True
    is_reached = True
    occupancy = 0.45
    frame_red_occupancy = 0.45
    debug_scores = {
        "schema_version": CONE_DIAGNOSTIC_SCHEMA_VERSION,
        "raw_probability": 0.08,
        "candidate_probability": 0.62,
        "pre_filter_probability": 1.0,
        "close_reached_ok": 1,
        "penalty_flags": "strict_red_x0.08",
        "image_direction": 0.27,
        "image_direction_valid": 1,
        "candidate_count": 7,
    }


class ConeDiagnosticsTest(unittest.TestCase):
    def test_schema_is_complete_unique_and_finite_by_default(self):
        diagnostics = normalize_cone_diagnostics()

        self.assertEqual(tuple(diagnostics), CONE_DIAGNOSTIC_KEYS)
        self.assertEqual(len(CONE_DIAGNOSTIC_KEYS), len(set(CONE_DIAGNOSTIC_KEYS)))
        self.assertEqual(len(CONE_DIAGNOSTIC_LOG_COLUMNS), len(set(CONE_DIAGNOSTIC_LOG_COLUMNS)))
        self.assertEqual(diagnostics["schema_version"], CONE_DIAGNOSTIC_SCHEMA_VERSION)
        for value in diagnostics.values():
            if isinstance(value, float):
                self.assertTrue(math.isfinite(value))

    def test_detector_snapshot_preserves_probability_stages_and_flags(self):
        diagnostics = detector_diagnostics(_FakeDetector(), sequence=7)

        self.assertEqual(diagnostics["sequence"], 7)
        self.assertEqual(diagnostics["probability"], 0.08)
        self.assertEqual(diagnostics["candidate_probability"], 0.62)
        self.assertEqual(diagnostics["pre_filter_probability"], 1.0)
        self.assertEqual(diagnostics["close_reached_ok"], 1)
        self.assertEqual(diagnostics["penalty_flags"], "strict_red_x0.08")
        self.assertEqual(diagnostics["image_direction"], 0.27)
        self.assertEqual(diagnostics["image_direction_valid"], 1)
        self.assertEqual(diagnostics["candidate_count"], 7)

    def test_mission_values_follow_shared_schema_order(self):
        diagnostics = detector_diagnostics(_FakeDetector(), sequence=3)
        values = mission_log_values(diagnostics)
        by_name = dict(zip(CONE_DIAGNOSTIC_LOG_COLUMNS, values))

        self.assertEqual(len(values), len(CONE_DIAGNOSTIC_LOG_COLUMNS))
        self.assertEqual(by_name["ConeSeq"], 3)
        self.assertEqual(by_name["ConeProb"], "0.080000")
        self.assertEqual(by_name["ConePenaltyFlags"], "strict_red_x0.08")


if __name__ == "__main__":
    unittest.main()
