import importlib.util
import math
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SONAR_DIAG_PATH = PROJECT_ROOT / "runs" / "diag" / "sonar.py"

spec = importlib.util.spec_from_file_location("sonar_diag_under_test", SONAR_DIAG_PATH)
sonar_diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sonar_diag)


class SonarDiagnosticTest(unittest.TestCase):
    def test_valid_distance_is_converted_to_centimeters(self):
        self.assertAlmostEqual(sonar_diag._valid_distance_cm(0.25), 25.0)
        self.assertAlmostEqual(
            sonar_diag._valid_distance_cm(sonar_diag.SONAR_MAX_DISTANCE - 0.01),
            (sonar_diag.SONAR_MAX_DISTANCE - 0.01) * 100.0,
        )

    def test_invalid_or_out_of_range_distance_is_rejected(self):
        invalid_values = (
            None,
            "not-a-number",
            math.nan,
            math.inf,
            -0.01,
            0.0,
            sonar_diag.SONAR_MAX_DISTANCE,
            sonar_diag.SONAR_MAX_DISTANCE + 0.01,
        )
        for value in invalid_values:
            with self.subTest(value=value):
                self.assertIsNone(sonar_diag._valid_distance_cm(value))


if __name__ == "__main__":
    unittest.main()
