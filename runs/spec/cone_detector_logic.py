import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONE_DETECT_PATH = PROJECT_ROOT / "lib" / "cone_detect.py"


def _load_detector_class():
    fake_cv2 = types.SimpleNamespace()
    fake_numpy = types.SimpleNamespace()
    fake_picamera2 = types.SimpleNamespace(Picamera2=object)
    spec = importlib.util.spec_from_file_location(
        "cone_detect_logic_under_test",
        CONE_DETECT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(
        sys.modules,
        {
            "cv2": fake_cv2,
            "numpy": fake_numpy,
            "picamera2": fake_picamera2,
        },
    ):
        spec.loader.exec_module(module)
    return module.detector


Detector = _load_detector_class()


class ConeDetectorLogicTest(unittest.TestCase):
    def test_roi_support_ratio_is_bounded(self):
        ratio = Detector._detector__bounded_support_ratio(15, 10)

        self.assertEqual(ratio, 1.0)
        self.assertEqual(Detector._detector__bounded_support_ratio(0, 0), 0.0)

    def test_small_lower_rescue_never_promotes_relaxed_color(self):
        detector = object.__new__(Detector)
        detector.small_lower_probability_cap = 0.24

        probability, rescued = detector._detector__small_lower_rescue(
            0.17,
            strict_red_ok=False,
            bbox_bottom_frac=0.90,
            occupancy=0.01,
            cone_shape_score=0.70,
            hue_redness_score=0.75,
            sv_score=0.26,
        )

        self.assertFalse(rescued)
        self.assertEqual(probability, 0.17)

    def test_small_lower_rescue_remains_available_for_strict_cone(self):
        detector = object.__new__(Detector)
        detector.small_lower_probability_cap = 0.24

        probability, rescued = detector._detector__small_lower_rescue(
            0.19,
            strict_red_ok=True,
            bbox_bottom_frac=0.90,
            occupancy=0.01,
            cone_shape_score=0.70,
            hue_redness_score=0.75,
            sv_score=0.30,
        )

        self.assertTrue(rescued)
        self.assertEqual(probability, 0.24)

    def test_small_fragment_requires_roi_or_strong_color_support(self):
        detector = object.__new__(Detector)
        detector.min_component_occupancy = 0.001
        fragment = {
            "occupancy": 0.0003,
            "cone_shape_score": 0.40,
            "hue_redness_score": 0.52,
            "sv_score": 0.35,
        }

        self.assertFalse(
            detector._detector__proposal_is_eligible(fragment, 0.02, "hue")
        )
        self.assertTrue(
            detector._detector__proposal_is_eligible(fragment, 0.22, "hue")
        )
        self.assertTrue(
            detector._detector__proposal_is_eligible(
                fragment,
                0.0,
                "hybrid_overlap",
            )
        )

    def test_hybrid_candidate_receives_mode_preference(self):
        component = {"score": 0.30, "occupancy": 0.01}

        hue_score = Detector._detector__candidate_mode_score(
            component,
            0.5,
            "hue",
        )
        hybrid_score = Detector._detector__candidate_mode_score(
            component,
            0.5,
            "hybrid_overlap",
        )

        self.assertGreater(hybrid_score, hue_score)

    def test_temporal_candidate_bonus_prefers_continuous_track(self):
        detector = object.__new__(Detector)
        detector.camera_width = 640
        detector._candidate_track_direction = 0.25
        detector._candidate_track_height = 100.0
        detector._candidate_track_mode = "hue"
        detector._candidate_track_at = 10.0
        near = {"centroid": [180, 200], "bbox": [150, 100, 60, 110]}
        far = {"centroid": [600, 200], "bbox": [570, 100, 60, 110]}

        near_bonus = detector._detector__candidate_temporal_bonus(
            near,
            "hue",
            10.2,
        )
        far_bonus = detector._detector__candidate_temporal_bonus(
            far,
            "hue",
            10.2,
        )

        self.assertGreater(near_bonus, far_bonus)


if __name__ == "__main__":
    unittest.main()
