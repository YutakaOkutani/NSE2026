import fnmatch
import unittest
from pathlib import Path

from mission.const import ROI_CAPTURE_DIR, ROI_GLOB_PATTERNS, ROI_PATH_1


class RoiPathSeparationTest(unittest.TestCase):
    def test_timestamped_captures_are_stored_outside_reference_directory(self):
        self.assertNotEqual(Path(ROI_CAPTURE_DIR), Path(ROI_PATH_1).parent)

    def test_timestamped_capture_name_is_not_loaded_as_reference(self):
        archive_name = "captured_roi_img_20260809_120000.png"
        self.assertFalse(
            any(fnmatch.fnmatch(archive_name, pattern) for pattern in ROI_GLOB_PATTERNS)
        )


if __name__ == "__main__":
    unittest.main()
