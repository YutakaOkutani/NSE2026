import importlib.util
import unittest
from pathlib import Path


SPEC_DIR = Path(__file__).resolve().parent


def load_tests(_loader, _standard_tests, _pattern):
    suite = unittest.TestSuite()
    for path in sorted(SPEC_DIR.glob("*.py")):
        if path.name == Path(__file__).name:
            continue
        module_name = f"runs_spec_{path.stem}"
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    return suite


if __name__ == "__main__":
    unittest.main()
