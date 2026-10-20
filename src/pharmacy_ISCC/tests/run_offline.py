#!/usr/bin/env python3
"""Run dependency-isolated developer checks with Python 3, no ROS required."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
suite = unittest.TestSuite()
for folder in ["unit", "integration"]:
    suite.addTests(unittest.defaultTestLoader.discover(str(ROOT / "tests" / folder), top_level_dir=str(ROOT / "tests")))
result = unittest.TextTestRunner(verbosity=2).run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
