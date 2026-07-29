#!/usr/bin/env python3
"""ros_pytest_runner -- pytest bridge for rostest.

Usage:
  1) Via rostest (.test launch file):
     <test test-name="imu_filter_pytest" pkg="imu_filter" type="ros_pytest_runner.py" />
  2) Standalone:
     python3 ros_pytest_runner.py --standalone

Returns: exit 0 = all pass, exit 1 = failure
"""

import argparse
import os
import subprocess
import sys
import unittest

PKG = "filter"


class PytestRunner(unittest.TestCase):
    """unittest.TestCase wrapper: runs pytest internally, converts to unittest assertions."""

    def test_all(self):
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        test_dir = os.path.join(pkg_root, "test")

        print(f"[ros_pytest_runner] test_dir = {test_dir}", flush=True)

        result = subprocess.run(
            [sys.executable, "-m", "pytest", test_dir, "-v", "--tb=short", "--color=yes"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=120, cwd=pkg_root,
        )

        if result.stdout:
            sys.stdout.write(result.stdout)
        if result.stderr:
            sys.stderr.write(result.stderr)

        self.assertEqual(result.returncode, 0, f"pytest exited with code {result.returncode}")


def run_standalone(test_dir=None):
    """Run pytest directly, no rostest dependency."""
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if test_dir is None:
        test_dir = os.path.join(pkg_root, "test")

    print(f"[ros_pytest_runner standalone] test_dir = {test_dir}", flush=True)

    result = subprocess.run(
        [sys.executable, "-m", "pytest", test_dir, "-v", "--tb=short", "--color=yes"],
        cwd=pkg_root,
    )
    return result.returncode


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test-dir", default=None, help="pytest test directory")
    parser.add_argument("--standalone", action="store_true", help="run without rostest")
    args, unknown = parser.parse_known_args()

    if args.standalone:
        sys.exit(run_standalone(args.test_dir))
    else:
        import rostest
        rostest.rosrun(PKG, "ros_pytest_runner", PytestRunner)
