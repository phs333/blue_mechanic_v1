import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "host"))

import run_host_tests  # noqa: E402


class MotionProfileHostTests(unittest.TestCase):
    """Roda os testes C do planejador de movimento (main/motion_profile.c) no PC."""

    def test_motion_profile_c(self):
        ok, output = run_host_tests.build_and_run()
        if ok is None:
            self.skipTest(output)
        self.assertTrue(ok, output)


if __name__ == "__main__":
    unittest.main()
