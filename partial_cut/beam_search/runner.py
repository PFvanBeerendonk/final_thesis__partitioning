from unittest import skip
from unittest.mock import patch
from unittest import TestCase


from main import main

class RunnerClass(TestCase):
    def run_all(self):
        self.test_seam0()
        self.test_seam0001()
        self.test_seam001()
        self.test_seam01()

    @patch('bsp.SEAM_WEIGHT', 0)
    @patch('helpers.SEAM_WEIGHT', 0)
    def test_seam0(self):
        main()

    @patch('bsp.SEAM_WEIGHT', 0.001)
    @patch('helpers.SEAM_WEIGHT', 0.001)
    def test_seam0001(self):
        main()

    @patch('bsp.SEAM_WEIGHT', 0.01)
    @patch('helpers.SEAM_WEIGHT', 0.01)
    def test_seam001(self):
        main()

    @patch('bsp.SEAM_WEIGHT', 0.1)
    @patch('helpers.SEAM_WEIGHT', 0.1)
    def test_seam01(self):
        main()

@patch('bsp.UTIL_WEIGHT', 0)
@patch('helpers.UTIL_WEIGHT', 0)
class RunnerTestUtil0(RunnerClass):
    pass

@patch('bsp.UTIL_WEIGHT', 0.1)
@patch('helpers.UTIL_WEIGHT', 0.1)
class RunnerTestUtil01(RunnerClass):
    pass

@patch('bsp.UTIL_WEIGHT', 0.01)
@patch('helpers.UTIL_WEIGHT', 0.01)
class RunnerTestUtil001(RunnerClass):
    pass

@patch('bsp.UTIL_WEIGHT', 0.001)
@patch('helpers.UTIL_WEIGHT', 0.001)
class RunnerTestUtil0001(RunnerClass):
    pass


if __name__ == '__main__':
    runner_classes = [RunnerTestUtil0, RunnerTestUtil01, RunnerTestUtil001, RunnerTestUtil0001]
    for c in runner_classes:
        runner = c()
        runner.run_all()
