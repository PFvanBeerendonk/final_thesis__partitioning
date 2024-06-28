from unittest import skip
from unittest.mock import patch
from unittest import TestCase


from main import main
class RunnerClass(TestCase):
    def run_all(self):
        pass

# PART_WEIGHT = 1
# UTIL_WEIGHT = 0.1
# SEAM_WEIGHT = 0
@patch('bsp.UTIL_WEIGHT', 0.1)
@patch('helpers.UTIL_WEIGHT', 0.1)
class RunnerTestUtil01(RunnerClass):
    def run_all(self):
        self.test_seam0()
        self.test_seam0001()
        self.test_seam001()
        self.test_seam01()

    @patch('bsp.SEAM_WEIGHT', 0)
    @patch('helpers.SEAM_WEIGHT', 0)
    def test_seam0():
        main()

    @patch('bsp.SEAM_WEIGHT', 0.001)
    @patch('helpers.SEAM_WEIGHT', 0.001)
    def test_seam0001():
        main()

    @patch('bsp.SEAM_WEIGHT', 0.01)
    @patch('helpers.SEAM_WEIGHT', 0.01)
    def test_seam001():
        main()

    @patch('bsp.SEAM_WEIGHT', 0.1)
    @patch('helpers.SEAM_WEIGHT', 0.1)
    def test_seam01():
        main()



def get_methods(Foo):
    return [func for func in dir(Foo) if callable(getattr(Foo, func))]

if __name__ == '__main__':
    RunnerTestUtil01.run_all()
