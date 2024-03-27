from unittest.mock import patch
from unittest import TestCase

from trimesh.creation import cylinder

from beam_search.bsp import Part


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestEstPartRequired(TestCase):
    def test_part_fits(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)

        assert p1.est_part_required() == 1

    def test_part_smoll(self):
        c1 = cylinder(radius=2, height=2, sections=100)
        p1 = Part(c1)

        assert p1.est_part_required() == 1
    
    def test_part_large(self):
        c1 = cylinder(radius=5, height=10, sections=100)
        p1 = Part(c1)

        assert p1.est_part_required() == 4

