from unittest.mock import patch
from unittest import TestCase
import pytest

from trimesh.creation import cylinder

from beam_search.bsp import BSP, Part


class TestInitBSP(TestCase):
    def test_no_parts(self):
        with pytest.raises(Exception) as e:
            BSP([])


class TestGetLargestPart(TestCase):
    def test_single_part(self):
        mesh = cylinder(radius=2, height=10, sections=100)
        part = Part(mesh)
        bsp = BSP([
                part
            ])
        
        assert bsp.get_largest_part() == part
        
    def test_more_parts(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=2.1, height=10, sections=100)
        p2 = Part(c2)
        c3 = cylinder(radius=2.1, height=9.99, sections=100)
        p3 = Part(c3)
        
        bsp = BSP([ p1, p2, p3 ])
        
        assert bsp.get_largest_part() == p2


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestAllFit(TestCase):
    def test_more_parts_not_fit(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=2.1, height=10, sections=100)
        p2 = Part(c2)
        c3 = cylinder(radius=5, height=9.99, sections=100)
        p3 = Part(c3)
        
        bsp = BSP([ p1, p2, p3 ])
        
        assert not bsp.all_fit()
    
    def test_more_parts_fit(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=2.1, height=10, sections=100)
        p2 = Part(c2)
        c3 = cylinder(radius=2.1, height=9.99, sections=100)
        p3 = Part(c3)
        
        bsp = BSP([ p1, p2, p3 ])
        
        assert bsp.all_fit()


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestObjectivePart(TestCase):
    def test_one_part(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        
        assert p1.est_part_required() == 1
        bsp = BSP([ p1 ])
        # 1 / 1 * sum(1)
        assert bsp._objective_part() == 1

    def test_mul_parts(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=2.1, height=10, sections=100)
        p2 = Part(c2)
        c3 = cylinder(radius=5, height=9.99, sections=100)
        p3 = Part(c3)
        assert p1.est_part_required() == 1
        assert p2.est_part_required() == 1
        assert p3.est_part_required() == 4
        
        bsp = BSP([ p1, p2, p3 ])
        # 1/1 * sum(1+1+4)
        assert bsp._objective_part() == 6

@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestObjectiveUtil(TestCase):
    def test_one_part(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        
        assert p1.est_part_required() == 1
        bsp = BSP([ p1 ])
        # max([1 - p1.volume() / (1 * 9*9*10)])
        assert bsp._objective_util() == 1 - p1.volume() / (1 * 9*9*10)

    def test_mul_parts(self):
        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=2.1, height=10, sections=100)
        p2 = Part(c2)
        c3 = cylinder(radius=5, height=9.99, sections=100)
        p3 = Part(c3)
        assert p1.est_part_required() == 1
        assert p2.est_part_required() == 1
        assert p3.est_part_required() == 4
        
        bsp = BSP([ p1, p2, p3 ])
        # max([
        #   1 - p1.volume() / (1 * 9*9*10),
        #   1 - p2.volume() / (1 * 9*9*10),
        #   1 - p3.volume() / (4 * 9*9*10)
        # ])
        assert bsp._objective_util() == 1 - p1.volume() / (1 * 9*9*10)
