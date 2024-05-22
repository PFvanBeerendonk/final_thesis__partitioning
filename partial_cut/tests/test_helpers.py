from unittest.mock import patch
from unittest import TestCase
import pytest

from trimesh.creation import cylinder

from beam_search.helpers import (
    highest_ranked, all_at_goal, not_at_goal_set, powerset_no_emptyset
)
from beam_search.bsp import BSP, Part


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestHighestRanked(TestCase):
    def test_highest_ranked(self):
        bsp_small = BSP([
            Part(cylinder(radius=2, height=9, sections=100))
        ])
        bsp_large = BSP([
            Part(cylinder(radius=2000, height=900, sections=100)),
            Part(cylinder(radius=2, height=9, sections=100)),
        ])

        assert bsp_large.score() > bsp_small.score()
        assert highest_ranked([bsp_small, bsp_large]) == bsp_small

    def test_empty(self):
        with pytest.raises(Exception) as e:
            highest_ranked([])


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestAllAtGoal(TestCase):
    def test_all_at_goal_fits(self):
        mesh = cylinder(radius=2, height=10, sections=100)
        bsp_set = [
            BSP([
                Part(mesh)
            ])
        ]
        assert all_at_goal(bsp_set)

    def test_all_at_goal_wont_fit(self):
        mesh = cylinder(radius=2, height=11, sections=100)
        bsp_set = [
            BSP([
                Part(mesh)
            ])
        ]
        assert not all_at_goal(bsp_set)


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestNotAtGoalSet(TestCase):
    def test_empty(self):
        mesh = cylinder(radius=2, height=10, sections=100)
        p0 = Part(mesh)
        b0 = BSP([p0])

        assert p0.fits_in_volume

        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=2.1, height=10, sections=100)
        p2 = Part(c2)
        c3 = cylinder(radius=2.1, height=9.99, sections=100)
        p3 = Part(c3)
        b1 = BSP([p1, p2, p3])

        assert p1.fits_in_volume
        assert p2.fits_in_volume
        assert p3.fits_in_volume
        
        bsp_set = [ b0, b1 ]
        assert not_at_goal_set(bsp_set) == []

    def test_partial(self):
        mesh = cylinder(radius=2, height=10, sections=100)
        p0 = Part(mesh)
        b0 = BSP([p0])

        assert p0.fits_in_volume

        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=5, height=7, sections=100)
        p2 = Part(c2)
        b1 = BSP([p1, p2])

        assert p1.fits_in_volume
        assert not p2.fits_in_volume
        
        bsp_set = [ b0, b1 ]
        assert not_at_goal_set(bsp_set) == [ b1 ]

    def test_fully(self):
        mesh = cylinder(radius=2, height=11, sections=100)
        p0 = Part(mesh)
        b0 = BSP([p0])

        assert not p0.fits_in_volume

        c1 = cylinder(radius=2, height=10, sections=100)
        p1 = Part(c1)
        c2 = cylinder(radius=5, height=7, sections=100)
        p2 = Part(c2)
        b1 = BSP([p1, p2])

        assert p1.fits_in_volume
        assert not p2.fits_in_volume
        
        bsp_set = [ b0, b1 ]
        assert not_at_goal_set(bsp_set) == bsp_set


class TestPowersetNoEmptyset(TestCase):
    def test_list(self):
        out = powerset_no_emptyset([1,2])
        assert list(out) == [[1], [2], [1,2]]

    def test_list_of_sets(self):
        out = powerset_no_emptyset(
            [{1},{2,4}]
        )
        assert list(out) == [[{1}], [{2,4}], [{1},{2,4}]]
