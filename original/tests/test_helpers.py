from beam_search.helpers import highest_ranked, all_at_goal
from beam_search.bsp import BSP, Part
from beam_search.config import PRINT_VOLUME

from unittest.mock import patch
from unittest import TestCase

from trimesh.creation import cylinder

class TestHighestRanked(TestCase):
    def test_highest_ranked(self):
        pass


@patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestAllAtGoal(TestCase):
    def test_all_at_goal_empty(self):
        bsp_set = [
            BSP([])
        ]
        assert all_at_goal(bsp_set)

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
    def test_not_at_goal_set(self):
        return
