from beam_search.helpers import highest_ranked, all_at_goal
from beam_search.bsp import BSP, Part

from unittest.mock import patch
from unittest import TestCase


class TestHighestRanked(TestCase):
    def test_highest_ranked(self):
        pass


class TestAllAtGoal(TestCase):

    def test_all_at_goal_empty(self):
        bsp_set = [
            BSP([])
        ]
        assert all_at_goal(bsp_set)

    @patch('beam_search.config.PRINT_VOLUME_WIDTH', 10)
    @patch('beam_search.config.PRINT_VOLUME_HEIGHT', 10)
    @patch('beam_search.config.PRINT_VOLUME_DEPTH', 10)
    def test_all_at_goal_default(self):
        return
        mesh = 1 #TODO: add a good mesh
        bsp_set = [
            BSP([
                Part(mesh)
            ])
        ]
        assert not all_at_goal(bsp_set)


# class TestNotAtGoalSet(TestCase):
#     def test_not_at_goal_set(self):
#         return
