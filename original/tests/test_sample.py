

from unittest.mock import patch
from unittest import TestCase
import pytest, os

import trimesh
from trimesh.caching import tracked_array

from beam_search.bsp import BSP, Part


# @patch('beam_search.bsp.PRINT_VOLUME', (9, 9, 10))
class TestAllFit(TestCase):
    def setUp(self):
        current_location = os.path.dirname(__file__)
        mesh_path = f'{current_location}\samples\sample__tue_origin.stl'

        mesh = trimesh.load_mesh(mesh_path)
        self.part = Part(mesh)

    def test_weird_origin(self):
        origin = tracked_array([ 0., 0., 92.36976624])
        normal = tracked_array([0, 0, 1])

        result = self.part.cut(normal, origin)
        assert len(result) >= 2
        
