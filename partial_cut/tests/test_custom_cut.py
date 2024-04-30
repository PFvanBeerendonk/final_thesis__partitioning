from unittest import TestCase
import pytest, os

import trimesh
from trimesh.caching import tracked_array
from trimesh.creation import cylinder

from beam_search.bsp import BSP, Part


from beam_search.helpers import export_part

class TestCut(TestCase):
    def setUp(self):
        current_location = os.path.dirname(__file__)
        mesh_path = f'{current_location}\samples\sample__u.stl'

        mesh = trimesh.load_mesh(mesh_path)
        self.part = Part(mesh)

    def test_horizontal_cut_original(self):
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])

        export_part(self.part)

        result_parts = self.part.cut(normal, origin)

        for p in result_parts:
            assert p.mesh.is_watertight

        assert len(result_parts) == 3
        
    def test_horizontal_cut_improved(self):
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])

        export_part(self.part)

        result_parts = self.part.twin_cut(normal, origin)

        for p in result_parts:
            assert p.mesh.is_watertight

        assert len(result_parts) == 3
