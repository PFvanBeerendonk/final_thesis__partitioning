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

        result_parts = self.part.cut(normal, origin)

        for p in result_parts:
            assert p.mesh.is_watertight

        assert len(result_parts) == 3
        
    def test_find_connected(self):
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])
        slice2d = self.part.mesh.section(
            plane_normal=normal,
            plane_origin=origin,
        )
        face_indeces = slice2d.metadata['face_index']

        assert len(face_indeces) == 16
        
        components = list(self.part._find_connected(face_indeces))
        assert len(components) == 2

        for id in face_indeces:
            assert id in components[0] or id in components[1]

    def test_horizontal_cut_improved(self):
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])


        result_parts = self.part.twin_cut(normal, origin)
        
        # 2 options when cutting a U above the bottom
        # assert len(result_parts) == 2
        for p_list in result_parts:
            for i, p in enumerate(p_list):
                print(p.volume)
                # export_part(p, i)
                    

            # each option should contain EXACTLY 2 parts
            assert len(p_list) == 2

            # TODO: implement cap
            # # and each part should NOT be degenerate
            # for p in p_list:
            #     assert p.mesh.is_watertight

