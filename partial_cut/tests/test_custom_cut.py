from unittest import TestCase, skip
import os
import numpy as np

import trimesh
from trimesh.caching import tracked_array

from beam_search.bsp import Part
from beam_search.twin_cut import twin_cut, _find_connected, replace_duplicate_vertices
from beam_search.helpers import export_part, export_mesh_list

class TestCutAndHelpers(TestCase):
    def setUp(self):
        current_location = os.path.dirname(__file__)
        mesh_path = f'{current_location}\\samples\\sample__u.stl'

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
        
        components = list(_find_connected(self.part.mesh, face_indeces))
        assert len(components) == 2

        for id in face_indeces:
            assert id in components[0] or id in components[1]

    def test_replace_duplicate_vertices(self):
        '''   p-------q
              |      /|
              |     / |
              |    /  |
            --a---b---c--- CUTPLANE
              |  /    |
              | /     |
              x-------y

            Let us consider Quads and Tris below the cutplane

            QUAD x, y, b, c   inserted b, c
            TRI x, a, b       inserted a, b
            with offset=10
            id x=0, y=1, a=10, b=11, c==12

            replace_duplicate_vertices Should merge b
        '''

        offset = 10
        new_tri_faces = np.array([[0, 10, 11]])
        new_tri_vertices = [[0.1, 0.1, 0.4], [1.2, 2.2020, 3.4], [0, 10, 20], [0.1, 0.1, 0.1]]
        new_quad_vertices = [[0.1, 0.1, 0.1], [1.2, 2.2020, 3.4], [10, 20, 30], [1,2,3]]
                
        # prep
        new_tri_vertices = tracked_array([tracked_array(v) for v in new_tri_vertices])
        new_quad_vertices = tracked_array([tracked_array(v) for v in new_quad_vertices])

        result = replace_duplicate_vertices(offset, new_tri_faces, new_quad_vertices, new_tri_vertices)

        assert len(result) == 1
        # original index 10 has no "equivalent vertex" in QUAD
        # original index 11 has "equivalent vertex" in QUAD at j=1
        #  ==> update to j + offset - len(QUAD) = 1 + 10 - 4 = 7
        assert (result[0] == [0, 10, 7]).all()


class TestHorizontalCutSamples(TestCase):
    def _excute_horizontal_cut_improved(self, sample_name, origin, normal, expected_count, export=False):
        '''
            cut samples/{sample_name} at origin, normal

            number of results should equal `expected_count`
            each result should have length 2 and each part should be watertight
        '''
        current_location = os.path.dirname(__file__)
        mesh = trimesh.load_mesh(f'{current_location}\\samples\\{sample_name}.stl')

        # Mesh must be watertight to guarentee a resulting mesh that is watertight
        assert mesh.is_watertight

        result_parts = twin_cut(mesh, normal, origin)
        
        assert len(result_parts) == expected_count
        for p_list in result_parts:
            # each option should contain EXACTLY 2 parts
            assert len(p_list) == 2
            if export: export_mesh_list(p_list)

            # and each part should NOT be degenerate
            for p in p_list:
                assert p.is_watertight
                assert p.volume > 0.0
                assert p.is_winding_consistent
        return result_parts

    def test_u(self):
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__u', origin, normal, 2
        )

    def test_bunny(self):
        origin = tracked_array([ 0, 0, 40])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'Bunny-LowPoly', origin, normal, 1
        )
        
    def test_spikes(self):
        origin = tracked_array([ 0, 0, 0])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__spikes', origin, normal, 1, True
        )

