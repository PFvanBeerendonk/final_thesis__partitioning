from unittest import TestCase, skip
import os
import numpy as np

import trimesh
from trimesh.caching import tracked_array

from beam_search.bsp import Part
from beam_search.twin_cut import twin_cut, _find_connected, replace_duplicate_vertices_tri
from beam_search.helpers import flatten, export_part, export_mesh_list

class BaseModelTestCase(TestCase):
    def _load_model(self, sample_name):
        current_location = os.path.dirname(__file__)
        mesh_path = f'{current_location}\\samples\\{sample_name}.stl'

        self.mesh = trimesh.load_mesh(mesh_path)

        # Mesh must be watertight to guarentee a resulting mesh that is watertight
        assert self.mesh.is_watertight

    def _excute_horizontal_cut_improved(self, sample_name, origin, normal, expected_count, export=False):
        '''
            cut samples/{sample_name} at origin, normal

            number of results should equal `expected_count`
            each result should have length 2 and each part should be watertight
        '''
        self._load_model(sample_name)

        result_parts = twin_cut(self.mesh, normal, origin)
        
        # export before assert
        if export: 
            for i, p_list in enumerate(result_parts):
                export_mesh_list(p_list, i)

        assert len(result_parts) == expected_count
        for p_list in result_parts:
            # each option should contain EXACTLY 2 parts
            assert len(p_list) == 2

            # and each part should NOT be degenerate
            for p in p_list:
                assert p.is_watertight
                assert p.volume > 0.0
                assert p.is_winding_consistent
        return result_parts

class TestConnected(BaseModelTestCase):
    def _val_find_connected(self, face_indeces, expected_num_indices, expected_num_components):
        assert len(face_indeces) == expected_num_indices
        
        components = list(_find_connected(self.mesh, face_indeces))
        assert len(components) == expected_num_components

        # union is empty, intersection is face_indeces
        component_intersection = set.intersection(*map(set,components))
        component_union = flatten(components)
        assert len(component_intersection) == 0
        assert len(set(component_union)) == expected_num_indices

        return components

    def test_find_connected(self):
        self._load_model('sample__u')
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])
        slice2d = self.mesh.section(
            plane_normal=normal,
            plane_origin=origin,
        )
        face_indeces = slice2d.metadata['face_index']
        self._val_find_connected(face_indeces, 16, 2)

    def test_find_connected_donut(self):
        self._load_model('donut')
        origin = tracked_array([ 0, 0, 0])
        normal = tracked_array([0, 0, 1])
        slice2d = self.mesh.section(plane_normal=normal, plane_origin=origin)
        face_indeces = slice2d.metadata['face_index']
        
        self._val_find_connected(face_indeces, 96, 2)

class TestCutAndHelpers(BaseModelTestCase):
    def setUp(self):
        self._load_model('sample__u')
        self.part = Part(self.mesh)

    def test_horizontal_cut_original(self):
        origin = tracked_array([ 0, 0, 4])
        normal = tracked_array([0, 0, 1])

        result_parts = self.part.cut(normal, origin)

        for p in result_parts:
            assert p.mesh.is_watertight

        assert len(result_parts) == 3

    def test_replace_duplicate_vertices_tri(self):
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

            replace_duplicate_vertices_tri Should merge b
        '''

        offset = 10
        new_tri_faces = np.array([[0, 10, 11]])
        new_tri_vertices = [[0.1, 0.1, 0.4], [1.2, 2.2020, 3.4], [0, 10, 20], [0.1, 0.1, 0.1]]
        new_quad_vertices = [[0.1, 0.1, 0.1], [1.2, 2.2020, 3.4], [10, 20, 30], [1,2,3]]
                
        # prep
        new_tri_vertices = tracked_array([tracked_array(v) for v in new_tri_vertices])
        new_quad_vertices = tracked_array([tracked_array(v) for v in new_quad_vertices])

        result = replace_duplicate_vertices_tri(offset, new_tri_faces, new_quad_vertices, new_tri_vertices)

        assert len(result) == 1
        # original index 10 has no "equivalent vertex" in QUAD
        # original index 11 has "equivalent vertex" in QUAD at j=1
        #  ==> update to j + offset - len(QUAD) = 1 + 10 - 4 = 7
        assert (result[0] == [0, 10, 7]).all()


class TestHorizontalCutSamples(BaseModelTestCase):
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
        origin = tracked_array([ 0, 0, 0.1])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__spikes', origin, normal, 1
        )

    # this variant will be fixed, currently returns fails in twin_cut where `if len(meshes) == 2:`.
    @skip
    def test_spikes_inner_mesh(self):
        origin = tracked_array([ 0, 0, 0])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__spikes', origin, normal, 1
        )

class Adsa(BaseModelTestCase):
    @skip
    def test_donut(self):
        origin = tracked_array([ 0, 0, 0])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'donut', origin, normal, 1
        )


