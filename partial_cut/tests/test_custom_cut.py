from unittest import skip
import numpy as np
from utils import BaseModelTestCase

from trimesh.caching import tracked_array

from beam_search.bsp import Part
from beam_search.twin_cut import _find_connected, replace_duplicate_vertices_tri
from beam_search.helpers import flatten


class TestHelperConnected(BaseModelTestCase):
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


class TestFailingTwinCuts(BaseModelTestCase):
    '''
    TODO: Main issues with twin cut are:
        - if there is a shape with a dimple in it 
          NOTE: imagine red blood cell, cut such that it becomes a donut and a solidcircle
        - For unknown reason, the donut cut case fails. 
          NOTE: a lot of vertices are ONPLANE, moreover each triangle to be cut has 1 vertex on-plane
    '''

    # TODO: this variant will be fixed, currently returns fails in twin_cut where `if len(meshes) == 2:`.
    @skip
    def test_spikes_inner_mesh(self):
        origin = tracked_array([ 0, 0, 0])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__spikes', origin, normal, 1
        )

    # TODO: raises empty_cut exception... why?
    @skip 
    def test_donut(self):
        origin = tracked_array([ 0, 0, 0])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'donut', origin, normal, 1
        )

