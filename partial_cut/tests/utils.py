from unittest import TestCase
import os

import trimesh

from beam_search.twin_cut import twin_cut
from beam_search.helpers import export_mesh_list

class BaseModelTestCase(TestCase):
    mesh : trimesh.base.Trimesh = None

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
                export_mesh_list(p_list, val=i)

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

