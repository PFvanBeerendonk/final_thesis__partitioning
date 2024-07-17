from unittest import skip
import numpy as np
from utils import BaseModelTestCase

from trimesh.caching import tracked_array

from beam_search.twin_cut import replace_duplicate_vertices_tri, merge_duplicate_vertices



class TestDuplicateRemoving(BaseModelTestCase):
    '''
    Test whether duplicates are removed on the cut. 

    Goal: remove `replace_duplicate_vertices_tri` etc with a quicker methods without changing behavior

    '''

    def test_horizontal_cut_spikes(self):
        origin = tracked_array([ 0, 0, 0.1])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__spikes', origin, normal, 1, True, True
        )

    @skip
    def test_horizontal_cut_tue(self):
        origin = tracked_array([ 0, 0, 70])
        normal = tracked_array([0, 0, 1])

        res = self._excute_horizontal_cut_improved(
            'sample__tue_origin', origin, normal, 2, True, True
        )
        
        
 
    # Function not used anymore; replaced by "merge_vertices"
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


class TestMergeDuplicateVertices(BaseModelTestCase):
    def test_merge_duplicate_vertices(self):
        vertices_old = [
            [1,2,3], #A
            [2,3,4],
            [4,5,6],
            [0,0,0],
        ]
        vertices_top = [
            [1,2,3], #A
            [8,4,5],
            [5,7,8], #B
            [11,1,1],
            [52,71,81],
        ]
        vertices_bottom = [
            [1,2,3], #A
            [5,7,8], #B
            [9,0,3],
            [100,10,1] #unreferenced by face
        ]
        vertices = np.array(vertices_old + vertices_top + vertices_bottom)

        faces_old = [
            [0,1,2],
            [1,2,3],
        ]
        faces_top = [
            [0,4,5],
            [0,5,6],
        ]
        faces_bottom = [
            [0,9,10],
            [4,9,11],
        ]
        faces = np.array(faces_old + faces_top + faces_bottom)
        
        res_vertices, res_faces = merge_duplicate_vertices(vertices, faces, len(vertices_old), len(vertices_old) + len(vertices_top))

        # should have replaces ONLY unreferenced (id = 12)
        # and should merge #A; but should not merge #B
        assert len(res_vertices) == len(vertices) - 2

        expected_res_vertices = np.array(vertices_old + vertices_top + [[5,7,8], [9,0,3]])
        assert np.array_equal(res_vertices, expected_res_vertices)

        # should replace all instances of 9 and 12 in faces
        #  9 ==> 0
        #  12 is removed, not referenced
        # in addition, each index is updated to reflect
        expected_res_faces = np.array(faces_old + faces_top + [[0,0,9], [4,0,10]])

        print(faces)
        print(res_faces)
        

        assert np.array_equal(res_faces, expected_res_faces)

    def test_merge_duplicate_vertices_test2(self):
        vertices_old = [
            [1,2,3], #A
            [2,3,4],
            [4,5,6],
            [0,0,0],
        ]
        vertices_top = [
            [1,2,3], #A
            [8,4,5],
            [5,7,8], #B
            [11,1,1],
            [11,1,1],
            [11,1,1],
            [52,71,81],
        ]
        vertices_bottom = [
            [1,2,3], #A
            [5,7,8], #B
            [9,0,3],
            [100,10,1] #unreferenced by face
        ]
        vertices = np.array(vertices_old + vertices_top + vertices_bottom)

        faces_old = [
            [0,1,2],
            [1,2,3],
        ]
        faces_top = [
            [0,4,5],
            [0,5,6],
        ]
        faces_bottom = [
            [0,11,12],
            [4,11,13],
        ]
        faces = np.array(faces_old + faces_top + faces_bottom)
        
        res_vertices, res_faces = merge_duplicate_vertices(vertices, faces, len(vertices_old), len(vertices_old) + len(vertices_top))

        # should have replaces ONLY unreferenced (id = 14)
        # and should merge #A; but should not merge #B
        assert len(res_vertices) == len(vertices) - 2

        expected_res_vertices = np.array(vertices_old + vertices_top + [[5,7,8], [9,0,3]])
        assert np.array_equal(res_vertices, expected_res_vertices)

        # should replace all instances of 9 and 12 in faces
        #  9 ==> 0
        #  12 is removed, not referenced
        # in addition, each index is updated to reflect
        expected_res_faces = np.array(faces_old + faces_top + [[0,0,11], [4,0,12]])

        print(faces)
        print(res_faces)
        

        assert np.array_equal(res_faces, expected_res_faces)
