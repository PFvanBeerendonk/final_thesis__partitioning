from typing import Self
import numpy as np
import math

from helpers import flatten
from twin_cut import twin_cut
from config import (
    PRINT_VOLUME, PART_WEIGHT, UTIL_WEIGHT, SEAM_WEIGHT
)

from trimesh import Trimesh
from trimesh.bounds import oriented_bounds

from trimesh.graph import face_adjacency
import networkx as nx

# Maintain knowledge about parts
class Part:
    def __init__(self, mesh: Trimesh):
        self.mesh = mesh
        
        _, self.extents = oriented_bounds(self.mesh)
        self._update_fit()

    '''
        Returns all parts after cutting self given a cut plane.
    '''
    def cut(self, plane_normal, plane_origin, face_index=None, cap=True) -> list[Self]:
        if face_index:
            faces = [self.mesh.faces[id] for id in face_index]
        else:
            faces = self.mesh.faces
        dot = np.dot(plane_normal, (self.mesh.vertices - plane_origin).T)[faces]

        s = self.mesh.slice_plane(plane_normal=plane_normal,
            plane_origin=plane_origin, cap=cap,
             face_index=face_index)
        dot *= -1
        sa = self.mesh.slice_plane(plane_normal=-plane_normal,
            plane_origin=plane_origin, cap=cap,
            face_index=face_index)

        # temporary hack, should get fixed: https://github.com/mikedh/trimesh/issues/2203
        
        # split mesh into disjoint parts
        s_meshes = s.split()
        sa_meshes = sa.split()
        if len(s_meshes) == 0:
            s_meshes = [s]
        if len(sa_meshes) == 0:
            sa_meshes = [sa]

        return [Part(p) for p in s_meshes + sa_meshes]


    '''
        Given a set of face_ids, and set of faces
        Return C s.t.
        U F \in C F = ids AND
        ∩ F \in C F = []  AND
        ∀ F \in C: i, j \in F <==> faces[i] ∩ faces[j] != []

        In other words, the result is a subdivision of ids, and if 2 face_ids are in the same subset, they are connected
    '''
    def _find_connected(self, ids: list[int]) -> list[list[int]]:
        faces = [self.mesh.faces[i] for i in ids]

        adjacent_faces = face_adjacency(faces)
        graph = nx.Graph()
        graph.add_edges_from(adjacent_faces)

        components = nx.connected_components(graph)
        return [ [ids[c] for c in c_list] for c_list in components ] 


    
    def twin_cut(self, plane_normal, plane_origin) -> list[list[Self]]:
        '''
            Cut self.mesh into exactly 2 parts, based on a plane
            NOTE: can return empty list, in case of an error

            @returns list of pairs of parts resulting from cutting using the plane and 'sowing' some cuts back together
        '''
        
        res, eps_seam = twin_cut(self.mesh, plane_normal, plane_origin)

        # flatten
        return flatten(res, (lambda p: Part(p))), eps_seam


    """
        Update self.fits_in_volume based on mesh and PRINT_VOLUME params
    """
    def _update_fit(self):
        self.fits_in_volume = all(self.extents[i] <= PRINT_VOLUME[i] for i in range(3))

    def volume(self):
        return self.mesh.volume

    '''
    (upper bound on) the number of print-volumes required
        Checks how many volumes are needed to tile the OBB
    '''
    def est_part_required(self):
        mul = 1
        for i in range(3):
            mul *= math.ceil(self.extents[i] / PRINT_VOLUME[i])
        return mul

class BSP:
    """
        We only need to maintain track of the parts, which are all meshes
    """

    def __init__(self, parts: list[Part], one_over_theta_zero=0, seam_sum=0, latest_eps_seam=0, diagonal_zero=0):
        if len(parts) == 0:
            raise Exception('Must have at least 1 part')
        self.parts = parts

        if one_over_theta_zero == 0:
            self.one_over_theta_zero = 1 / parts[0].est_part_required()
        else:
            self.one_over_theta_zero = one_over_theta_zero

        if diagonal_zero == 0:
            # length of diagonal of OBB, is sqrt(x^2 + y^2 + z^2)
            self.diagonal_zero = math.sqrt(sum(e**2 for e in parts[0].extents))
        else:
            self.diagonal_zero = diagonal_zero

        ### maintain "on the fly" objectives ###
        # First sum in seam objective: \sum_{C \in T}eps(C)
        self.seam_sum = seam_sum
        self.latest_eps_seam = latest_eps_seam

    def all_fit(self):
        return all(part.fits_in_volume for part in self.parts)

    def get_largest_part(self) -> Part:
        if len(self.parts) == 0:
            raise Exception('Must have at least 1 part')
        return max(self.parts, key=lambda x: x.volume())

    '''
        cut `part` in `self` using plane, and return new bsp with `part` removed
        and a sub-parts added
    '''
    def cut_part(self, part: Part, plane_normal, plane_origin) -> Self:
        parts = [p for p in self.parts]
        parts.remove(part)

        new_parts, eps_seam = part.twin_cut(
            plane_normal=plane_normal, 
            plane_origin=plane_origin,
        )

        if len(new_parts) > 0:
            return BSP(
                parts=parts + new_parts, 
                one_over_theta_zero=self.one_over_theta_zero,
                seam_sum=self.seam_sum + eps_seam,
                latest_eps_seam=eps_seam,
            )
        else:
            return None

    
    ### OBJECTIVE FUNCTIONS ###
    def score(self):
        sum_parts_est_req = sum(p.est_part_required() for p in self.parts)
        return (
            PART_WEIGHT * self._objective_part(sum_parts_est_req) + 
            UTIL_WEIGHT * self._objective_util() +
            SEAM_WEIGHT * self._objective_seam(sum_parts_est_req)
        )

    def _objective_part(self, sum_parts_est_req):
        # 1/Theta * \sum p /in T O(p)
        return self.one_over_theta_zero * sum_parts_est_req

    def _objective_util(self):
        print_volume = PRINT_VOLUME[0] * PRINT_VOLUME[1] * PRINT_VOLUME[2]
        def _util(part):
            return 1 - part.volume() / (part.est_part_required()*print_volume)

        return max(_util(p) for p in self.parts)

    def _objective_seam(self, sum_parts_est_req):
        return self.one_over_theta_zero * (
            self.seam_sum / self.diagonal_zero + self.latest_eps_seam * (sum_parts_est_req - len(self.parts))
        ) 
