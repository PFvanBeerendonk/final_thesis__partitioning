from typing import Self
import numpy as np
import math

from config import PRINT_VOLUME, PART_WEIGHT, UTIL_WEIGHT
from helpers import export_part

from trimesh import Trimesh
from trimesh.bounds import oriented_bounds
from trimesh.intersections import slice_mesh_plane

# Maintain knowledge about parts
class Part:
    def __init__(self, mesh: Trimesh):
        self.mesh = mesh
        
        _, self.extents = oriented_bounds(self.mesh)
        self._update_fit()

    '''
        Returns all parts after cutting self given a cut plane.
    '''
    def cut(self, plane_normal, plane_origin) -> list[Self]:
        # change -0.0 into 0.0
        plane_normal[plane_normal==0.] = 0.
        plane_origin[plane_origin==0.] = 0.

        dot = np.dot(plane_normal, (self.mesh.vertices - plane_origin).T)[self.mesh.faces]

        s = self.mesh.slice_plane(plane_normal=plane_normal,
            plane_origin=plane_origin, cap=True,
            cached_dots=dot)
        dot *= -1
        sa = self.mesh.slice_plane(plane_normal=-plane_normal,
            plane_origin=plane_origin, cap=True,
            cached_dots=dot)

        # TODO: handle case where output mesh (s, sa) is one mesh with multiple parts
        #       eventually return a part for each seperate part in the output
        return (Part(s), Part(sa))


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

    def __init__(self, parts: list[Part], theta_zero = 0):
        if len(parts) == 0:
            raise Exception('Must have at least 1 part')
        self.parts = parts

        if theta_zero == 0:
            self.theta_zero = parts[0].est_part_required()
        else:
            self.theta_zero = theta_zero

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

        p1, p2 = part.cut(plane_normal, plane_origin)

        # TODO: maintain score
        return BSP(parts=parts + [p1, p2])

    
    ### OBJECTIVE FUNCTIONS ###
    def score(self):
        return (
            PART_WEIGHT * self._objective_part() + 
            UTIL_WEIGHT * self._objective_util()
        )
    
    def _objective_part(self):
        # 1/Theta * \sum p /in T O(p)
        return 1/self.theta_zero * sum(p.est_part_required() for p in self.parts)

    def _objective_util(self):
        print_volume = PRINT_VOLUME[0]*PRINT_VOLUME[1]*PRINT_VOLUME[2]
        def _util(part):
            return 1 - part.volume() / (part.est_part_required()*print_volume)

        return max(_util(p) for p in self.parts)
