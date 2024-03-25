from typing import Self
import numpy as np

from config import PRINT_VOLUME
from helpers import export_part

from trimesh import Trimesh
from trimesh.bounds import oriented_bounds
from trimesh.intersections import slice_mesh_plane

# Maintain knowledge about parts
class Part:
    def __init__(self, mesh: Trimesh):
        self.mesh = mesh

        self._update_fit()

    '''
        Returns all parts after cutting self given a cut plane.
    '''
    def cut(self, plane_normal, plane_origin) -> list[Self]:
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
        _, extents = oriented_bounds(self.mesh)
        self.fits_in_volume = all(extents[i] <= PRINT_VOLUME[i] for i in range(3))

    def volume(self):
        return self.mesh.volume


class BSP:
    """
        We only need to maintain track of the parts, which are all meshes
    """

    def __init__(self, parts: list[Part]):
        if len(parts) == 0:
            raise Exception('Must have at least 1 part')
        self.parts = parts

    def score(self):
        return 0

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
