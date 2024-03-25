from typing import Self
from config import PRINT_VOLUME

from trimesh import Trimesh
from trimesh.bounds import oriented_bounds

# Maintain knowledge about parts
class Part:
    def __init__(self, mesh: Trimesh):
        self.mesh = mesh

        self._update_fit()

    '''
        Returns all parts after cutting self given a cut plane.
    '''
    def cut(self, plane) -> list[Self]:
        return [self]

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
