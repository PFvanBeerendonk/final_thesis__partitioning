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
        for i in range(3):
            print(extents[i], PRINT_VOLUME[i])
        
        self.fits_in_volume = all(extents[i] <= PRINT_VOLUME[i] for i in range(3))


class BSP:
    """
        We only need to maintain track of the parts, which are all meshes
    """

    def __init__(self, parts: list[Part]):
        self.parts = parts

    def score(self):
        return 0

    def all_fit(self):
        return all(part.fits_in_volume for part in self.parts)
