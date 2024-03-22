from typing import Self
from config import (
    PRINT_VOLUME_WIDTH,
    PRINT_VOLUME_HEIGHT,
    PRINT_VOLUME_DEPTH
)


# Maintain knowledge about parts
class Part:
    def __init__(self, mesh):
        self.mesh = mesh
        self.fits_in_volume = False

    '''
        Returns all parts after cutting self given a cut plane.
    '''

    def cut(self, plane) -> list[Self]:
        return [self]


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
