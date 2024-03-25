from __future__ import annotations
from typing import TYPE_CHECKING
import os
import numpy as np

from trimesh.creation import icosphere

from config import PLANE_SPACER, OUTPUT_FOLDER
if TYPE_CHECKING:
    from bsp import BSP, Part


'''
    Returns best scoring bsp over a set of BSPs
'''
def highest_ranked(bsp_set: list[BSP]) -> BSP:
    if len(bsp_set) == 0:
        raise Exception('bsp_set cannot be empty')
    return min(bsp_set, key=lambda x: x.score())

'''
    Check if all bsps in bsp_set fit in Printing Volume
    
    Assumes that fits_in_volume is correct for each part
'''
def all_at_goal(bsp_set: list[BSP]) -> bool:
    return all(bsp.all_fit() for bsp in bsp_set)


'''
    Returns all bsps in bsp_set that have a part that does not fit in Printing Volume
    
    Assumes that fits_in_volume is correct for each part
'''
def not_at_goal_set(bsp_set: list[BSP]) -> list[BSP]:
    return [bsp for bsp in bsp_set if not bsp.all_fit()]

'''
    Returns a set of "uniformly spaced" normals.
    
    Original paper used: vertices of thrice-subdivided regular octahedron.
    We will, for now, used vertices of an thrice-subdivided icosphere
'''
def get_uniform_normals():
    return icosphere(subdivisions=2, radius=1).vertices

def sufficiently_different(bsp: BSP, bsp_set: list[BSP]) -> bool:
    # TODO: make working
    return True

'''
    return origins for planes with `normal` ranging over `part`
'''
def sample_origins(part: Part, normal) -> list[(int, int, int)]:
    projection = part.mesh.vertices @ normal

    return [d * normal for d in np.arange(projection.min(), projection.max(), PLANE_SPACER)][1:]



# Export helpers
def export_part(part: Part):
    current_location = os.path.dirname(__file__)
    part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/intermediate.stl')

def export_bsp(bsp: BSP):
    current_location = os.path.dirname(__file__)
    for i, part in enumerate(bsp.parts):
        part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/out{i}.stl')
