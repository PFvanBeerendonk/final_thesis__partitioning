from __future__ import annotations
from typing import TYPE_CHECKING
import os, math
from igl import ambient_occlusion, per_vertex_normals
import numpy as np
from datetime import datetime

from trimesh.base import Trimesh
from trimesh.creation import icosphere
from trimesh.transformations import angle_between_vectors

from config import (
    PLANE_SPACER, OUTPUT_FOLDER, SEAM_OCCLUSION_RAY_COUNT,
    PART_WEIGHT, UTIL_WEIGHT, SEAM_WEIGHT, PRINT_VOLUME, 
    SUF_DIFF_DISTANCE, SUF_DIFF_ANGLE,
)
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
    We will, for now, use vertices of an twice-subdivided icosphere
'''
def get_uniform_normals():
    return icosphere(subdivisions=2, radius=1).vertices

def _sufficiently_different_bsp(bsp1: BSP, bsp2: BSP):
    if bsp1.latest_origin is None or bsp2.latest_origin is None:
        return True
    if bsp1.latest_normal is None or bsp2.latest_normal is None:
        return True

    o = bsp2.latest_origin  # other plane origin
    delta = o - bsp1.latest_origin  # vector from this plane's origin to the others'
    dist = abs(bsp1.latest_normal @ delta)  # distance along this plane's normal to other plane
    # angle between this plane's normal vector and the other plane's normal vector
    angle = angle_between_vectors(bsp1.latest_normal, bsp2.latest_normal)
    # also consider angle between the vectors in the opposite direction
    angle = min(np.pi - angle, angle)
    # check if either the distance or the angle are above their respective thresholds
    return dist > DISTANCE_THRESHOLD or angle > ANGLE_THRESHOLD

def sufficiently_different(bsp: BSP, bsp_set: list[BSP]) -> bool:
    # TODO: make working
    return all(_sufficiently_different_bsp(bsp, b) for b in bsp_set)

'''
    return origins for planes with `normal` ranging over `part`
    Returned normals should cut `part` into at least 2 parts
'''
def sample_origins(part: Part, normal) -> list[(int, int, int)]:
    projection = part.mesh.vertices @ normal

    return [d * normal for d in np.arange(projection.min(), projection.max(), PLANE_SPACER)][1:]



# Export helpers
def export_part(part: Part):
    current_location = os.path.dirname(__file__)
    part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/intermediate.stl')

def export_bsp(bsp: BSP):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    for i, part in enumerate(bsp.parts):
        part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/out{i}--{now.strftime("%m-%d-%Y--%H-%M")}.stl')
