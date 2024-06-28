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

DIAGONAL_PRINTING_VOLUME =  math.sqrt(sum(e**2 for e in PRINT_VOLUME))

def highest_ranked(bsp_set: list[BSP]) -> BSP:
    '''
    Returns best scoring bsp over a set of BSPs
    '''
    if len(bsp_set) == 0:
        raise Exception('bsp_set cannot be empty')
    return min(bsp_set, key=lambda x: x.score())

def all_at_goal(bsp_set: list[BSP]) -> bool:
    '''
    Check if all bsps in bsp_set fit in Printing Volume
    
    Assumes that fits_in_volume is correct for each part
    '''
    return all(bsp.all_fit() for bsp in bsp_set)

def not_at_goal_set(bsp_set: list[BSP]) -> list[BSP]:
    '''
    Returns all bsps in bsp_set that have a part that does not fit in Printing Volume
    
    Assumes that fits_in_volume is correct for each part
    '''
    return [bsp for bsp in bsp_set if not bsp.all_fit()]

def get_uniform_normals():
    '''
    Returns a set of "uniformly spaced" normals.
    
    Original paper used: vertices of thrice-subdivided regular octahedron.
    We will, for now, use vertices of an twice-subdivided icosphere
    '''
    return icosphere(subdivisions=2, radius=1).vertices

DISTANCE_THRESHOLD = DIAGONAL_PRINTING_VOLUME * SUF_DIFF_DISTANCE
ANGLE_THRESHOLD = np.pi * SUF_DIFF_ANGLE

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
    """
    If latest cut of bsp is too close to some bsp in bsp_set, return false

    NOTE: bsp.normals and bsp.origins can be empty
    """
    return all(_sufficiently_different_bsp(bsp, b) for b in bsp_set)

def sample_origins(mesh: Trimesh, normal) -> list[(int, int, int)]:
    """
    Returns origins.
    Each plane defined by normal and one such origin MUST intersect mesh

    Parameters
    ---------
    mesh : Trimesh
        Some mesh
    normal : (3,) float
        Normal vector

    Returns
    ----------
    origins : (n, 3) : float
        list of points such that the plane defined by `normal` and that point intersects the part
    """
    projection = mesh.vertices @ normal

    return [d * normal for d in np.arange(projection.min(), projection.max(), PLANE_SPACER)][1:]

def calculate_eps_objective_seam(vertices, faces, cap):
    """
    Given a filter for the vertices that are on the plane
    Determine the penalty for each vertex that is on the plane, based on ambient occlusion
    The more occuled, lower ao value, the better!
    
    Parameters
    ---------
    vertices : (3,) float
        List of all vertices of a mesh
    faces : (3,) float
        List of all faces for the same mesh
    cap : int
        Only calculate ambient occlusion for vertices in the slice [cap:]

    Returns
    ----------
    eps_seam : (n, 3) : float
        list of points such that the plane defined by `normal` and that point intersects the part
    """
    # used: https://libigl.github.io/libigl-python-bindings/tut-chapter5/
    # must calculate normals over ALL vertices, otherwise we run into trouble
    normals = per_vertex_normals(vertices, faces)

    # take all vertices at and beyond index original_length
    v_sample = vertices[cap:]
    n_sample = normals[cap:]
    # calculate ambient occlusion
    # as far as I understand, calculates ao for v_sample with normals n_sample 
    # the final number is a scalar showing 0 if very occluded, and 1 if not at all occluded
    # it is also used to render ambient lighting
    ao = ambient_occlusion(vertices, faces, v_sample, n_sample, SEAM_OCCLUSION_RAY_COUNT)

    # eps(C) = \sum_{onedge} p where p is ambient occlusion
    return sum(ao)

# Export helpers
def export_part(part: Part, name='intermediate', val=''):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/{name}{val}--{now.strftime("%m-%d-%Y--%H-%M")}.stl')

def export_bsp(bsp: BSP, name='out', timing=None):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    path_to_files = f'{current_location}/{OUTPUT_FOLDER}/{now.strftime("%m-%d-%Y--%H-%M")}'
    os.mkdir(path_to_files)
    for i, part in enumerate(bsp.parts):
        part.mesh.export(f'{path_to_files}/{name}{i}--.stl')
    
    file = open(f'{path_to_files}/stats.txt', 'w')
    file.write(f'''time: {timing} seconds

final scores:
Nr of Parts: {len(bsp.parts)}
Util:        {bsp._objective_util()}
Seam:        {bsp._objective_seam(bsp._get_sum_parts_est_req())}

Weights:
PART_WEIGHT = {PART_WEIGHT}
UTIL_WEIGHT = {UTIL_WEIGHT}
SEAM_WEIGHT = {SEAM_WEIGHT}
''')
    file.close()

def export_mesh_list(lst, name='lout', val=''):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    for i, mesh in enumerate(lst):
        mesh.export(f'{current_location}/{OUTPUT_FOLDER}/{name}{val}{i}--{now.strftime("%m-%d-%Y--%H-%M")}.stl')


# mathematics

def powerset_no_emptyset(s: list[any]) -> list[list[any]]:
    '''
        Given list s, return the powerset of s. But exclude the emptyset
    '''
    x = sum(1 for _ in s)
    masks = [1 << i for i in range(x)]
    for i in range(1, 1 << x):
        yield [ss for mask, ss in zip(masks, s) if i & mask]

def flatten(lst: list[list[any]], f=None) -> list[any]:
    """
        Flatten list of lists
    """
    # flattens list
    if callable(f):
        # apply f
        return [
            f(p)
            for sub_list in lst
            for p in sub_list
        ]
    else:
        # do not apply f
        return [
            p
            for sub_list in lst
            for p in sub_list
        ]
