from __future__ import annotations
from typing import TYPE_CHECKING
import os
from igl import ambient_occlusion, per_vertex_normals
import numpy as np
from datetime import datetime

from trimesh.base import Trimesh
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
    We will, for now, use vertices of an twice-subdivided icosphere
'''
def get_uniform_normals():
    return icosphere(subdivisions=2, radius=1).vertices

def sufficiently_different(bsp: BSP, bsp_set: list[BSP]) -> bool:
    # TODO: make working
    # just store most recent normal + origin
    return True

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

def calculate_eps_objective_seam(vertices, faces, original_length):
    """
    Given a filter for the vertices that are on the plane
    Determine the penalty for each vertex that is on the plane, based on ambient occlusion
    The more occuled, lower ao value, the better!
    """
    # used: https://libigl.github.io/libigl-python-bindings/tut-chapter5/
    # must calculate normals over ALL vertices, otherwise we run into trouble
    normals = per_vertex_normals(vertices, faces)

    # take all vertices at and beyond index original_length
    v_sample = vertices[original_length:]
    n_sample = normals[original_length:]
    # calculate ambient occlusion
    # as far as I understand, calculates ao for v_sample with normals n_sample 
    # the final number is a scalar showing 0 if very occluded, and 1 if not at all occluded
    # it is also used to render ambient lighting
    ao = ambient_occlusion(vertices, faces, v_sample, n_sample, 20)

    # eps(C) = \sum_{onedge} p where p is ambient occlusion
    return sum(ao)

# Export helpers
def export_part(part: Part, name='intermediate', val=''):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/{name}{val}--{now.strftime("%m-%d-%Y--%H-%M")}.stl')

def export_bsp(bsp: BSP, name='out'):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    for i, part in enumerate(bsp.parts):
        part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/{name}{i}--{now.strftime("%m-%d-%Y--%H-%M")}.stl')

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
