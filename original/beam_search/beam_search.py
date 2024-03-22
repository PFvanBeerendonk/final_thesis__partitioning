from config import (
    INPUT_FILE_PATH,
    OUTPUT_FOLDER,
    BEAM_WIDTH,
)
from trimesh import Trimesh
from trimesh.intersections import slice_mesh_plane
from bsp import BSP, Part
from helpers import all_at_goal, not_at_goal_set, highest_ranked

def beam_search(object_mesh: Trimesh):
    # Initialize with a bsp containing just the object
    current_bsp_set = [BSP([Part(object_mesh)])]

    while not all_at_goal(current_bsp_set):
        new_bsp_set = []
        for bsp_t in not_at_goal_set(current_bsp_set):
            current_bsp_set.remove(bsp_t)
            largest_part = get_largest_part(bsp_t)
            new_bsp_set += eval_cuts(bsp_t, largest_part)
        while len(current_bsp_set) < BEAM_WIDTH:
            current_bsp_set.append(highest_ranked(new_bsp_set))
    
    return highest_ranked(current_bsp_set)

def get_largest_part(bsp_t):
    return 0

def eval_cuts(bsp_t, part):
    return [bsp_t]
