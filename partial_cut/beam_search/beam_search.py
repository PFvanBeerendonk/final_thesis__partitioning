import sys

from config import BEAM_WIDTH
from trimesh import Trimesh
from bsp import BSP, Part
from helpers import (
    all_at_goal, not_at_goal_set, highest_ranked, 
    get_uniform_normals, sufficiently_different,
    sample_origins,
)

uniform_normals = get_uniform_normals()

def beam_search(object_mesh: Trimesh):
    # Initialize with a bsp containing just the object
    current_bsp_set = [BSP([Part(object_mesh)])]

    number_of_cuts = 0
    # As long as not all parts, of each bsp fit...
    while not all_at_goal(current_bsp_set):
        print(f'\n\nEvaluating cut {number_of_cuts+1}')
        new_bsp_set = []
        bsp_todo_set = not_at_goal_set(current_bsp_set)
        for i, bsp_t in enumerate(bsp_todo_set):
            current_bsp_set.remove(bsp_t)
            largest_part = bsp_t.get_largest_part()
            new_bsp_set += eval_cuts(bsp_t, largest_part, i+1, len(bsp_todo_set))

            
        while len(current_bsp_set) < BEAM_WIDTH:
            best = highest_ranked(new_bsp_set)
            new_bsp_set.remove(best)
            current_bsp_set.append(best)
        number_of_cuts += 1

    print('\n')
    return highest_ranked(current_bsp_set)

'''
    Returns set of BSPs which are copies of `bsp_t` with a cut added to cut `part`

    Requires: `part` is in `bsp_t.parts`
'''
def eval_cuts(bsp_t: BSP, part: Part, part_id: int, part_todo: int) -> list[BSP]:
    candidate_cuts = []
    # collect normals
    normals = uniform_normals
    _report_eval_cut(0, len(normals), part_id, part_todo)
    for i, n in enumerate(normals):
        if i % 10 == 0:
            _report_eval_cut(i, len(normals), part_id, part_todo)
        for p in sample_origins(part.mesh, n):
            new_candidate_cut = bsp_t.cut_part(part, n, p)
            if new_candidate_cut:
                candidate_cuts.append(new_candidate_cut)


    result_cuts = []
    candidate_cuts.sort(key=lambda x: x.score())
    for c in candidate_cuts:
        if sufficiently_different(c, result_cuts):
            result_cuts.append(c)

    return result_cuts


'''
    Prints a progress bar to show how close `progress` is to `target`
    i.e. how many normals have been evaluated
'''
def _report_eval_cut(progress, target, part_i, part_todo):
    sys.stdout.write('\r')
    # the exact output you're looking for:
    progress_percentage = int(progress / target * 100)
    sys.stdout.write(f"Evaluating Normals for bsp {part_i}/{part_todo} [%-100s] %d%%" % ('='*progress_percentage, progress_percentage))
    sys.stdout.flush()
