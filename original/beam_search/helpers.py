from bsp import BSP

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
    return [bsp for bsp in bsp_set if bsp.all_fit()]
