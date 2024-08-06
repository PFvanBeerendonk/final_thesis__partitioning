from config import INPUT_FILE_PATH

import trimesh, os, time

from beam_search import beam_search
from helpers import export_bsp

def main():
    print(f'Loading File "{INPUT_FILE_PATH}"\n')
    current_location = os.path.dirname(__file__)
    mesh_path = f'{current_location}/{INPUT_FILE_PATH}'

    mesh = trimesh.load_mesh(mesh_path)

    if not mesh.is_watertight:
        raise Exception('Mesh not watertight')
    
    parts = mesh.split()
    if len(parts) != 1:
        raise Exception('Too many parts, mesh must consist of a single part')

    # get last part of path, and then cut of .stl (last 4 chars)
    model_name = INPUT_FILE_PATH.split('/')[-1][:-4]
    print(f'Starting beam search for model {model_name}, start time')
    start_time = time.time()

    bsp = beam_search(mesh)

    end_time = time.time()
    timing = end_time - start_time
    print('Finished beam search, end time')
    print(f'\n--- Time elapsed = {timing} seconds ---\n')
    print('final scores:')
    print(f'Nr of Parts: {len(bsp.parts)}')
    print(f'Util:        {bsp._objective_util()}')
    print(f'Seam:        {bsp._objective_seam(bsp._get_sum_parts_est_req())}\n')

    print('Exporting Files of best Partition')
    export_bsp(bsp, name=f'out-{model_name}', timing=timing)

if __name__ == '__main__':
    main()
