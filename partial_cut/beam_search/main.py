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


    print('Starting beam search, start time')
    start_time = time.time()

    bsp = beam_search(mesh)

    end_time = time.time()
    print('Finished beam search, end time')
    print(f'\n--- Time elapsed = {end_time - start_time} ---\n')

    print('Exporting Files of best Partition')
    export_bsp(bsp)

if __name__ == '__main__':
    main()
