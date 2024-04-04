from config import INPUT_FILE_PATH

import trimesh, os
from beam_search import beam_search
from helpers import export_bsp

def main():
    print('Loading File')
    current_location = os.path.dirname(__file__)
    mesh_path = f'{current_location}/{INPUT_FILE_PATH}'

    mesh = trimesh.load_mesh(mesh_path)
    
    # TODO: maybe validate the input mesh, 
    # - watertightness
    # - is one part


    print('Starting beam search')
    bsp = beam_search(mesh)
    print('Finished beam search')

    print('Exporting Files of best Partition')
    export_bsp(bsp)

if __name__ == '__main__':
    main()
