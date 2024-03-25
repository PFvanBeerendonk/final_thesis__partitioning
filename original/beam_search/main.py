from config import INPUT_FILE_PATH, OUTPUT_FOLDER

import trimesh, os
from beam_search import beam_search


def main():
    print('Loading File')
    current_location = os.path.dirname(__file__)
    mesh_path = f'{current_location}/{INPUT_FILE_PATH}'

    mesh = trimesh.load_mesh(mesh_path)
    
    # maybe validate the input mesh, to check for watertightness
    
    print('Starting beam search')
    bsp = beam_search(mesh)
    print('Finished beam search')

    print('Exporting Files of best Partition')
    for i, part in enumerate(bsp.parts):
        part.mesh.export(f'{current_location}/{OUTPUT_FOLDER}/out{i}.stl')


if __name__ == '__main__':
    main()
