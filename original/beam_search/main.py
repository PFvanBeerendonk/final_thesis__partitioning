from config import INPUT_FILE_PATH, OUTPUT_FOLDER

import trimesh, os
from beam_search import beam_search


def main():
    print('Loading File')
    current_location = os.path.dirname(__file__)
    mesh_path = f'{current_location}/{INPUT_FILE_PATH}'

    mesh = trimesh.load_mesh(mesh_path)
    
    print('Starting beam search')
    beam_search(mesh)


    print('Exporting Files')
    # sliced_mesh.export(f'{current_location}/{OUTPUT_FOLDER}/out.stl')


if __name__ == '__main__':
    main()
