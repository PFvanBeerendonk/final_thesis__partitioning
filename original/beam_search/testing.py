from config import (
    INPUT_FILE_PATH,
    OUTPUT_FOLDER,
    BEAM_WIDTH,
)
import trimesh, os
from trimesh.intersections import slice_mesh_plane
from bsp import BSP


def main():
    current_location = os.path.dirname(__file__)
    mesh_path = f'{current_location}/{INPUT_FILE_PATH}'

    mesh = trimesh.load_mesh(mesh_path)


    print('Exporting Files')
    sliced_mesh.export(f'{current_location}/{OUTPUT_FOLDER}/out.stl')

    plane_normal = (0, 0, -1)
    plane_origin = (0, 0, 16)
    sliced_mesh = slice_mesh_plane(
        mesh,
        plane_normal,
        plane_origin
    )


if __name__ == '__main__':
    main()
