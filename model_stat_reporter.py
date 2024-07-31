

from trimesh.bounds import oriented_bounds

import trimesh, os



if __name__ == '__main__':
    folder = '/models'
    input_file_paths=['propeller.stl', 'tue_logo.stl', 'models/donut.stl']
    current_location = os.path.dirname(__file__) + folder

    files = os.listdir(current_location)

    
    print('Get bounds: H D W \n')

    for file in files:
        if file.endswith(".stl"):
            mesh_path = f'{current_location}/{file}'

            mesh = trimesh.load_mesh(mesh_path)
            _, obb = oriented_bounds(mesh)
            h = round(obb[0], 3)
            d = round(obb[1], 3)
            w = round(obb[2], 3)

            print(file)
            print(f'{h} & {d} & {w} & {mesh.volume} & {len(mesh.vertices)} & {len(mesh.edges)} & {len(mesh.faces)} \\\\')
            print('\n')

    
        