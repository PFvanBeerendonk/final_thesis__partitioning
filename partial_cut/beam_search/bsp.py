from typing import Self
import numpy as np
import math

from helpers import powerset_no_emptyset
from config import (
    PRINT_VOLUME, PART_WEIGHT, UTIL_WEIGHT, CONNECTOR_WEIGHT,
    CONNECTOR_CONSTANT
)

from trimesh import Trimesh
from trimesh.bounds import oriented_bounds

from trimesh.graph import face_adjacency
import networkx as nx

# Maintain knowledge about parts
class Part:
    def __init__(self, mesh: Trimesh):
        self.mesh = mesh
        
        _, self.extents = oriented_bounds(self.mesh)
        self._update_fit()

    '''
        Returns all parts after cutting self given a cut plane.
    '''
    def cut(self, plane_normal, plane_origin, face_index=None, cap=True) -> list[Self]:
        if face_index:
            faces = [self.mesh.faces[id] for id in face_index]
        else:
            faces = self.mesh.faces
        dot = np.dot(plane_normal, (self.mesh.vertices - plane_origin).T)[faces]

        s = self.mesh.slice_plane(plane_normal=plane_normal,
            plane_origin=plane_origin, cap=cap,
             face_index=face_index)
        dot *= -1
        sa = self.mesh.slice_plane(plane_normal=-plane_normal,
            plane_origin=plane_origin, cap=cap,
            face_index=face_index)

        # temporary hack, should get fixed: https://github.com/mikedh/trimesh/issues/2203
        
        # split mesh into disjoint parts
        s_meshes = s.split()
        sa_meshes = sa.split()
        if len(s_meshes) == 0:
            s_meshes = [s]
        if len(sa_meshes) == 0:
            sa_meshes = [sa]

        return [Part(p) for p in s_meshes + sa_meshes]


    '''
        Given a set of face_ids, and set of faces
        Return C s.t.
        U F \in C F = ids AND
        ∩ F \in C F = []  AND
        ∀ F \in C: i, j \in F <==> faces[i] ∩ faces[j] != []

        In other words, the result is a subdivision of ids, and if 2 face_ids are in the same subset, they are connected
    '''
    def _find_connected(self, ids: list[int]) -> list[list[int]]:
        faces = [self.mesh.faces[i] for i in ids]

        adjacent_faces = face_adjacency(faces)
        graph = nx.Graph()
        graph.add_edges_from(adjacent_faces)
        return nx.connected_components(graph)

    '''
        Cut self.mesh into exactly 2 parts, based on a plane

        @returns list of pairs of parts resulting from cutting using the plane and 'sowing' some cuts back together
    '''
    def twin_cut(self, plane_normal, plane_origin) -> list[list[Self]]:
        # twin_cut(self.mesh, plane_normal, plane_origin)
        
        return
        # find the list of indeces of faces on the plane
        slice2d = self.mesh.section(
            plane_normal=plane_normal,
            plane_origin=plane_origin,
        )
        face_indeces = slice2d.metadata['face_index']

        # determine which faces are connected
        connected_components = list(self._find_connected(face_indeces))

        out_list = []
        for components in powerset_no_emptyset(connected_components):
            face_index = [item for c in components for item in c]

            from helpers import export_part
            export_part(
                Part(mesh=Trimesh(vertices=self.mesh.vertices, faces=[self.mesh.faces[f] for f in face_index])),
                face_index
            )

            print('\n'*2, 'face_index', face_index)
            # cut
            res = twin_cut(self.mesh, plane_normal, plane_origin, face_index, cap=False)
            

            parts = []
            # has exactly 2 parts?
            if len(parts) == 2:
                
                # Trimesh has not implemented `cap` in combination with `face_index`. So we cap ourselves 
                # TODO
                out_list.append(parts)


        return out_list


    """
        Update self.fits_in_volume based on mesh and PRINT_VOLUME params
    """
    def _update_fit(self):
        self.fits_in_volume = all(self.extents[i] <= PRINT_VOLUME[i] for i in range(3))

    def volume(self):
        return self.mesh.volume

    '''
    (upper bound on) the number of print-volumes required
        Checks how many volumes are needed to tile the OBB
    '''
    def est_part_required(self):
        mul = 1
        for i in range(3):
            mul *= math.ceil(self.extents[i] / PRINT_VOLUME[i])
        return mul

class BSP:
    """
        We only need to maintain track of the parts, which are all meshes
    """

    def __init__(self, parts: list[Part], theta_zero = 0, obj_conn = 0):
        if len(parts) == 0:
            raise Exception('Must have at least 1 part')
        self.parts = parts

        if theta_zero == 0:
            self.theta_zero = parts[0].est_part_required()
        else:
            self.theta_zero = theta_zero

        # maintain "on the fly" objectives
        self.obj_conn = obj_conn

    def all_fit(self):
        return all(part.fits_in_volume for part in self.parts)

    def get_largest_part(self) -> Part:
        if len(self.parts) == 0:
            raise Exception('Must have at least 1 part')
        return max(self.parts, key=lambda x: x.volume())

    '''
        cut `part` in `self` using plane, and return new bsp with `part` removed
        and a sub-parts added
    '''
    def cut_part(self, part: Part, plane_normal, plane_origin) -> Self:
        parts = [p for p in self.parts]
        parts.remove(part)

        new_parts = part.cut(plane_normal, plane_origin)

        return BSP(
            parts=parts + new_parts, 
            theta_zero=self.theta_zero,

            # update on_the_fly objectives
            obj_conn=self._update_objectives_at_cut(part, plane_normal, plane_origin),
        )

    
    ### OBJECTIVE FUNCTIONS ###
    def score(self):
        return (
            PART_WEIGHT * self._objective_part() + 
            UTIL_WEIGHT * self._objective_util() +
            CONNECTOR_WEIGHT * self._objective_connector()
        )
    
    

    def _update_objectives_at_cut(self, part: Part, plane_origin, plane_normal):
        return 0

        print('\n')
        # slice_cap, _ = part.mesh.section(plane_origin=plane_origin, plane_normal=plane_normal).to_planar()
        from trimesh.intersections import mesh_plane

        # xx, yy = mesh_plane(part.mesh, plane_normal, plane_origin, return_faces=True)
        # print('uwu', xx)


        cap_3d = part.mesh.section(plane_origin=plane_origin, plane_normal=plane_normal)
        # print(cap_3d.explode())
        # returns Path3D
        cap_2d, _ = cap_3d.to_planar() # maybe pass plane_normal, do a speed check. Should speed it up and this is the plane we wanna project to anyways
        area_g = cap_2d.area
        print()

        from trimesh.path.polygons import paths_to_polygons

        # print( paths_to_polygons(slice_cap.entities))



        raise Exception('end')

        x = 1 # set to Ag/ag - CONNECTOR_CONSTANT
        obj_conn = max(self.obj_conn, x)
        return obj_conn

    def _objective_part(self):
        # 1/Theta * \sum p /in T O(p)
        return 1/self.theta_zero * sum(p.est_part_required() for p in self.parts)

    def _objective_util(self):
        print_volume = PRINT_VOLUME[0] * PRINT_VOLUME[1] * PRINT_VOLUME[2]
        def _util(part):
            return 1 - part.volume() / (part.est_part_required()*print_volume)

        return max(_util(p) for p in self.parts)

    # self.obj_conn is initially 0 (as no cuts have been made)
    # Every time a cut is made, we update obj_conn
    def _objective_connector(self):
        return self.obj_conn
