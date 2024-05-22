from trimesh import Trimesh
import numpy as np

from trimesh.graph import face_adjacency
import networkx as nx

from helpers import powerset_no_emptyset
from bsp import Part


def _find_connected(mesh, ids: list[int]) -> list[list[int]]:
        faces = [mesh.faces[i] for i in ids]

        adjacent_faces = face_adjacency(faces)
        graph = nx.Graph()
        graph.add_edges_from(adjacent_faces)
        return nx.connected_components(graph)


def twin_cut(mesh, plane_normal, plane_origin) -> list[list[Part]]:
    slice2d = mesh.section(
        plane_normal=plane_normal,
        plane_origin=plane_origin,
    )
    face_indeces = slice2d.metadata['face_index']

    # determine which faces are connected
    connected_components = list(_find_connected(face_indeces))

    out_list = []
    for components in powerset_no_emptyset(connected_components):
        face_index = [item for c in components for item in c]

        from helpers import export_part
        export_part(
            Part(mesh=Trimesh(vertices=mesh.vertices, faces=[mesh.faces[f] for f in face_index])),
            face_index
        )

        print('\n'*2, 'face_index', face_index)
        # cut
        slice_mesh_plane(
            mesh, plane_normal=plane_normal, plane_origin=plane_origin, 
            face_index=face_index, cap=False, 
        )
        parts = []
        
        
        # has exactly 2 parts?
        if len(parts) == 2:
            
            # Trimesh has not implemented `cap` in combination with `face_index`. So we cap ourselves 
            # TODO
            out_list.append(parts)


    return out_list


def cut(mesh, plane_normal, plane_origin, face_index=None, cap=True) -> list[Part]:

    s = slice_mesh_plane(mesh, plane_normal=plane_normal,
        plane_origin=plane_origin, cap=cap, face_index=face_index,
    )

    # temporary hack, should get fixed: https://github.com/mikedh/trimesh/issues/2203
    
    # split mesh into disjoint parts
    return s




from trimesh import grouping, geometry, util
from trimesh import transformations as tf
from trimesh.constants import tol
from trimesh import triangles as tm
from scipy.spatial import cKDTree

from trimesh.base import Trimesh
from trimesh.creation import triangulate_polygon
from trimesh.path import polygons

# Adapted from Trimesh
def slice_mesh_plane(
    mesh,
    plane_normal,
    plane_origin,
    cap=False,
    cached_dots=None,
    engine=None,
    face_index=None,
    **kwargs,
):
    """
    slice a mesh with a plane, but ONLY cut the faces with face_index.

    Return a Trimesh for each side of the plane

    Slice a mesh with a plane returning a new mesh that is the
    portion of the original mesh to the positive normal side
    of the plane.

    Parameters
    ---------
    mesh : Trimesh object
      Source mesh to slice
    plane_normal : (3,) float
      Normal vector of plane to intersect with mesh
    plane_origin :  (3,) float
      Point on plane to intersect with mesh
    cap : bool
      If True, cap the result with a triangulated polygon
    cached_dots : (n, 3) float
      If an external function has stored dot
      products pass them here to avoid recomputing
    engine : None or str
      Triangulation engine passed to `triangulate_polygon`
    kwargs : dict
      Passed to the newly created sliced mesh

    Returns
    ----------
    new_mesh : Trimesh object
      Sliced mesh
    """

    # check input plane
    plane_normal = np.asanyarray(plane_normal, dtype=np.float64)
    plane_origin = np.asanyarray(plane_origin, dtype=np.float64)

    # start with copy of original mesh, faces, and vertices
    vertices = mesh.vertices.copy()
    faces = mesh.faces.copy()

    # slice away specified planes
    for origin, normal in zip(
        plane_origin.reshape((-1, 3)), plane_normal.reshape((-1, 3))
    ):
        # save the new vertices and faces
        vertices, faces = slice_faces_plane(
            vertices=vertices,
            faces=faces,
            plane_normal=normal,
            plane_origin=origin,
            cached_dots=cached_dots,
        )
        # check if cap arg specified
        if cap:
            # start by deduplicating vertices again
            unique, inverse = grouping.unique_rows(vertices)
            vertices = vertices[unique]
            # will collect additional faces
            f = inverse[faces]
            # remove degenerate faces by checking to make sure
            # that each face has three unique indices
            f = f[(f[:, :1] != f[:, 1:]).all(axis=1)]
            # transform to the cap plane
            to_2D = geometry.plane_transform(origin=origin, normal=-normal)
            to_3D = np.linalg.inv(to_2D)

            vertices_2D = tf.transform_points(vertices, to_2D)
            edges = geometry.faces_to_edges(f)
            edges.sort(axis=1)

            on_plane = np.abs(vertices_2D[:, 2]) < 1e-8
            edges = edges[on_plane[edges].all(axis=1)]
            edges = edges[edges[:, 0] != edges[:, 1]]
            
            unique_edge = grouping.group_rows(edges, require_count=1)
            if len(unique) < 3:
                continue

            tree = cKDTree(vertices)
            # collect new faces
            faces = [f]
            for p in polygons.edges_to_polygons(edges[unique_edge], vertices_2D[:, :2]):
                vn, fn = triangulate_polygon(p, engine=engine)
                # collect the original index for the new vertices
                vn3 = tf.transform_points(util.stack_3D(vn), to_3D)
                distance, vid = tree.query(vn3)
                if distance.max() > 1e-8:
                    util.log.debug("triangulate may have inserted vertex!")
                # triangulation should not have inserted vertices
                faces.append(vid[fn])
            faces = np.vstack(faces)

    # return the sliced mesh
    return Trimesh(vertices=vertices, faces=faces, **kwargs)


def slice_faces_plane(
    vertices,
    faces,
    plane_normal,
    plane_origin,
    cached_dots=None,
):
    """
    Slice a mesh (given as a set of faces and vertices) with a plane, returning a
    new mesh (again as a set of faces and vertices) that is the
    portion of the original mesh to the positive normal side of the plane.

    Parameters
    ---------
    vertices : (n, 3) float
        Vertices of source mesh to slice
    faces : (n, 3) int
        Faces of source mesh to slice
    plane_normal : (3,) float
        Normal vector of plane to intersect with mesh
    plane_origin :  (3,) float
        Point on plane to intersect with mesh
    face_index : ((m,) int)
        Indexes of faces to slice. When no mask is provided, the
        default is to slice all faces.
    cached_dots : (n, 3) float
        If an external function has stored dot
        products pass them here to avoid recomputing

    Returns
    ----------
    new_vertices : (n, 3) float
        Vertices of sliced mesh
    new_faces : (n, 3) int
        Faces of sliced mesh
    """

    if len(vertices) == 0:
        return vertices, faces

    if cached_dots is not None:
        dots = cached_dots
    else:
        # dot product of each vertex with the plane normal indexed by face
        # so for each face the dot product of each vertex is a row
        # shape is the same as faces (n,3)
        dots = np.dot(vertices - plane_origin, plane_normal)

    # Find vertex orientations w.r.t. faces for all triangles:
    #  -1 -> vertex "inside" plane (positive normal direction)
    #   0 -> vertex on plane
    #   1 -> vertex "outside" plane (negative normal direction)
    signs = np.zeros(len(vertices), dtype=np.int8)
    signs[dots < -tol.merge] = 1
    signs[dots > tol.merge] = -1
    signs = signs[faces]

    # Find all triangles that intersect this plane
    # onedge <- indices of all triangles intersecting the plane
    # inside <- indices of all triangles "inside" the plane (positive normal)
    signs_sum = signs.sum(axis=1, dtype=np.int8)
    signs_asum = np.abs(signs).sum(axis=1, dtype=np.int8)

    # Cases:
    # (0,0,0),  (-1,0,0),  (-1,-1,0), (-1,-1,-1) <- inside
    # (1,0,0),  (1,1,0),   (1,1,1)               <- outside
    # (1,0,-1), (1,-1,-1), (1,1,-1)              <- onedge
    onedge = np.logical_and(signs_asum >= 2, np.abs(signs_sum) <= 1)

    inside = signs_sum == -signs_asum

    # for any faces that lie exactly on-the-plane
    # we want to only include them if their normal
    # is backwards from the slicing normal
    on_plane = signs_asum == 0
    if on_plane.any():
        # compute the normals and whether
        # face is degenerate here
        check, valid = tm.normals(vertices[faces[on_plane]])
        # only include faces back from normal
        dot_check = np.dot(check, plane_normal)
        # exclude any degenerate faces from the result
        inside[on_plane] = valid
        # exclude the degenerate face from our mask
        on_plane[on_plane] = valid
        # apply results for this subset
        inside[on_plane] = dot_check < 0.0

    # Automatically include all faces that are "inside"
    new_faces = faces[inside]

    # Separate faces on the edge into two cases: those which will become
    # quads (two vertices inside plane) and those which will become triangles
    # (one vertex inside plane)
    triangles = vertices[faces]
    cut_triangles = triangles[onedge]
    cut_faces_quad = faces[np.logical_and(onedge, signs_sum < 0)]
    cut_faces_tri = faces[np.logical_and(onedge, signs_sum >= 0)]
    cut_signs_quad = signs[np.logical_and(onedge, signs_sum < 0)]
    cut_signs_tri = signs[np.logical_and(onedge, signs_sum >= 0)]

    # If no faces to cut, the surface is not in contact with this plane.
    # Thus, return a mesh with only the inside faces
    if len(cut_faces_quad) + len(cut_faces_tri) == 0:
        if len(new_faces) == 0:
            # if no new faces at all return empty arrays
            empty = (
                np.zeros((0, 3), dtype=np.float64),
                np.zeros((0, 3), dtype=np.int64),
            )
            return empty

        # find the unique indices in the new faces
        # using an integer-only unique function
        unique, inverse = grouping.unique_bincount(
            new_faces.reshape(-1), minlength=len(vertices), return_inverse=True
        )

        # use the unique indices for our final vertices and faces
        final_vert = vertices[unique]
        final_face = inverse.reshape((-1, 3))

        return final_vert, final_face

    # Extract the intersections of each triangle's edges with the plane
    o = cut_triangles  # origins
    d = np.roll(o, -1, axis=1) - o  # directions
    num = (plane_origin - o).dot(plane_normal)  # compute num/denom
    denom = np.dot(d, plane_normal)
    denom[denom == 0.0] = 1e-12  # prevent division by zero
    dist = np.divide(num, denom)
    # intersection points for each segment
    int_points = np.einsum("ij,ijk->ijk", dist, d) + o

    # Initialize the array of new vertices with the current vertices
    new_vertices = vertices
    new_quad_vertices = np.zeros((0, 3))
    new_tri_vertices = np.zeros((0, 3))

    # Handle the case where a new quad is formed by the intersection
    # First, extract the intersection points belonging to a new quad
    quad_int_points = int_points[(signs_sum < 0)[onedge], :, :]
    num_quads = len(quad_int_points)
    if num_quads > 0:
        # Extract the vertex on the outside of the plane, then get the vertices
        # (in CCW order of the inside vertices)
        quad_int_inds = np.where(cut_signs_quad == 1)[1]
        quad_int_verts = cut_faces_quad[
            np.stack((range(num_quads), range(num_quads)), axis=1),
            np.stack(((quad_int_inds + 1) % 3, (quad_int_inds + 2) % 3), axis=1),
        ]

        # Fill out new quad faces with the intersection points as vertices
        new_quad_faces = np.append(
            quad_int_verts,
            np.arange(len(new_vertices), len(new_vertices) + 2 * num_quads).reshape(
                num_quads, 2
            ),
            axis=1,
        )

        # Extract correct intersection points from int_points and order them in
        # the same way as they were added to faces
        new_quad_vertices = quad_int_points[
            np.stack((range(num_quads), range(num_quads)), axis=1),
            np.stack((((quad_int_inds + 2) % 3).T, quad_int_inds.T), axis=1),
            :,
        ].reshape(2 * num_quads, 3)

        # Add new vertices to existing vertices, triangulate quads, and add the
        # resulting triangles to the new faces
        new_vertices = np.append(new_vertices, new_quad_vertices, axis=0)
        new_tri_faces_from_quads = geometry.triangulate_quads(new_quad_faces)
        new_faces = np.append(new_faces, new_tri_faces_from_quads, axis=0)

    # Handle the case where a new triangle is formed by the intersection
    # First, extract the intersection points belonging to a new triangle
    tri_int_points = int_points[(signs_sum >= 0)[onedge], :, :]
    num_tris = len(tri_int_points)
    if num_tris > 0:
        # Extract the single vertex for each triangle inside the plane and get the
        # inside vertices (CCW order)
        tri_int_inds = np.where(cut_signs_tri == -1)[1]
        tri_int_verts = cut_faces_tri[range(num_tris), tri_int_inds].reshape(num_tris, 1)

        # Fill out new triangles with the intersection points as vertices
        new_tri_faces = np.append(
            tri_int_verts,
            np.arange(len(new_vertices), len(new_vertices) + 2 * num_tris).reshape(
                num_tris, 2
            ),
            axis=1,
        )

        # Extract correct intersection points and order them in the same way as
        # the vertices were added to the faces
        new_tri_vertices = tri_int_points[
            np.stack((range(num_tris), range(num_tris)), axis=1),
            np.stack((tri_int_inds.T, ((tri_int_inds + 2) % 3).T), axis=1),
            :,
        ].reshape(2 * num_tris, 3)

        # Append new vertices and new faces
        new_vertices = np.append(new_vertices, new_tri_vertices, axis=0)
        new_faces = np.append(new_faces, new_tri_faces, axis=0)

    # find the unique indices in the new faces
    # using an integer-only unique function
    unique, inverse = grouping.unique_bincount(
        new_faces.reshape(-1), minlength=len(new_vertices), return_inverse=True
    )

    # use the unique indexes for our final vertex and faces
    final_vert = new_vertices[unique]
    final_face = inverse.reshape((-1, 3))

    return final_vert, final_face
