from typing import Self
import numpy as np
import math

from config import (
    PRINT_VOLUME, PART_WEIGHT, UTIL_WEIGHT, SEAM_WEIGHT
)

PRINT_VOLUME_CALCULATED = PRINT_VOLUME[0] * PRINT_VOLUME[1] * PRINT_VOLUME[2]

from helpers import calculate_eps_objective_seam
from trimesh import Trimesh
from trimesh.bounds import oriented_bounds

# Maintain knowledge about parts
class Part:
    def __init__(self, mesh: Trimesh):
        self.mesh = mesh
        
        _, self.extents = oriented_bounds(self.mesh)
        self._update_fit()

    '''
        Returns all parts after cutting self given a cut plane.
    '''
    def cut(self, plane_normal, plane_origin) -> list[Self]:
        dot = np.dot(plane_normal, (self.mesh.vertices - plane_origin).T)[self.mesh.faces]

        s, eps_seam_s = slice_mesh_plane(self.mesh, plane_normal=plane_normal,
            plane_origin=plane_origin, cap=True,
            cached_dots=dot)
        dot *= -1
        sa, eps_seam_sa = slice_mesh_plane(self.mesh, plane_normal=-plane_normal,
            plane_origin=plane_origin, cap=True,
            cached_dots=dot)

        # temporary hack, should get fixed: https://github.com/mikedh/trimesh/issues/2203
        
        # split mesh into disjoint parts
        s_meshes = s.split()
        sa_meshes = sa.split()
        if len(s_meshes) == 0:
            s_meshes = [s]
        if len(sa_meshes) == 0:
            sa_meshes = [sa]

        eps_seam = eps_seam_s + eps_seam_sa

        return [Part(p) for p in s_meshes + sa_meshes], eps_seam


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

    def __init__(self, parts: list[Part],  one_over_theta_zero=0, seam_sum=0, latest_eps_seam=0, one_over_diagonal_zero=0, latest_normal=None, latest_origin=None):
        if len(parts) == 0:
            raise Exception('Must have at least 1 part')
        self.parts = parts

        if one_over_theta_zero == 0:
            self.one_over_theta_zero = 1 / parts[0].est_part_required()
        else:
            self.one_over_theta_zero = one_over_theta_zero

        if one_over_diagonal_zero == 0:
            # length of diagonal of OBB, is sqrt(x^2 + y^2 + z^2)
            self.one_over_diagonal_zero = 1/math.sqrt(sum(e**2 for e in parts[0].extents))
        else:
            self.one_over_diagonal_zero = one_over_diagonal_zero

        ### maintain "on the fly" objectives ###
        # First sum in seam objective: \sum_{C \in T}eps(C)
        self.seam_sum = seam_sum
        self.latest_eps_seam = latest_eps_seam

        # maintain sufficiently different
        self.latest_normal=latest_normal
        self.latest_origin=latest_origin

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

        new_parts, eps_seam = part.cut(
            plane_normal=plane_normal, 
            plane_origin=plane_origin,
        )

        return BSP(
            parts=parts + new_parts, 
            one_over_theta_zero=self.one_over_theta_zero,
            seam_sum=self.seam_sum + eps_seam * self.one_over_diagonal_zero,
            latest_eps_seam=eps_seam * self.one_over_diagonal_zero,
            one_over_diagonal_zero=self.one_over_diagonal_zero,
            latest_normal=plane_normal,
            latest_origin=plane_origin,
        )

    
    ### OBJECTIVE FUNCTIONS ###
    def score(self):
        sum_parts_est_req = self._get_sum_parts_est_req()
        return (
            PART_WEIGHT * self._objective_part(sum_parts_est_req) + 
            UTIL_WEIGHT * self._objective_util() +
            SEAM_WEIGHT * self._objective_seam(sum_parts_est_req)
        )
    
    def _get_sum_parts_est_req(self):
        return sum(p.est_part_required() for p in self.parts)

    def _objective_part(self, sum_parts_est_req):
        # 1/Theta * \sum p /in T O(p)
        return self.one_over_theta_zero * sum_parts_est_req

    def _objective_util(self):
        def _util(part):
            return 1 - (part.volume() / (part.est_part_required() * PRINT_VOLUME_CALCULATED))

        print([_util(p) for p in self.parts])
        print(self.parts)
        return max(_util(p) for p in self.parts)

    def _objective_seam(self, sum_parts_est_req):
        nr_of_cuts_todo = sum_parts_est_req - len(self.parts)

        return self.one_over_theta_zero * (
            self.seam_sum + self.latest_eps_seam * nr_of_cuts_todo
        )



# trimesh adaptation, adding eps_seam calculation to slice_mesh_plane
# we did remove some small stuff (uv, visual, face_index) that was not used/needed
from trimesh import geometry, grouping, util
from trimesh import transformations as tf
# from trimesh.intersections import slice_faces_plane
from trimesh import triangles as tm
from trimesh.constants import tol
from trimesh.triangles import points_to_barycentric

def slice_mesh_plane(
    mesh,
    plane_normal,
    plane_origin,
    cap=False,
    engine=None,
    **kwargs,
):
    """
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
    face_index : ((m,) int)
      Indexes of mesh.faces to slice. When no mask is provided, the
      default is to slice all faces.
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
    eps_seam : float
      eps seam score
    """
    # check input for none
    if mesh is None:
        return None

    # avoid circular import
    from scipy.spatial import cKDTree

    from trimesh.base import Trimesh
    from trimesh.creation import triangulate_polygon
    from trimesh.path import polygons

    # check input plane
    plane_normal = np.asanyarray(plane_normal, dtype=np.float64)
    plane_origin = np.asanyarray(plane_origin, dtype=np.float64)

    # check to make sure origins and normals have acceptable shape
    shape_ok = (
        (plane_origin.shape == (3,) or util.is_shape(plane_origin, (-1, 3)))
        and (plane_normal.shape == (3,) or util.is_shape(plane_normal, (-1, 3)))
        and plane_origin.shape == plane_normal.shape
    )
    if not shape_ok:
        raise ValueError("plane origins and normals must be (n, 3)!")

    # start with copy of original mesh, faces, and vertices
    vertices = mesh.vertices.copy()
    faces = mesh.faces.copy()

    if "process" not in kwargs:
        kwargs["process"] = False

    # added to find seam
    slice2d = mesh.section(
        plane_normal=plane_normal,
        plane_origin=plane_origin,
    )
    path = slice2d.to_planar()[0]
    seam_length = path.length

    # slice away specified planes
    for origin, normal in zip(
        plane_origin.reshape((-1, 3)), plane_normal.reshape((-1, 3))
    ):
        # save the new vertices and faces
        vertices, faces, eps_seam = slice_faces_plane(
            vertices=vertices,
            faces=faces,
            plane_normal=normal,
            plane_origin=origin,
            seam_length=seam_length,
        )
        # CAP
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
            nf = vid[fn]
            # hmm but it may have returned faces that are now degenerate
            nf_ok = (nf[:, 1:] != nf[:, :1]).all(axis=1) & (nf[:, 1] != nf[:, 2])
            faces.append(nf[nf_ok])

        faces = np.vstack(faces)

    if eps_seam <= 0.0:
        raise Exception('not possible, reject')
    
    # return the sliced mesh
    return Trimesh(vertices=vertices, faces=faces, **kwargs), eps_seam

def slice_faces_plane(
    vertices,
    faces,
    plane_normal,
    plane_origin,
    face_index=None,
    cached_dots=None,
    seam_length=0,
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
    uv : (n, 2) float, optional
        UV coordinates of source mesh to slice
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
    eps_seam : int
        seam score
    """

    if len(vertices) == 0:
        return vertices, faces, 0


    # Construct a mask for the faces to slice.
    if face_index is not None:
        faces = faces[face_index]

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
                0,
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

        return final_vert, final_face, 0

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

    eps_seam = calculate_eps_objective_seam(vertices.copy(), faces.copy(), len(new_vertices) - len(vertices), seam_length)

    # find the unique indices in the new faces
    # using an integer-only unique function
    unique, inverse = grouping.unique_bincount(
        new_faces.reshape(-1), minlength=len(new_vertices), return_inverse=True
    )

    # use the unique indexes for our final vertex and faces
    final_vert = new_vertices[unique]
    final_face = inverse.reshape((-1, 3))

    return final_vert, final_face, eps_seam
