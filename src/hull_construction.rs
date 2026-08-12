// ! This part contains the real construction of the convex hull.

use crate::geometry_helper::Triangle;
use crate::{ConvexHullError, TriangleIndices};
use fxhash::FxHashSet;
use glam::Vec3;

const RELATIVE_TOLERANCE: f32 = 1e-5;

/// Computes the numerical tolerance based on the bounding box diagonal
pub(crate) fn compute_tolerance_value(vertices: &[Vec3]) -> f32 {
    let (min, max) = vertices
        .iter()
        .fold(
            (Vec3::INFINITY, Vec3::NEG_INFINITY),
            |(lo, hi), v| (lo.min(*v), hi.max(*v)),
        );
    (max - min).length() * RELATIVE_TOLERANCE
}

pub(crate) struct HullConstructor<'a> {
    /// The vertices we still need to process.
    vertices: &'a [Vec3],
    /// The hull as it currently stands in the construction process.
    hull_triangles: Vec<Triangle<'a>>,
    /// The barrier we use for tolerance checks.
    tolerance: f32,
}

/// Applies the probe_function to all vertices and finds the element with the highest value,
/// the index of that vertex is returned and eliminated from the processing list.
fn get_best_index_and_remove(
    vertex_list: &mut Vec<usize>,
    probe_function: impl Fn(usize) -> f32 + Sync,
) -> usize {
    let (inner_index, result, _) = vertex_list
        .iter()
        .enumerate()
        .map(|(field_ind, vert_ind)| (field_ind, vert_ind, probe_function(*vert_ind)))
        .max_by(|(_, _, val_a), (_, _, val_b)| val_a.total_cmp(val_b))
        .expect("There should be at least one vertex left");

    let res = *result;
    vertex_list.swap_remove(inner_index);
    res
}

/// Assigns the vertices handed over, except the exclusion vertex over to the triangles. We pick the triangle we have the largest distance to
/// the exlusion vertex is discarded, this is typically the new vertex coming in. A certain minimum distance must be kept.
fn assign_vertices_to_tris(
    vertices: &[usize],
    exclusion_vert: usize,
    triangles: &mut [Triangle],
    tolerance: f32,
) {
    let transfer = vertices
        .iter()
        .filter(|&x| *x != exclusion_vert)
        .filter_map(|vert_index| {
            let (best_tri, _) = triangles.iter().enumerate().fold(
                (None, tolerance),
                |(best, best_dist), (tri_index, triangle)| {
                    let dist = triangle.get_signed_distance(*vert_index);
                    if dist > best_dist {
                        (Some((tri_index, vert_index)), dist)
                    } else {
                        (best, best_dist)
                    }
                },
            );
            best_tri
        })
        .collect::<Vec<_>>();

    for (tri_index, &vert_index) in transfer {
        triangles[tri_index].assign_vertex(vert_index);
    }
}

impl<'a> HullConstructor<'a> {
    /// Vertices are handed over.
    pub(crate) fn new(vertices: &'a [Vec3]) -> Self {
        Self {
            vertices,
            hull_triangles: Vec::new(),
            tolerance: 1.0,
        }
    }

    /// Builds an initial tetrahedron from the inner vertices.
    fn build_initial_tetrahedron(&mut self) -> Result<(), ConvexHullError> {
        debug_assert!(
            self.hull_triangles.is_empty(),
            "The initial tetrahedron must be empty."
        );

        // Field gets a linear scale.
        self.tolerance = compute_tolerance_value(self.vertices);

        let mut vertex_list = (0..self.vertices.len()).collect::<Vec<_>>();

        // The first vertex is the one furthest out in x
        let i0 = get_best_index_and_remove(&mut vertex_list, |i| self.vertices[i].x);
        let pos0 = self.vertices[i0];
        // The second vertex is the one furthest away from first.
        let i1 = get_best_index_and_remove(&mut vertex_list, |i| {
            (self.vertices[i] - pos0).length_squared()
        });
        let pos1 = self.vertices[i1];
        let a = pos1 - pos0;
        let dir_a = a.normalize_or_zero();
        // The third vertex is the one furthest away from the edge.
        let i2 = get_best_index_and_remove(&mut vertex_list, |i| {
            let point_on_line = pos0 + (self.vertices[i] - pos0).dot(dir_a) * dir_a;
            (self.vertices[i] - point_on_line).length_squared()
        });
        let pos2 = self.vertices[i2];
        let b = pos2 - pos0;
        // Gram Schmidt step.
        let dir_b = (b - b.dot(dir_a) * dir_a).normalize_or_zero();
        // The fourth and last point is the point the furthest away from the constructed plane.
        let i3 = get_best_index_and_remove(&mut vertex_list, |i| {
            let delta = self.vertices[i] - pos0;
            let point_on_plane = pos0 + delta.dot(dir_a) * dir_a + delta.dot(dir_b) * dir_b;
            (self.vertices[i] - point_on_plane).length_squared()
        });
        let pos3 = self.vertices[i3];
        let c = pos3 - pos0;

        // Let us check for degeneration
        let volume_scale = a.length() * b.length() * c.length();
        if volume_scale <= 0.0 || (a.cross(b).dot(c).abs() / volume_scale) < 1e-6 {
            return Err(ConvexHullError::DegenerateInput);
        }

        let mut triangles = vec![
            Triangle::new(self.vertices, [i0, i1, i2]),
            Triangle::new(self.vertices, [i1, i0, i3]),
            Triangle::new(self.vertices, [i2, i1, i3]),
            Triangle::new(self.vertices, [i3, i0, i2]),
        ];

        if triangles[0].get_signed_distance(i3) > 0.0 {
            triangles = triangles
                .into_iter()
                .map(|tri| tri.get_flipped_version())
                .collect();
        }

        // Vertex i0 is removed anyway.
        assign_vertices_to_tris(
            &vertex_list,
            i0,
            &mut triangles,
            self.tolerance,
        );

        self.hull_triangles = triangles;
        Ok(())
    }

    /// Gets the best candidate if it exists.
    fn get_best_vertex(&self) -> Option<usize> {
        let (candidate,_) = self.hull_triangles.iter().fold(
            (None, f32::NEG_INFINITY),
            |(probe, best_dist), tri| {
                if let Some((vertex, dist)) = tri.furthest_vertex_and_dist() && dist > best_dist {
                    (Some(vertex), dist)
                } else {
                    (probe, best_dist)
                }
            },
        );

        candidate
    }

    /// The inner call to generate a convex hull.
    pub(crate) fn generate_convex_hull(&mut self) -> Result<Vec<TriangleIndices>, ConvexHullError> {
        self.build_initial_tetrahedron()?;
        while let Some(next_vertex) = self.get_best_vertex() {

            // Mark and collect
            let mut vertices_to_reassign = Vec::new();
            let mut all_edges = FxHashSet::default();

            // Collect triangle indices to delete.
            let mut to_delete = Vec::new();
            for (i, tri) in self.hull_triangles.iter().enumerate() {
                if tri.get_signed_distance(next_vertex) > 0.0 {
                    to_delete.push(i);
                }
            }

            // Delete triangles, reassign vertices and collect edges.
            for &i in to_delete.iter().rev() {
                let tri = self.hull_triangles.swap_remove(i);
                vertices_to_reassign.extend(tri.regarded_vertices());
                all_edges.extend(tri.edges());
            }

            // Get the seam edges.
            let boundary: Vec<_> = all_edges.iter()
                .filter(|e| !all_edges.contains(&e.reversed()))
                .copied()
                .collect();

            // Create new triangles and reassign vertices.            
            let mut new_triangles: Vec<_> = boundary.iter()
                .map(|e| Triangle::from_edge_and_points(self.vertices, e, next_vertex))
                .collect();
            assign_vertices_to_tris(&vertices_to_reassign, next_vertex, &mut new_triangles, self.tolerance);
            self.hull_triangles.extend(new_triangles);
        }

        Ok(self
            .hull_triangles
            .iter()
            .map(|tri| tri.get_triple_representation())
            .collect())
    }
}
