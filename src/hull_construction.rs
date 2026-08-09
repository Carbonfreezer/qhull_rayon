// ! This part contains the real construction of the convex hull.

use crate::geometry_helper::Triangle;
use crate::{ConvexHullError, TriangleIndices};
use fxhash::FxHashSet;
use glam::Vec3;
use rayon::prelude::{
    IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator, ParallelIterator,
};
use std::mem::take;

pub const RELATIVE_TOLERANCE: f32 = 1e-5;

pub(crate) struct HullConstructor<'a> {
    /// The vertices we still need to process.
    vertices: &'a [Vec3],
    /// The hull as it currently stands in the construction process.
    hull_triangles: Vec<Triangle<'a>>,
    /// The list with the indices into the vertices that still have to be processed.
    indices_to_process: Vec<usize>,
    /// The barrier we use for tolerance checks.
    tolerance: f32,
}

impl<'a> HullConstructor<'a> {
    /// Vertices are handed over.
    pub(crate) fn new(vertices: &'a [Vec3]) -> Self {
        let indices_to_process = (0..vertices.len()).collect::<Vec<_>>();
        Self {
            vertices,
            indices_to_process,
            hull_triangles: Vec::new(),
            tolerance: 1.0,
        }
    }

    /// Gets the best index pair as index into the inner vector and the vertex position.
    fn get_best_index(&self, probe_function: impl Fn(usize) -> f32 + Sync) -> (usize, usize) {
        let (inner_index, result, _) = self
            .indices_to_process
            .par_iter()
            .enumerate()
            .map(|(field_ind, vert_ind)| (field_ind, vert_ind, probe_function(*vert_ind)))
            .max_by(|(_, _, val_a), (_, _, val_b)| val_a.total_cmp(val_b))
            .expect("There should be at least one vertex left");
        (inner_index, *result)
    }
    /// Applies the probe_function to all vertices and finds the element with the highest value,
    /// the index of that vertex is returned and eliminated from the processing list.
    fn get_best_index_and_remove(&mut self, probe_function: impl Fn(usize) -> f32 + Sync) -> usize {
        let (inner_index, result) = self.get_best_index(probe_function);
        self.indices_to_process.swap_remove(inner_index);
        result
    }

    /// Builds an initial tetrahedron from the inner vertices.
    fn build_initial_tetrahedron(&mut self) -> Result<(), ConvexHullError> {
        debug_assert!(
            self.hull_triangles.is_empty(),
            "The initial tetrahedron must be empty."
        );

        // The first vertex is the one furthest out in x
        let i0 = self.get_best_index_and_remove(|i| self.vertices[i].x);
        let pos0 = self.vertices[i0];
        // The second vertex is the one furthest away from first.
        let i1 = self.get_best_index_and_remove(|i| (self.vertices[i] - pos0).length_squared());
        let pos1 = self.vertices[i1];
        let a = pos1 - pos0;
        let dir_a = a.normalize();
        // The third vertex is the one furthest away from the edge.
        let i2 = self.get_best_index_and_remove(|i| {
            let point_on_line = pos0 + (self.vertices[i] - pos0).dot(dir_a) * dir_a;
            (self.vertices[i] - point_on_line).length_squared()
        });
        let pos2 = self.vertices[i2];
        let b = pos2 - pos0;
        // Gram Schmidt step.
        let dir_b = (b - b.dot(dir_a) * dir_a).normalize_or_zero();
        // The fourth and last point is the point the furthest away from the constructed plane.
        let i3 = self.get_best_index_and_remove(|i| {
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
        // Field gets a linear scale.
        self.tolerance = RELATIVE_TOLERANCE * a.length();

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

        self.hull_triangles = triangles;
        Ok(())
    }

    /// Analyzes the existing vertices and finds the index that is furthest away from the existing hull
    /// and removes all vertices that are inside the hull. If there is no outer vertex left it returns None.
    fn get_best_vertex_index_and_sweep(&mut self) -> Option<usize> {
        // Get highest signed distance for every vertex.
        let highest_signed_distance = self
            .indices_to_process
            .par_iter()
            .map(|i| {
                self.hull_triangles
                    .iter()
                    .map(|tri| tri.get_signed_distance(*i))
                    .fold(f32::NEG_INFINITY, f32::max)
            })
            .collect::<Vec<_>>();
        // Get the position with the max value.
        let (best_position, highest_value) = highest_signed_distance
            .par_iter()
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(b.1))
            .expect("indices_to_process is non-empty per loop condition");

        // In this case we have only inner points left.
        if *highest_value <= self.tolerance {
            self.indices_to_process.clear();
            return None;
        }

        let next_vertex = self.indices_to_process[best_position];
        // Filter out the all vertices we do not need any more.
        debug_assert_eq!(
            self.indices_to_process.len(),
            highest_signed_distance.len(),
            "The two vectors should be of same length"
        );
        self.indices_to_process = self
            .indices_to_process
            .par_iter()
            .zip(highest_signed_distance.into_par_iter())
            .filter_map(|(&i, dist)| (i != next_vertex && dist > self.tolerance).then_some(i))
            .collect();

        Some(next_vertex)
    }

    /// The inner call to generate a convex hull.
    pub(crate) fn generate_convex_hull(&mut self) -> Result<Vec<TriangleIndices>, ConvexHullError> {
        self.build_initial_tetrahedron()?;
        while !self.indices_to_process.is_empty() {
            // Get the index the furthest away and remove all inner vertices along the way.
            let Some(next_vertex) = self.get_best_vertex_index_and_sweep() else {
                break;
            };

            // Partition the triangles in to be deleted and remaining.
            let (mut remaining, deleted): (Vec<_>, Vec<_>) = take(&mut self.hull_triangles)
                .into_iter()
                .partition(|tri| tri.get_signed_distance(next_vertex) <= 0.0);

            // Extract the outer boundary edges of the elements that get deleted.
            let all_edges = deleted
                .into_iter()
                .flat_map(|tri| tri.edges())
                .collect::<FxHashSet<_>>();
            let boundary_edges = all_edges
                .iter()
                .filter(|edge| !all_edges.contains(&edge.reversed()))
                .collect::<Vec<_>>();

            // Construct the kitting triangles from the boundary to the new vertex.
            remaining.extend(
                boundary_edges
                    .iter()
                    .map(|edge| Triangle::from_edge_and_points(self.vertices, edge, next_vertex)),
            );
            self.hull_triangles = remaining;
        }

        Ok(self
            .hull_triangles
            .iter()
            .map(|tri| tri.get_triple_representation())
            .collect())
    }
}
