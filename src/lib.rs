//! This library contains an implementation of the q hull algorithm using glam for vector algebra and
//! rayon for parallelization. Contains the qhull from wikipedia:  <https://en.wikipedia.org/wiki/Quickhull>

mod geometry_helper;

use crate::geometry_helper::Triangle;
use fxhash::FxHashSet;
use glam::Mat3;
pub use glam::Vec3;
use rayon::prelude::{
    IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator, ParallelIterator,
};

/// The Triangle with its three indices, these are the indices that have been handed over in the vector
/// wit the vertices to compute the convex hull from.
pub struct TriangleIndices(pub usize, pub usize, pub usize);

/// All errors that can happen in the handed over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConvexHullError {
    /// Fewer than four vertices were supplied.
    TooFewVertices { count: usize },
    /// All vertices lie in a common plane (or on a line/point).
    DegenerateInput,
    /// A vertex contains a non-finite coordinate.
    NonFiniteVertex { index: usize },
}

impl std::fmt::Display for ConvexHullError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConvexHullError::TooFewVertices { count } => {
                write!(f, "Too few vertices: {}", count)
            }
            ConvexHullError::DegenerateInput => write!(f, "All vertices are coplanar"),
            ConvexHullError::NonFiniteVertex { index } => {
                write!(f, "Non finite vertex: {}", index)
            }
        }
    }
}

impl std::error::Error for ConvexHullError {}

struct HullConstructor<'a> {
    /// The vertices we still need to process.
    vertices: &'a [Vec3],
    /// The hull as it currently stands in the construction process.
    hull_triangles: Vec<Triangle<'a>>,
    /// The list with the indices into the vertices that still have to be processed.
    indices_to_process: Vec<usize>,
}

impl<'a> HullConstructor<'a> {
    /// Vertices are handed over.
    fn new(vertices: &'a [Vec3]) -> Self {
        let indices_to_process = (0..vertices.len()).collect::<Vec<_>>();
        Self {
            vertices,
            indices_to_process,
            hull_triangles: Vec::new(),
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

        let i0 = self.get_best_index_and_remove(|i| self.vertices[i].x);
        let i1 = self.get_best_index_and_remove(|i| self.vertices[i].y);
        let i2 = self.get_best_index_and_remove(|i| self.vertices[i].z);
        let i3 = self.get_best_index_and_remove(|i| -self.vertices[i].z);

        // Now we have to check for degeneration.
        let matrix = Mat3::from_cols(self.vertices[i3] - self.vertices[i0],
                                     self.vertices[i2] - self.vertices[i0],
                                     self.vertices[i1] - self.vertices[i0]);
        if matrix.determinant() < f32::EPSILON {return Err(ConvexHullError::DegenerateInput)}


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
    /// and removes all vertices that are inside the hull.
    fn get_best_vertex_index_and_sweep(&mut self) -> usize {
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
        debug_assert!(
            *highest_value >= 0.0,
            "There should be at least one outer vertex left"
        );
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
            .filter_map(|(&i, dist)| (i != next_vertex && dist > f32::EPSILON).then_some(i))
            .collect();

        next_vertex
    }

    fn generate_convex_hull(&mut self) -> Result<(), ConvexHullError> {
        self.build_initial_tetrahedron()?;
        while !self.indices_to_process.is_empty() {
            // Get the index the furthest away and remove all inner vertices along the way.
            let next_vertex = self.get_best_vertex_index_and_sweep();

            // Partition the triangles in to be deleted and remaining.
            let (mut remaining, deleted): (Vec<_>, Vec<_>) = self
                .hull_triangles
                .par_iter()
                .cloned()
                .partition(|tri| tri.get_signed_distance(next_vertex) <= 0.0);

            // Extract the outer boundary edges of the elements that get deleted.
            let all_edges = deleted
                .into_par_iter()
                .flat_map(|tri| tri.edges())
                .collect::<FxHashSet<_>>();
            let boundary_edges = all_edges
                .par_iter()
                .filter(|edge| !all_edges.contains(&edge.reversed()))
                .collect::<Vec<_>>();

            // Construct the kitting triangles from the boundary to the new vertex.
            remaining.extend(
                boundary_edges
                    .into_iter()
                    .map(|edge| Triangle::from_edge_and_points(self.vertices, edge, next_vertex)),
            );
            self.hull_triangles = remaining;
        }

        Ok(())
    }
}

/// Generates the convex hull from a list of positions handed over. From the positions triangles are returned
/// with the indices given in counterclockwise order seen from the outside.
pub fn generate_convex_hull(vertices: &[Vec3]) -> Result<Vec<TriangleIndices>, ConvexHullError> {
    if vertices.len() < 4 {
        return Err(ConvexHullError::TooFewVertices {
            count: vertices.len(),
        });
    }
    if let Some(index) = vertices.par_iter().position_any(|v| !v.is_finite()) {
        return Err(ConvexHullError::NonFiniteVertex { index });
    }

    let mut constructor = HullConstructor::new(vertices);
    constructor.generate_convex_hull()?;
    Ok(constructor
        .hull_triangles
        .into_iter()
        .map(|tri| tri.get_triple_representation())
        .collect())
}
