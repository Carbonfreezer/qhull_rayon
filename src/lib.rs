//! This library contains an implementation of the q hull algorithm using glam for vector algebra and
//! rayon for parallelization. Contains the qhull from wikipedia:  <https://en.wikipedia.org/wiki/Quickhull>

mod geometry_helper;
mod hull_construction;

use crate::geometry_helper::Triangle;
use crate::hull_construction::{HullConstructor, RELATIVE_TOLERANCE};
pub use glam::Vec3;
use rayon::prelude::{IndexedParallelIterator, IntoParallelRefIterator};

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

/// Generates the convex hull from a list of positions handed over. From the positions triangles are returned
/// with the indices given in counterclockwise order seen from the outside.
///
/// # Example
/// ```
/// use glam::Vec3;
/// use rust_qhull::generate_convex_hull;
/// let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
/// let result = generate_convex_hull(&positions).expect("Input should be fine");
/// assert_eq!(result.len(), 4, "We should get the four triangles of the outer tetrahedron");
/// ```
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
    constructor.generate_convex_hull()
}

/// Runs a consistency check on the vertices and the computed convex hull. It makes sure,
/// * there is no vertex outside computed convex hull.
/// * that for every edge there is an edge in opposite direction.
/// * no hull vertex is inside the hull.
/// Mainly used for test / debug purposes.
///
/// # Example
/// ```
/// use glam::Vec3;
/// use rust_qhull::{consistency_check, generate_convex_hull};
/// let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
/// let result = generate_convex_hull(&positions).expect("Input should be fine");
/// assert!(consistency_check(&positions, &result), "Something went wrong");
/// ```
pub fn consistency_check(vertices: &[Vec3], convex_hull: &[TriangleIndices]) -> bool {
    // For epsilon tests we need the bounding box:
    let (min, max) = vertices
        .iter()
        .fold((Vec3::INFINITY, Vec3::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(*v), hi.max(*v))
        });
    let tolerance = (max - min).length() * RELATIVE_TOLERANCE;

    let tri_list = convex_hull
        .iter()
        .map(|tri| Triangle::new(vertices, [tri.0, tri.1, tri.2]))
        .collect::<Vec<_>>();
    let all_edges = tri_list
        .iter()
        .flat_map(|tri| tri.edges())
        .collect::<Vec<_>>();

    // Convexity check.
    for vert in 0..vertices.len() {
        for tri in &tri_list {
            if tri.get_signed_distance(vert) > tolerance {
                return false;
            }
        }
    }

    // Now we go over the BB vertices. They appear several times, we do not care about filtering multiples.
    let failed_inner = convex_hull
        .iter()
        .flat_map(|tri| [tri.0, tri.1, tri.2])
        .any(|index| {
            let mut failure = true;
            for tri in &tri_list {
                failure = failure && tri.get_signed_distance(index) < -tolerance;
            }
            failure
        });
    if failed_inner {
        return false;
    }

    // Now we check for closedness every edge must exist exactly once in itself and the reverse direction.
    for edge in &all_edges {
        let (forward, backward) =
            all_edges
                .iter()
                .fold((0, 0), |(forward, backward), candidate| {
                    (
                        forward + if candidate == edge { 1 } else { 0 },
                        backward + if *candidate == edge.reversed() { 1 } else { 0 },
                    )
                });
        if forward != 1 || backward != 1 {
            return false;
        }
    }

    true
}
