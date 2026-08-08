//! This library contains an implementation of the q hull algorithm using glam for vector algebra and
//! rayon for parallelization. Contains the qhull from wikipedia:  <https://en.wikipedia.org/wiki/Quickhull>

mod geometry_helper;
mod hull_construction;

pub use glam::Vec3;
use rayon::prelude::{IndexedParallelIterator, IntoParallelRefIterator};
use crate::hull_construction::HullConstructor;

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
