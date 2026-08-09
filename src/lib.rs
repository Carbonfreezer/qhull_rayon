//! This library contains an implementation of the q-hull algorithm using glam for vector algebra and
//! rayon for parallelization. Uses an adaptation of [q-hull](https://en.wikipedia.org/wiki/Quickhull) for three dimensions.
//! For vector representation, we use the [glam library](https://docs.rs/glam/latest/glam/). This library shines
//! performance-wise, when you can expect many inner vertices that do not belong to the hull, which
//! is the standard use case for collision geometry.

#![warn(missing_docs)]

mod geometry_helper;
mod hull_construction;
// lib.rs
#[cfg(feature = "test-utils")]
pub mod test_utils;

use crate::hull_construction::HullConstructor;
pub use glam::Vec3;
use rayon::prelude::{IndexedParallelIterator, IntoParallelRefIterator};

/// The Triangle with its three indices; these are the indices that refer to the vector that has been
/// handed over. The triangles are generated in CCW order seen from the outside.
#[derive(Clone, Debug, Eq, Hash, PartialEq, PartialOrd, Ord)]
pub struct TriangleIndices(pub usize, pub usize, pub usize);

impl TriangleIndices {
    /// Gets the inner indices as an array. Useful for further processing with iterators.
    pub fn get_array(&self) -> [usize; 3] {
        [self.0, self.1, self.2]
    }
}

/// All errors that can happen in the handover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConvexHullError {
    /// Fewer than four vertices were supplied.
    TooFewVertices { 
        /// The number of vertices found.
        count: usize 
    },
    /// All vertices lie in a common plane (or on a line/point).
    DegenerateInput,
    /// A vertex contains a non-finite coordinate.
    NonFiniteVertex { 
        /// The index of the vertex that was not finite.
        index: usize 
    },
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

/// Generates the convex hull from a list of positions handed over. From the positions, triangles are returned
/// with the indices given in counterclockwise order seen from the outside.
/// 
/// This function returns an error in the following cases
/// 1. There are less than 4 vertices handed over.
/// 2. One of the vertices as an infinite or a NaN component.
/// 3. The vertices handed over are coplanar, colinear or the same. 
///
/// # Example
/// ```
/// use glam::Vec3;
/// use qhull_rayon::generate_convex_hull;
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
