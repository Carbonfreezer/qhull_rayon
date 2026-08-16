//! This library contains an implementation of the q-hull algorithm using [glam](https://docs.rs/glam/latest/glam/)
//! for vector algebra. If you do not use glam or an incompatible version to 0.33.3, raw functions are implemented that operate on `[f32;3]` representations.
//! The crate is an adaptation of [q-hull](https://en.wikipedia.org/wiki/Quickhull) for three dimensions.
//! This library shines performance-wise when you can expect many inner vertices that do not belong to the hull, which
//! is the standard use case for collision geometry. The crate name *qhull_rayon* has a historical reason. An earlier
//! version of this library was parallelized with rayon. After a substantial optimization, this turned out to be counterproductive.  

#![warn(missing_docs)]

mod geometry_helper;
mod hull_construction;
// lib.rs
pub mod mesh;
#[cfg(feature = "test-utils")]
pub mod test_utils;

use crate::hull_construction::HullConstructor;

/// The Vec3 from glam v0.33.3:  
pub use glam::Vec3;

/// The Triangle with its three indices; these are the indices that refer to the vector that has been
/// handed over. The triangles are generated in CCW order seen from the outside.
#[derive(Clone, Debug, Eq, Hash, PartialEq, PartialOrd, Ord)]
pub struct TriangleIndices(pub usize, pub usize, pub usize);

impl TriangleIndices {
    /// Gets the inner indices as an array. Useful for further processing with iterators.
    ///
    /// # Example
    /// ```
    /// use qhull_rayon::TriangleIndices;
    /// let a = TriangleIndices(0,1,2);
    /// let b = a.to_array();
    /// assert_eq!(b, [0,1,2]);
    /// ```
    pub fn to_array(&self) -> [usize; 3] {
        [self.0, self.1, self.2]
    }

    /// Computes an index structure from an array.
    ///
    /// # Example
    /// ```
    /// use qhull_rayon::TriangleIndices;
    /// let a = TriangleIndices::from_array([1,2,3]);
    /// assert_eq!(a, TriangleIndices(1,2,3));
    pub fn from_array(indices: [usize; 3]) -> Self {
        TriangleIndices(indices[0], indices[1], indices[2])
    }
}

/// All errors that can happen in the handover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConvexHullError {
    /// Fewer than four vertices were supplied.
    TooFewVertices {
        /// The number of vertices found.
        count: usize,
    },
    /// All vertices lie in a common plane (or on a line/point).
    DegenerateInput,
    /// A vertex contains a non-finite coordinate.
    NonFiniteVertex {
        /// The index of the vertex that was not finite.
        index: usize,
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
///
/// # Error
/// This function returns a [convex hull error](ConvexHullError) in the following cases
/// 1. There are fewer than 4 vertices handed over.
/// 2. One of the vertices has an infinite or NaN component.
/// 3. The vertices handed over are coplanar, collinear, or the same.
///
/// # Example
/// ```
/// # use std::error::Error;
/// #
/// # fn main() -> Result<(), Box<dyn Error>> {
/// use qhull_rayon::{generate_convex_hull, Vec3};
/// let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
/// let result = generate_convex_hull(&positions)?;
/// assert_eq!(result.len(), 4, "We should get the four triangles of the outer tetrahedron");
/// #
/// #     Ok(())
/// # }
/// ```
pub fn generate_convex_hull(vertices: &[Vec3]) -> Result<Vec<TriangleIndices>, ConvexHullError> {
    let vertices: &[Vec3] = bytemuck::cast_slice(vertices);
    if vertices.len() < 4 {
        return Err(ConvexHullError::TooFewVertices {
            count: vertices.len(),
        });
    }
    if let Some((index, _)) = vertices.iter().enumerate().find(|(_, v)| !v.is_finite()) {
        return Err(ConvexHullError::NonFiniteVertex { index });
    }

    let mut constructor = HullConstructor::new(vertices);
    constructor.generate_convex_hull()
}

/// Structurally the same as [generate_convex_hull], useful if you do not use glam internally or a different version than 0.33.3
///
/// # Error
/// This function returns a [convex hull error](ConvexHullError) in the following cases
/// 1. There are fewer than 4 vertices handed over.
/// 2. One of the vertices has an infinite or NaN component.
/// 3. The vertices handed over are coplanar, collinear, or the same.
///
/// # Example
/// ```
/// # use std::error::Error;
/// #
/// # fn main() -> Result<(), Box<dyn Error>> {
/// use qhull_rayon::{generate_convex_hull_from_raw};
/// let positions = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.1, 0.1, 0.1]];
/// let result = generate_convex_hull_from_raw(&positions)?;
/// assert_eq!(result.len(), 4, "We should get the four triangles of the outer tetrahedron");
/// #
/// #     Ok(())
/// # }
/// ```
pub fn generate_convex_hull_from_raw(
    vertices: &[[f32; 3]],
) -> Result<Vec<TriangleIndices>, ConvexHullError> {
    let vertices: &[Vec3] = bytemuck::cast_slice(vertices);
    generate_convex_hull(vertices)
}
