//! This module contains a series of methods that are mean for profiling and testing.
//! They fall into the category of generating vertices to generate convex hulls from and for
//! making consistency checks on computed convex hulls.

use crate::TriangleIndices;
use crate::geometry_helper::{Edge, Triangle};
use crate::hull_construction::RELATIVE_TOLERANCE;
use fxhash::{FxHashMap, FxHashSet};
use glam::Vec3;
use itertools::iproduct;
use rand::prelude::StdRng;
use rand::{RngExt, SeedableRng};

/// The consistency check errors found.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum ConsistencyError {
    /// There is a point outside the convex hull.
    PointOutsideConvexHull(usize),
    /// The form is not closed.
    HullNotClosed,
    /// The face vertex relation does not hold.
    EulerRelationError,
}

/// Runs a consistency check on the vertices and the computed convex hull. It makes sure,
/// * there is no vertex outside computed convex hull.
/// * that for every edge there is an edge in opposite direction.
/// * For s simplex hull we must have F = 2V - 4
///
/// Mainly used for test / debug purposes.
///
/// # Example
/// ```
/// use glam::Vec3;
/// use qhull_rayon::{generate_convex_hull};
/// use qhull_rayon::test_utils::consistency_check;
/// let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
/// let result = generate_convex_hull(&positions).expect("Input should be fine");
/// assert_eq!(consistency_check(&positions, &result), Ok(()), "Something went wrong");
/// ```
pub fn consistency_check(
    vertices: &[Vec3],
    convex_hull: &[TriangleIndices],
) -> Result<(), ConsistencyError> {
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
                return Err(ConsistencyError::PointOutsideConvexHull(vert));
            }
        }
    }

    // Now we check for closedness every edge must exist exactly once in itself and the reverse direction.
    let mut counts: FxHashMap<Edge, u32> = FxHashMap::default();
    for e in &all_edges {
        *counts.entry(*e).or_insert(0) += 1;
    }
    let closed = counts
        .iter()
        .all(|(e, &c)| c == 1 && counts.get(&e.reversed()) == Some(&1));
    if !closed {
        return Err(ConsistencyError::HullNotClosed);
    }

    // Now we get the amount of used vertices in our hull.
    let used_vertices = FxHashSet::from_iter(
        tri_list
            .iter()
            .flat_map(|tri| tri.get_triple_representation().get_array()),
    )
    .len();
    if tri_list.len() + 4 != 2 * used_vertices {
        return Err(ConsistencyError::EulerRelationError);
    }

    Ok(())
}

/// Generates an  axis parallel cube with several inner points. Additional vertices are the number
/// of vertices that come on top of 27 vertices that are already on the surface of the cube.
///
/// # Example
/// ```
/// use qhull_rayon::generate_convex_hull;
/// use qhull_rayon::test_utils::{consistency_check, generate_cube};
/// let cube = generate_cube(100.0, 10_000, 42);
/// let hull = generate_convex_hull(&cube).unwrap();
/// let _ = consistency_check(&cube, &hull).unwrap();
/// ```
pub fn generate_cube(scale: f32, additional_vertices: usize, seed: u64) -> Vec<Vec3> {
    let mut result: Vec<Vec3> = Vec::with_capacity(additional_vertices + 27);
    let mut rng = StdRng::seed_from_u64(seed);

    // We add additional coplanar vertices.
    for (x, y, z) in iproduct!(-1..=1, -1..=1, -1..=1) {
        result.push(Vec3::new(x as f32, y as f32, z as f32) * scale);
    }

    let inner_scale = scale * 0.95;

    for _ in 0..additional_vertices {
        let x = rng.random_range(-inner_scale..=inner_scale);
        let y = rng.random_range(-inner_scale..=inner_scale);
        let z = rng.random_range(-inner_scale..=inner_scale);

        result.push(Vec3::new(x, y, z));
    }

    result
}

/// Generates a random sphere used for testing and profiling.
/// Vertices are located on the inner side of the sphere. 
///
/// # Example
/// ```
/// use qhull_rayon::generate_convex_hull;
/// use qhull_rayon::test_utils::{consistency_check, generate_sphere};
/// let sphere = generate_sphere(100.0, 10_000, 42);
/// let hull = generate_convex_hull(&sphere).unwrap();
/// let _ = consistency_check(&sphere, &hull).unwrap();
/// ```
pub fn generate_sphere(scale: f32, vertices: usize, seed: u64) -> Vec<Vec3> {
    let mut rng = StdRng::seed_from_u64(seed);
    let mut result: Vec<Vec3> = Vec::with_capacity(vertices);

    for _ in 0..vertices {
        let x = rng.random_range(-1.0..=1.0);
        let y = rng.random_range(-1.0..=1.0);
        let z = rng.random_range(-1.0..=1.0);

        let direction = Vec3::new(x, y, z).normalize_or_zero();
        let radius = rng.random_range(0.0f32..=1.0).cbrt() * scale;
        result.push(direction * radius);
    }

    result
}

/// Generates a random hollow sphere used for testing and profiling.
/// Vertices are only located on the sphere surface. 
/// This is the worst case assumption for this algorithm.
///
/// # Example
/// ```
/// use qhull_rayon::generate_convex_hull;
/// use qhull_rayon::test_utils::{consistency_check, generate_sphere_hull};
/// let sphere = generate_sphere_hull(100.0, 1_000, 42);
/// let hull = generate_convex_hull(&sphere).unwrap();
/// let _ = consistency_check(&sphere, &hull).unwrap();
/// ```
pub fn generate_sphere_hull(scale: f32, vertices: usize, seed: u64) -> Vec<Vec3> {
    let mut rng = StdRng::seed_from_u64(seed);
    let mut result: Vec<Vec3> = Vec::with_capacity(vertices);

    for _ in 0..vertices {
        let x = rng.random_range(-1.0..=1.0);
        let y = rng.random_range(-1.0..=1.0);
        let z = rng.random_range(-1.0..=1.0);

        let direction = Vec3::new(x, y, z).normalize_or_zero();
        result.push(direction * scale);
    }

    result
}
