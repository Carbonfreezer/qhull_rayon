//! Test for the qhull library.

use fxhash::FxHashSet;
use proptest::prelude::*;
use qhull_rayon::test_utils::*;
use qhull_rayon::*;
use qhull_rayon::mesh::Mesh;


/// Checks if there are some vertices left over in the containing vertex array that are not indexed.
fn test_internal_consistency(mesh : &Mesh) -> Result<(), usize> {
    let all_indices = mesh.triangles.iter().flat_map(TriangleIndices::to_array).collect::<FxHashSet<_>>();
    for i in 0..mesh.vertices.len() {
        if !all_indices.contains(&i) {return Err(i)}
    }
    Ok(())
}

// Tests
proptest! {
    #[test]
    fn box_test(radius in 0.0001f32..100_000.0f32, addtional_vertices in 0usize..20_000, seed in any::<u64>()) {
       let vertices = generate_cube(radius, addtional_vertices, seed);
       let hull = generate_convex_hull(&vertices)
            .map_err(|e| TestCaseError::fail(format!("hull failed: {e}")))?;
        prop_assert_eq!(consistency_check(&vertices, &hull), Ok(()));
    }
}

proptest! {
    #[test]
    fn  sphere_test(radius in 0.0001f32..100_000.0f32, vert_num in 4usize..4_000, seed in any::<u64>()) {
       let vertices = generate_sphere(radius, vert_num, seed);
       match generate_convex_hull(&vertices) {
           Ok(hull) => prop_assert_eq!(consistency_check(&vertices, &hull), Ok(())),
           Err(ConvexHullError::DegenerateInput) => {} // Can happen with fewer poins
           Err(e) => return Err(TestCaseError::fail(format!("hull failed: {e}"))),
       }
    }
}

proptest! {
    #[test]
    fn  sphere_hull_test(radius in 0.0001f32..100_000.0f32, vert_num in 4usize..400, seed in any::<u64>()) {
       let vertices = generate_sphere_hull(radius, vert_num, seed);
       match generate_convex_hull(&vertices) {
           Ok(hull) => prop_assert_eq!(consistency_check(&vertices, &hull), Ok(())),
           Err(ConvexHullError::DegenerateInput) => {} // Can happen with fewer poins
           Err(e) => return Err(TestCaseError::fail(format!("hull failed: {e}"))),
       }
    }
}

proptest! {
    #[test]
    fn  mesh_test(radius in 0.0001f32..100_000.0f32, vert_num in 4usize..4_000, seed in any::<u64>()) {
       let vertices = generate_sphere(radius, vert_num, seed);
       let hull = match generate_convex_hull(&vertices) {
           Ok(hull) => { prop_assert_eq!(consistency_check(&vertices, &hull), Ok(())); hull} ,
           Err(ConvexHullError::DegenerateInput) => {return Ok(())} // Can happen with fewer poins
           Err(e) => return Err(TestCaseError::fail(format!("hull failed: {e}"))),
       };

       // Can we create a reduced mesh out if it?
       let mesh = match Mesh::new(&vertices, &hull) {
            Ok(mesh) => mesh,
            Err(e) => return Err(TestCaseError::fail(format!("mesh construction failed: {e}")))
        };

        // Are there any vertices left over.
        if let Err(e) = test_internal_consistency(&mesh) {
            return Err(TestCaseError::fail(format!("Vertex with index left over: {e}")))
        }

        // Is it really a convex mesh?
        prop_assert_eq!(consistency_check(&mesh.vertices, &mesh.triangles), Ok(()));
    }
}

#[test]
fn degenerate_convex_hull_test() {
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
    ];
    let hull = generate_convex_hull(&vertices);
    assert_eq!(
        hull,
        Err(ConvexHullError::DegenerateInput),
        "Degenerate convex hull expected"
    );
}

#[test]
fn too_few_vertices_test() {
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.1, 0.0),
    ];
    let hull = generate_convex_hull(&vertices);
    assert_eq!(
        hull,
        Err(ConvexHullError::TooFewVertices { count: 3 }),
        "Too few vertices with 3 expected."
    );
}

#[test]
fn broken_vertex_test() {
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.1, 0.0),
        Vec3::new(0.0, 0.0, f32::NAN),
    ];
    let hull = generate_convex_hull(&vertices);
    assert_eq!(
        hull,
        Err(ConvexHullError::NonFiniteVertex { index: 3 }),
        "Broken vertex at index 3 expected."
    );
}
