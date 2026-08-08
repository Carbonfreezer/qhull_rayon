//! Test for te qhull library.
use proptest::prelude::*;
use qhull_rayon::test_utils::*;
use qhull_rayon::*;

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
