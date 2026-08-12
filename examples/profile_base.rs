//! This is a simple example to run flamegraph on it.

use qhull_rayon::generate_convex_hull;
use qhull_rayon::test_utils::generate_sphere_hull;

fn main() {
    let vertices = generate_sphere_hull(1.0, 2_000, 42);
    for _ in 0..20 {
        let _hull = generate_convex_hull(&vertices);
    }
}
