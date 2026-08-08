

use criterion::{ criterion_group, criterion_main, BenchmarkId, Criterion};
use qhull_rayon::*;
use qhull_rayon::test_utils::*;

fn bench_hull(c: &mut Criterion) {
    let mut group = c.benchmark_group("hull");

    for n in [50usize, 1000, 10_000, 40_000, 100_000] {
        group.bench_with_input(BenchmarkId::new("sphere", n), &n, |b, &n| {
            let vertices = generate_sphere(1.0, n, 42);
            b.iter(|| {let _ = generate_convex_hull(&vertices);});
        });

        group.bench_with_input(BenchmarkId::new("cube", n), &n, |b, &n| {
            let vertices = generate_cube(1.0, n - 27, 42);
            b.iter(|| {let _ = generate_convex_hull(&vertices);});
        });
    }
    group.finish();
}


criterion_group!(benches, bench_hull);
criterion_main!(benches);
