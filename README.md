# Q-Hull in 3D

![License](https://img.shields.io/badge/license-MIT-green)
![Built with](https://img.shields.io/badge/built%20with-Rust%20-orange)
## Overview

This is an optimized version of the [q-hull](https://en.wikipedia.org/wiki/Quickhull) algorithm in 3D. 
The name q-hull rayon has historical reasons, because the first implementation was made with rayon. After a 
substantial optimization, the parallelized version was slower than the non-parallelized one.
For vertices, it uses the [glam library](https://docs.rs/glam/latest/glam/) version `0.33.3` and works on `f32`. Alternative functions that operate on `[f32;3]` are provided. 

For many inner vertices in the convex hull, the time complexity
of this algorithm is approximately linear, and for the worst case, where all vertices handed over are part of the convex hull,
it is around n^1.7.
## Basic usage example

The direct way to use the algorithm is by:

```rust
use qhull_rayon::{generate_convex_hull, Vec3};
let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
let result = generate_convex_hull(&positions)?;
assert_eq!(result.len(), 4, "We should get the four triangles of the outer tetrahedron");
```

The index structure for the triangles returned refers to the original vertex positions. If you want to have a separate mesh structure with only the vertices left,
that are part of the convex hull, you can use the `Mesh` structure, which is part of the library. Its main function is to filter out unused vertices. 
The contents can also be saved as a Wavefront OBJ file.

```rust
use qhull_rayon::{generate_convex_hull, Vec3};
use qhull_rayon::mesh::Mesh;
let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
let indices = generate_convex_hull(&positions)?;
let new_mesh = Mesh::new(&positions, &indices)?;
new_mesh.save_as_obj_file(Path::new("Processed.obj"))?;
assert_eq!(new_mesh.vertices().len(), 4, "We should have exactly four vertices left.");
```

If you do not want to use the `Vec3` that comes with the specific version of glam `0.33.3`, alternative functions that operate on `[f32;3]` are provided.

The entry in `cargo.toml` can be done in two ways: the standard way. 

```cargo.toml
[dependencies]
qhull_rayon = "0.2.0"
```

or if you want to have access to some internal generation and test functions
```cargo.toml
[dependencies]
qhull_rayon = {version = "0.2.0", features = ["test-utils"]}
```


## Additional material

The git hub repo comes with an example that can be started by:
> `cargo run --release --example obj_converter -- test_data/in_file.obj test_data/out_file.obj`

The example takes the exported Suzanne monkey from Blender and computes the convex hull. A rendering of the result can
be seen here: ![monkey with convex hull](https://raw.githubusercontent.com/Carbonfreezer/qhull_rayon/main/test_data/Monkey.png).

It also comes with a benchmark system
> `cargo bench`

The bench system uses [Criterion](https://crates.io/crates/criterion). After execution, the result can be found as an 
HTML report in `target/criterion/report/index.html`.
Benches are executed using three different data sets. The first one is a sphere where points are evenly
distributed within the sphere. Measurements are done here via Criterion over several numbers of vertices. 
The time used on a computer with an AMD Ryzen 9 9950X3D2 was

| vertices | time (ms) |
|----------|-----------|
| 50       | 0.009     |
| 500      | 0.012     |
| 1_000    | 0.114     |
| 10_000   | 1.076     |
| 40_000   | 3.76      |
| 80_000   | 7.00      |
| 100_000  | 9.05      |


The computation distributions are shown here:

![sphere full line](https://raw.githubusercontent.com/Carbonfreezer/qhull_rayon/main/performance_shots/sphere_full_lines.png).

The second one is a box that naturally has a much simpler convex hull, with the option to cut in by culling a lot more
aggressively. In this experiment, the same number of vertices was used. The timing table is:

| vertices | time (ms) |
|----------|-----------|
| 50       | 0.002     |
| 500      | 0.011     |
| 1_000    | 0.022     |
| 10_000   | 0.204     |
| 40_000   | 1.036     |
| 80_000   | 2.31      |
| 100_000  | 2.76      |

The corresponding line plot is displayed here:

![box lines](https://raw.githubusercontent.com/Carbonfreezer/qhull_rayon/main/performance_shots/box_lines.png).

The effective time complexity in both cases is approximately linear in the number of vertices.

The Achilles' heel of this algorithm is a point cloud in which all points lie on the convex hull. To demonstrate
this effect, we use a data set that contains vertices that all reside on a sphere surface. Here we restrict ourselves
to smaller vertex counts. The timing result is shown in this table:

| vertices | time (ms) |
|----------|-----------|
| 100      | 0.063     |
| 500      | 0.74      |
| 1_000    | 2.25      |
| 1_200    | 3.02      |
| 1_300    | 3.42      |
| 1_300    | 3.9       |
| 1_500    | 4.3       |
| 2_000    | 7.01      |

The time complexity, depending on the number of vertices, is approximately n^1.7.

This one also comes with a plot:

![hollow sphere lines](https://raw.githubusercontent.com/Carbonfreezer/qhull_rayon/main/performance_shots/sphere_hollow_lines.png)

The efficiency of the algorithm hinges clearly on its culling ability.

The project also contains an extensive test suite making use of property tests
> `cargo test`

The test suite makes use of property testing by the [proptest](https://crates.io/crates/proptest), 
which scans the valid input range. On failure, it tries to find the minimal counterexample.

Documentation can be generated with
> `cargo doc --open`

If you want to include the documentation for the test utilities, add all-features:
> `cargo doc --open --all-features`
