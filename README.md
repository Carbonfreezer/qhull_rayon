# Q-Hull Rayon

## Overview

This is a rayon parallelized version of the [q-hull](https://en.wikipedia.org/wiki/Quickhull) algorithm in 3D. For
vertices it uses the [glam library]((https://docs.rs/glam/latest/glam/)) and works on `f32`. This algorithm is optimized
for culling so it works best with geometries that have a lot of inner vertices like you may get for collision proxies.

## Basic usage example

The direct way to use the algorithm is by:

```rust
use glam::Vec3;
use qhull_rayon::generate_convex_hull;
let positions = [Vec3{x:0.0, y:0.0, z:0.0}, Vec3{x:1.0, y:0.0, z:0.0}, Vec3{x:0.0, y:1.0, z:0.0}, Vec3{x:0.0, y:0.0, z:1.0}, Vec3{x:0.1, y:0.1, z:0.1}];
let result = generate_convex_hull( & positions).expect("Input should be fine");
assert_eq!(result.len(), 4, "We should get the four triangles of the outer tetrahedron");
```

## Additional material

The bundle comes with an example, that can be started by:
> `cargo run --release --example obj_converter -- test_data/in_file.obj test_data/out_file.obj`

The example takes the exported Suzanne monkey from Blender and computes the convex hull. A rendering of the result can
be seen here: ![monkey with convex hull](test_data/Monkey.png).

It also comes with a benchmark system
> `cargo bench`

Benches get executed with three different types of data set. The first one is a sphere where points are evenly
distributed within the sphere. Measurements are done here via Criterion on 50, 1000, 10_000, 40_000 and 100_000 data
points. The computations distributions are shown here:

![sphere full violin](performance_shots/sphere_full_violin.svg)
and as line here:

![sphere full line](performance_shots/sphere_full_lines.svg).

The second one is a box that naturally has a much simpler convex hull with the option to cut in with culling a lot more
aggressively. For the same vertex amounts as in the sphere the plots are displayed here:

![box violin](performance_shots/box_violin.svg)
and here:

![box lines](performance_shots/box_lines.svg).

The achilles heel of this algorithms is a point cloud where all points belong to the convex hull. In this case there are
faster implementations that track the correspondence between faces and vertices and rely less on culling. To demonstrate
this effect we use a data set, that contains vertices, that all reside on a sphere surface. Here we restrict ourselves
to vertex amounts of 100, 1_000, 1_500 and 2_000 vertices. This one also comes with a violin plot:

![hollow sphere violin](performance_shots/sphere_hollow_violin.svg) and a line plot:

![hollow sphere lines](performance_shots/sphere_hollow_lines.svg)

The efficiency of the algorithm hinges clearly on its culling ability.

The project also contains an extensive test suite making use of property tests
> `cargo test`

A documentation can be generated with
> `cargo doc --open`