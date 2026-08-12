// ! This part contains the real construction of the convex hull.

use crate::geometry_helper::Triangle;
use crate::{ConvexHullError, TriangleIndices};
use fxhash::FxHashSet;
use glam::Vec3;
use rayon::prelude::{
    IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator,
    IntoParallelRefMutIterator, ParallelIterator,
};
use slotmap::{DefaultKey, DenseSlotMap, Key};
use std::mem::take;

const RELATIVE_TOLERANCE: f32 = 1e-5;

/// The triangle map used relatively often.
type TriangleMap<'a> = DenseSlotMap<DefaultKey, Triangle<'a>>;

/// Computes the numerical tolerance based on the bounding box diagonal
pub(crate) fn compute_tolerance_value(vertices: &[Vec3]) -> f32 {
    let (min, max) = vertices
        .par_iter()
        .fold(
            || (Vec3::INFINITY, Vec3::NEG_INFINITY),
            |(lo, hi), v| (lo.min(*v), hi.max(*v)),
        )
        .reduce(
            || (Vec3::INFINITY, Vec3::NEG_INFINITY),
            |(min_a, max_a), (min_b, max_b)| (min_a.min(min_b), max_a.max(max_b)),
        );
    (max - min).length() * RELATIVE_TOLERANCE
}

/// This is a vertex we regard for processing.
struct RegardedVertex {
    /// The index into the original vertex array.
    index: usize,
    /// The furthest away triangle if known.
    furthest_away_triangle: DefaultKey,
    /// The distance to the triangle.
    distance_to_furthest_triangle: f32,
}

impl RegardedVertex {
    /// Constructs a new vertex from the index handed over.
    fn new(index: usize) -> Self {
        Self {
            index,
            furthest_away_triangle: DefaultKey::null(),
            distance_to_furthest_triangle: f32::NEG_INFINITY,
        }
    }

    /// Returns the distance to the triangle we are furthest away from, eventually updates the internal structure.
    fn get_furthest_distance(&mut self, triangle_map: &TriangleMap) -> f32 {
        if triangle_map.contains_key(self.furthest_away_triangle) {
            self.distance_to_furthest_triangle
        } else {
            // The triangle is not in the map (anymore), so we have to search for it.
            let (new_dist, new_triangle) = triangle_map.iter().fold(
                (f32::NEG_INFINITY, DefaultKey::null()),
                |(best_dist, best_key), (new_key, new_triangle)| {
                    let dist = new_triangle.get_signed_distance(self.index);
                    if dist > best_dist {
                        (dist, new_key)
                    } else {
                        (best_dist, best_key)
                    }
                },
            );

            // Update the value.
            self.distance_to_furthest_triangle = new_dist;
            self.furthest_away_triangle = new_triangle;

            new_dist
        }
    }
}

pub(crate) struct HullConstructor<'a> {
    /// The vertices we still need to process.
    vertices: &'a [Vec3],
    /// The hull as it currently stands in the construction process.
    hull_triangles: TriangleMap<'a>,
    /// The list with the indices into the vertices that still have to be processed.
    vertices_to_process: Vec<RegardedVertex>,
    /// The barrier we use for tolerance checks.
    tolerance: f32,
}

impl<'a> HullConstructor<'a> {
    /// Vertices are handed over.
    pub(crate) fn new(vertices: &'a [Vec3]) -> Self {
        let vertices_to_process = (0..vertices.len())
            .map(|i| RegardedVertex::new(i))
            .collect::<Vec<_>>();
        Self {
            vertices,
            vertices_to_process,
            hull_triangles: TriangleMap::new(),
            tolerance: 1.0,
        }
    }

    /// Gets the best index pair as an index into the inner vector and the vertex position.
    fn get_best_index(&self, probe_function: impl Fn(usize) -> f32 + Sync) -> (usize, usize) {
        let (inner_index, result, _) = self
            .vertices_to_process
            .par_iter()
            .enumerate()
            .map(|(field_ind, regarded_vertex)| {
                (
                    field_ind,
                    regarded_vertex,
                    probe_function(regarded_vertex.index),
                )
            })
            .max_by(|(_, _, val_a), (_, _, val_b)| val_a.total_cmp(val_b))
            .expect("There should be at least one vertex left");
        (inner_index, result.index)
    }
    /// Applies the probe_function to all vertices and finds the element with the highest value,
    /// the index of that vertex is returned and eliminated from the processing list.
    fn get_best_index_and_remove(&mut self, probe_function: impl Fn(usize) -> f32 + Sync) -> usize {
        let (inner_index, result) = self.get_best_index(probe_function);
        self.vertices_to_process.swap_remove(inner_index);
        result
    }

    /// Cleans out all inner vertices by the contained data.
    fn clean_inner(&mut self) {
        self.vertices_to_process = take(&mut self.vertices_to_process)
            .into_par_iter()
            .filter_map(|mut vert| {
                let dist = vert.get_furthest_distance(&self.hull_triangles);
                (dist > self.tolerance).then_some(vert)
            })
            .collect();
    }

    /// Builds an initial tetrahedron from the inner vertices.
    fn build_initial_tetrahedron(&mut self) -> Result<(), ConvexHullError> {
        debug_assert!(
            self.hull_triangles.is_empty(),
            "The initial tetrahedron must be empty."
        );

        // Field gets a linear scale.
        self.tolerance = compute_tolerance_value(self.vertices);

        // The first vertex is the one furthest out in x
        let i0 = self.get_best_index_and_remove(|i| self.vertices[i].x);
        let pos0 = self.vertices[i0];
        // The second vertex is the one furthest away from first.
        let i1 = self.get_best_index_and_remove(|i| (self.vertices[i] - pos0).length_squared());
        let pos1 = self.vertices[i1];
        let a = pos1 - pos0;
        let dir_a = a.normalize_or_zero();
        // The third vertex is the one furthest away from the edge.
        let i2 = self.get_best_index_and_remove(|i| {
            let point_on_line = pos0 + (self.vertices[i] - pos0).dot(dir_a) * dir_a;
            (self.vertices[i] - point_on_line).length_squared()
        });
        let pos2 = self.vertices[i2];
        let b = pos2 - pos0;
        // Gram Schmidt step.
        let dir_b = (b - b.dot(dir_a) * dir_a).normalize_or_zero();
        // The fourth and last point is the point the furthest away from the constructed plane.
        let i3 = self.get_best_index_and_remove(|i| {
            let delta = self.vertices[i] - pos0;
            let point_on_plane = pos0 + delta.dot(dir_a) * dir_a + delta.dot(dir_b) * dir_b;
            (self.vertices[i] - point_on_plane).length_squared()
        });
        let pos3 = self.vertices[i3];
        let c = pos3 - pos0;

        // Let us check for degeneration
        let volume_scale = a.length() * b.length() * c.length();
        if volume_scale <= 0.0 || (a.cross(b).dot(c).abs() / volume_scale) < 1e-6 {
            return Err(ConvexHullError::DegenerateInput);
        }

        let mut triangles = vec![
            Triangle::new(self.vertices, [i0, i1, i2]),
            Triangle::new(self.vertices, [i1, i0, i3]),
            Triangle::new(self.vertices, [i2, i1, i3]),
            Triangle::new(self.vertices, [i3, i0, i2]),
        ];

        if triangles[0].get_signed_distance(i3) > 0.0 {
            triangles = triangles
                .into_iter()
                .map(|tri| tri.get_flipped_version())
                .collect();
        }

        for tri in triangles {
            self.hull_triangles.insert(tri);
        }
        Ok(())
    }

    /// Analyzes the existing vertices and finds the index that is furthest away from the existing hull
    /// and removes all vertices that are inside the hull.
    fn get_best_vertex_index_and_sweep(&mut self) -> usize {
        let (best_index, highest_value) = self
            .vertices_to_process
            .par_iter_mut()
            .enumerate()
            .fold(
                || (0usize, f32::NEG_INFINITY),
                |(best_index, highest_value), (index, vertex)| {
                    let dist = vertex.get_furthest_distance(&self.hull_triangles);
                    if dist > highest_value {
                        (index, dist)
                    } else {
                        (best_index, highest_value)
                    }
                },
            )
            .reduce(
                || (0usize, f32::NEG_INFINITY),
                |(best_index, highest_value), (new_index, new_value)| {
                    if new_value > highest_value {
                        (new_index, new_value)
                    } else {
                        (best_index, highest_value)
                    }
                },
            );

        debug_assert!(
            highest_value > 0.0,
            "We should not get here with inner vertices."
        );

        let result = self.vertices_to_process[best_index].index;
        self.vertices_to_process.swap_remove(best_index);

        result
    }

    /// The inner call to generate a convex hull.
    pub(crate) fn generate_convex_hull(&mut self) -> Result<Vec<TriangleIndices>, ConvexHullError> {
        self.build_initial_tetrahedron()?;
        self.clean_inner();
        while !self.vertices_to_process.is_empty() {
            // Get the index the furthest away and remove all inner vertices along the way.
            let next_vertex = self.get_best_vertex_index_and_sweep();

            // Get the triangles to be deleted.
            let deleted: Vec<_> = self
                .hull_triangles
                .iter()
                .filter_map(|(key, tri)| {
                    (tri.get_signed_distance(next_vertex) > 0.0).then(|| (key, tri.clone()))
                })
                .collect();

            // Extract the outer boundary edges of the elements that get deleted.
            let all_edges = deleted
                .iter()
                .flat_map(|(_, tri)| tri.edges())
                .collect::<FxHashSet<_>>();
            let boundary_edges = all_edges
                .iter()
                .filter(|edge| !all_edges.contains(&edge.reversed()))
                .collect::<Vec<_>>();

            // Now we need to update the hull.
            // First delete the old triangles.
            for (index, _) in deleted {
                self.hull_triangles.remove(index);
            }
            // Add the new ones.
            for edge in boundary_edges {
                self.hull_triangles.insert(Triangle::from_edge_and_points(
                    self.vertices,
                    edge,
                    next_vertex,
                ));
            }

            // Clean out.
            self.clean_inner();
        }
        Ok(self
            .hull_triangles
            .values()
            .map(|tri| tri.get_triple_representation())
            .collect())
    }
}
