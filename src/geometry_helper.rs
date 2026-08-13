//! Contains some helpers for the convex hull computation.
//! This is mainly information about triangles and edges.

use crate::TriangleIndices;
use glam::Vec3;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub(crate) struct Edge {
    pub(crate) start: usize,
    pub(crate) end: usize,
}

impl Edge {
    pub(crate) fn reversed(&self) -> Self {
        Self {
            start: self.end,
            end: self.start,
        }
    }
}

/// The triangle representation with all cached values needed for q-hull.
#[derive(Clone, Debug)]
pub(crate) struct Triangle<'a> {
    /// The base vertices we are based on.
    base_vertices: &'a [Vec3],
    /// One point in the triangle plane.
    base_point: Vec3,
    /// The normal of the triangle.
    normal: Vec3,
    /// Used indizes
    used_indices: [usize; 3],
    /// The edges we have.
    used_edges: [Edge; 3],
    /// The vertices we take control over.
    assigned_vertices: Vec<usize>,
    /// The furthest-away vertex we have, if we have one.
    furthest_vertex_and_dist: Option<(usize, f32)>,
}

impl<'a> Triangle<'a> {
    /// Asks for the vertex index that is furthest away for this triangle, if it exists.
    pub fn furthest_vertex_and_dist(&self) -> Option<(usize, f32)> {
        self.furthest_vertex_and_dist
    }

    /// Asks for the vertices we are responsible for.
    pub fn regarded_vertices(&self) -> impl Iterator<Item = &usize> + '_ {
        self.assigned_vertices.iter()
    }

    /// Creates a new triangle from three indices; the indices must be given in CCW order.
    pub(crate) fn new(base_vertices: &'a [Vec3], indices: [usize; 3]) -> Self {
        let base_point = base_vertices[indices[0]];
        Self {
            base_vertices,
            base_point,
            normal: (base_vertices[indices[1]] - base_point)
                .cross(base_vertices[indices[2]] - base_point)
                .normalize_or_zero(),
            used_indices: indices,
            used_edges: [
                Edge {
                    start: indices[0],
                    end: indices[1],
                },
                Edge {
                    start: indices[1],
                    end: indices[2],
                },
                Edge {
                    start: indices[2],
                    end: indices[0],
                },
            ],
            assigned_vertices: Vec::new(),
            furthest_vertex_and_dist: None,
        }
    }

    /// Creates a new triangle from an edge and a point given over.
    pub(crate) fn from_edge_and_points(
        base_vertices: &'a [Vec3],
        edge: &Edge,
        new_point: usize,
    ) -> Self {
        Self::new(base_vertices, [edge.start, edge.end, new_point])
    }

    /// Gets a flipped version of ourselves.
    pub(crate) fn get_flipped_version(&self) -> Triangle<'a> {
        Self::new(
            self.base_vertices,
            [
                self.used_indices[0],
                self.used_indices[2],
                self.used_indices[1],
            ],
        )
    }

    /// Asks for the triangle indices.-
    pub(crate) fn get_triple_representation(&self) -> TriangleIndices {
        TriangleIndices(
            self.used_indices[0],
            self.used_indices[1],
            self.used_indices[2],
        )
    }

    /// Gets the signed distance handed over from another triangle,
    pub(crate) fn get_signed_distance(&self, other_index: usize) -> f32 {
        self.normal
            .dot(self.base_vertices[other_index] - self.base_point)
    }

    /// Assigns the vertex to our responsibility
    pub(crate) fn assign_vertex(&mut self, candidate: usize) {
        self.assigned_vertices.push(candidate);
        let dist = self.get_signed_distance(candidate);
        if let Some((_, old_dist)) = self.furthest_vertex_and_dist {
            if dist > old_dist {
                self.furthest_vertex_and_dist = Some((candidate, dist));
            }
        } else {
            self.furthest_vertex_and_dist = Some((candidate, dist));
        }
    }

    pub(crate) fn edges(&self) -> [Edge; 3] {
        self.used_edges
    }
}
