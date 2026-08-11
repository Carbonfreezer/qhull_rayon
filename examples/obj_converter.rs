//! This is a test sample that converts two OBJ files. It reads one, generates the convex hull of the geometry, and saves it as a new file.

use glam::Vec3;
use qhull_rayon::generate_convex_hull;
use qhull_rayon::mesh::Mesh;
use std::path::Path;
use tobj;
use tobj::LoadError;

/// Uses the tobj library to load the obj file, goes over all models in the obj file and collects the data.
fn load_vertices_from_file(path: &Path) -> Result<Vec<Vec3>, LoadError> {
    let data = tobj::load_obj(path, &tobj::OFFLINE_RENDERING_LOAD_OPTIONS);
    let mut result = Vec::new();

    let (models, _) = data?;
    for model in models.iter() {
        let mesh = &model.mesh;

        for v in 0..mesh.positions.len() / 3 {
            result.push(Vec3::new(
                mesh.positions[3 * v],
                mesh.positions[3 * v + 1],
                mesh.positions[3 * v + 2],
            ));
        }
    }

    Ok(result)
}

/// Program that reads in an obj file and generates one for the convex hull.
fn main() {
    let commands = std::env::args().skip(1).collect::<Vec<String>>();
    if commands.len() < 2 {
        eprintln!("usage: command <in.obj> <out.obj>");
        return;
    }
    let in_file = Path::new(&commands[0]);
    let out_file = Path::new(&commands[1]);

    let vert_vec = match load_vertices_from_file(in_file) {
        Ok(vertices) => vertices,
        Err(e) => {
            eprintln!(
                "Failed to load raw data from file {} error {}",
                in_file.display(),
                e
            );
            return;
        }
    };

    let tri_vec = match generate_convex_hull(&vert_vec) {
        Ok(vec) => vec,
        Err(e) => {
            eprintln!("Failed to generate convex hull: {}", e);
            return;
        }
    };

    let mesh = match Mesh::new(&vert_vec, &tri_vec) {
        Ok(mesh) => mesh,
        Err(e) => {
            eprintln!("Internal consistency error: {}", e);
            return;
        }
    };

    if let Err(e) = mesh.save_as_obj_file(out_file) {
        eprintln!(
            "Failed to generate target file {} error {}",
            out_file.display(),
            e
        );
    }
}
