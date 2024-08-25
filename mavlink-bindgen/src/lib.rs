pub use crate::error::BindGenError;
use std::fs::{read_dir, File};
use std::io::BufWriter;
use std::ops::Deref;
use std::path::{Path, PathBuf};
use std::process::Command;

pub mod binder;
pub mod error;
pub mod parser;
mod util;

#[derive(Debug)]
pub struct GeneratedBinding {
    pub module_name: String,
    pub mavlink_xml: PathBuf,
    pub rust_module: PathBuf,
}

#[derive(Debug)]
pub struct GeneratedBindings {
    pub bindings: Vec<GeneratedBinding>,
    pub mod_rs: PathBuf,
}

/// Generate Rust MAVLink dialect binding for dialects present in `definitions_dir` into `destination_dir`.
///
/// If successful returns paths of generated bindings linked to their dialect definitions files.
pub fn generate<P1: AsRef<Path>, P2: AsRef<Path>>(
    definitions_dir: P1,
    destination_dir: P2,
) -> Result<GeneratedBindings, BindGenError> {
    _generate(definitions_dir.as_ref(), destination_dir.as_ref())
}

fn _generate(
    definitions_dir: &Path,
    destination_dir: &Path,
) -> Result<GeneratedBindings, BindGenError> {
    let mut bindings = vec![];

    for entry_maybe in read_dir(definitions_dir).map_err(|source| {
        BindGenError::CouldNotReadDefinitionsDirectory {
            source,
            path: definitions_dir.to_path_buf(),
        }
    })? {
        let entry = entry_maybe.map_err(|source| {
            BindGenError::CouldNotReadDirectoryEntryInDefinitionsDirectory {
                source,
                path: definitions_dir.to_path_buf(),
            }
        })?;

        let definition_file = PathBuf::from(entry.file_name());
        let module_name = util::to_module_name(&definition_file);

        let definition_rs = PathBuf::from(&module_name).with_extension("rs");

        let dest_path = destination_dir.join(definition_rs);
        let mut outf = BufWriter::new(File::create(&dest_path).map_err(|source| {
            BindGenError::CouldNotCreateRustBindingsFile {
                source,
                dest_path: dest_path.clone(),
            }
        })?);

        // generate code
        parser::generate(definitions_dir, &definition_file, &mut outf)?;

        bindings.push(GeneratedBinding {
            module_name,
            mavlink_xml: entry.path(),
            rust_module: dest_path,
        });
    }

    // output mod.rs
    {
        let dest_path = destination_dir.join("mod.rs");
        let mut outf = File::create(&dest_path).map_err(|source| {
            BindGenError::CouldNotCreateRustBindingsFile {
                source,
                dest_path: dest_path.clone(),
            }
        })?;

        // generate code
        binder::generate(
            bindings
                .iter()
                .map(|binding| binding.module_name.deref())
                .collect(),
            &mut outf,
        );

        Ok(GeneratedBindings {
            bindings,
            mod_rs: dest_path,
        })
    }
}

/// Formats generated code using `rustfmt`.
pub fn format_generated_code(result: &GeneratedBindings) {
    if let Err(error) = Command::new("rustfmt")
        .args(
            result
                .bindings
                .iter()
                .map(|binding| binding.rust_module.clone()),
        )
        .arg(result.mod_rs.clone())
        .status()
    {
        eprintln!("{error}");
    }
}

/// Prints definitions for cargo that describe which files the generated code depends on, indicating when it has to be regenerated.
pub fn emit_cargo_build_messages(result: &GeneratedBindings) {
    for binding in &result.bindings {
        // Re-run build if definition file changes
        println!(
            "cargo:rerun-if-changed={}",
            binding.mavlink_xml.to_string_lossy()
        );
    }
}
