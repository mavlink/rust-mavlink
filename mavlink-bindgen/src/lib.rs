pub use crate::error::BindGenError;
use std::fs::{read_dir, File};
use std::io::{self, BufWriter};
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

/// Specifies the source(s) of MAVLink XML definition files used for generating
/// Rust MAVLink dialect bindings.
pub enum XmlDefinitions<T: AsRef<Path>> {
    /// A collection of individual MAVLink XML definition files.
    Files(Vec<T>),
    /// A directory containing one or more MAVLink XML definition files.
    Directory(T),
}

/// Generate Rust MAVLink dialect binding for dialects present in the given `xml_definitions`
/// into `destination_dir`.
///
/// If successful returns paths of generated bindings linked to their dialect definitions files.
pub fn generate<P1: AsRef<Path>, P2: AsRef<Path>>(
    xml_definitions: XmlDefinitions<P1>,
    destination_dir: P2,
) -> Result<GeneratedBindings, BindGenError> {
    let destination_dir = destination_dir.as_ref();

    let mut bindings = vec![];

    match xml_definitions {
        XmlDefinitions::Files(files) => {
            if files.is_empty() {
                return Err(
                    BindGenError::CouldNotReadDirectoryEntryInDefinitionsDirectory {
                        source: io::Error::new(
                            io::ErrorKind::InvalidInput,
                            "At least one file must be given.",
                        ),
                        path: PathBuf::default(),
                    },
                );
            }

            for file in files {
                let file = file.as_ref();

                bindings.push(generate_single_file(file, destination_dir)?);
            }
        }
        XmlDefinitions::Directory(definitions_dir) => {
            let definitions_dir = definitions_dir.as_ref();

            if !definitions_dir.is_dir() {
                return Err(
                    BindGenError::CouldNotReadDirectoryEntryInDefinitionsDirectory {
                        source: io::Error::new(
                            io::ErrorKind::InvalidInput,
                            format!("{} is not a directory.", definitions_dir.display()),
                        ),
                        path: definitions_dir.to_owned(),
                    },
                );
            }

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

                let definition_filename = PathBuf::from(entry.file_name());
                // Skip non-XML files
                if !definition_filename.extension().is_some_and(|e| e == "xml") {
                    continue;
                }

                bindings.push(generate_single_file(entry.path(), destination_dir)?);
            }
        }
    };

    // Creating `mod.rs`
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

/// Generate a Rust MAVLink dialect binding for the given `source_file` dialect into `destination_dir`.
///
/// If successful returns path of the generated binding linked to their dialect definition file.
fn generate_single_file<P1: AsRef<Path>, P2: AsRef<Path>>(
    source_file: P1,
    destination_dir: P2,
) -> Result<GeneratedBinding, BindGenError> {
    let source_file = source_file.as_ref();
    let destination_dir = destination_dir.as_ref();

    let definitions_dir = source_file.parent().unwrap_or(Path::new(""));

    if !source_file.exists() {
        return Err(
            BindGenError::CouldNotReadDirectoryEntryInDefinitionsDirectory {
                source: io::Error::new(io::ErrorKind::NotFound, "File not found."),
                path: definitions_dir.to_owned(),
            },
        );
    }

    if !source_file.extension().is_some_and(|e| e == "xml") {
        return Err(
            BindGenError::CouldNotReadDirectoryEntryInDefinitionsDirectory {
                source: io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "Non-XML files are not supported.",
                ),
                path: definitions_dir.to_owned(),
            },
        );
    }

    let definition_filename = PathBuf::from(source_file.file_name().unwrap());
    let module_name = util::to_module_name(&definition_filename);
    let definition_rs = PathBuf::from(&module_name).with_extension("rs");

    let dest_path = destination_dir.join(definition_rs);
    let mut outf = BufWriter::new(File::create(&dest_path).map_err(|source| {
        BindGenError::CouldNotCreateRustBindingsFile {
            source,
            dest_path: dest_path.clone(),
        }
    })?);

    // codegen
    parser::generate(definitions_dir, &definition_filename, &mut outf)?;

    Ok(GeneratedBinding {
        module_name,
        mavlink_xml: source_file.to_owned(),
        rust_module: dest_path,
    })
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
        if std::env::args()
            .next()
            .unwrap_or_default()
            .contains("build-script-")
        {
            println!("cargo:warning=Failed to run rustfmt: {error}");
        } else {
            eprintln!("Failed to run rustfmt: {error}");
        }
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
