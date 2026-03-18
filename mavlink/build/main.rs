#![recursion_limit = "256"]

use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::read_dir;
use std::path::Path;
use std::process::{Command, ExitCode};

use mavlink_bindgen::XmlDefinitions;

fn main() -> ExitCode {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mavlink_dir = src_dir.join("mavlink");

    // It is a submodule if it contains `.git` or if it's completely empty (uninitialized)
    let is_mavlink_empty = read_dir(&mavlink_dir)
        .map(|mut d| d.next().is_none())
        .unwrap_or(true);
    let is_submodule = mavlink_dir.join(".git").exists() || is_mavlink_empty;

    if is_submodule {
        if let Err(error) = Command::new("git")
            .arg("submodule")
            .arg("update")
            .arg("--init")
            .current_dir(src_dir)
            .status()
        {
            eprintln!("Failed to update MAVLink definitions submodule: {error}");
            return ExitCode::FAILURE;
        }
    }

    // find & apply patches to XML definitions to avoid crashes
    let patch_dir = src_dir.join("build/patches");
    if let Ok(dir) = read_dir(patch_dir) {
        for entry in dir.flatten() {
            if let Err(error) = Command::new("git")
                .arg("apply")
                .arg(entry.path().as_os_str())
                .current_dir(&mavlink_dir)
                .status()
            {
                eprintln!("Failed to apply MAVLink definitions patches: {error}");
                return ExitCode::FAILURE;
            }
        }
    }

    let source_definitions_dir = mavlink_dir.join("message_definitions/v1.0");

    // Check if the source definitions directory exists
    if !source_definitions_dir.is_dir() {
        eprintln!(
            "MAVLink message definitions directory not found at: {}\n\
             Ensure submodules are included.",
            source_definitions_dir.display(),
        );
        return ExitCode::FAILURE;
    }

    let enabled_dialects: BTreeSet<String> = env::vars()
        .filter_map(|(key, _)| {
            key.strip_prefix("CARGO_FEATURE_DIALECT_")
                .map(str::to_lowercase)
        })
        .collect();

    let mut definitions_to_bind = BTreeSet::new();

    if !enabled_dialects.is_empty() {
        // Handle case-insensitive Cargo features against case-sensitive files e.g., `csAirLink.xml`.
        let mut available_dialects = BTreeMap::new();
        for entry in read_dir(&source_definitions_dir)
            .into_iter()
            .flatten()
            .flatten()
        {
            let path = entry.path();
            let Some(stem) = path.file_stem() else {
                continue;
            };

            available_dialects.insert(stem.to_string_lossy().to_lowercase(), path);
        }

        // Check if the expected dialects requested by Cargo features are missing
        for dialect in &enabled_dialects {
            let Some(actual_path) = available_dialects.get(dialect) else {
                eprintln!(
                    "Dialect definition for '{}' not found in {}",
                    dialect,
                    source_definitions_dir.display(),
                );
                return ExitCode::FAILURE;
            };

            definitions_to_bind.insert(actual_path.clone());
        }
    }

    let xml_definitions = if definitions_to_bind.is_empty() {
        XmlDefinitions::Directory(source_definitions_dir)
    } else {
        XmlDefinitions::Files(definitions_to_bind.into_iter().collect())
    };

    let out_dir = env::var("OUT_DIR").unwrap();
    let result = match mavlink_bindgen::generate(xml_definitions, out_dir) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };

    #[cfg(feature = "format-generated-code")]
    mavlink_bindgen::format_generated_code(&result);

    mavlink_bindgen::emit_cargo_build_messages(&result);

    ExitCode::SUCCESS
}
