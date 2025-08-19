#![recursion_limit = "256"]

use std::env;
use std::fs::read_dir;
use std::path::Path;
use std::process::{Command, ExitCode};

use mavlink_bindgen::XmlDefinitions;

fn main() -> ExitCode {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR"));

    // Check if git is installed
    if Command::new("git").arg("--version").status().is_err() {
        eprintln!("error: Git is not installed or could not be found.");
        return ExitCode::FAILURE;
    }

    // Update and init submodule
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

    // find & apply patches to XML definitions to avoid crashes
    let patch_dir = src_dir.join("build/patches");
    let mavlink_dir = src_dir.join("mavlink");

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

    let out_dir = env::var("OUT_DIR").unwrap();

    let source_definitions_dir = src_dir.join("mavlink/message_definitions/v1.0");

    let enabled_features: Vec<String> = env::vars()
        .filter_map(|(key, _)| key.strip_prefix("CARGO_FEATURE_").map(str::to_lowercase))
        .collect();

    let mut definitions_to_bind = vec![];

    if let Ok(dir) = read_dir(&source_definitions_dir) {
        for entry in dir.flatten() {
            let filename = entry
                .path()
                .file_stem()
                .unwrap()
                .to_string_lossy()
                .to_lowercase();

            if enabled_features.contains(&filename) {
                definitions_to_bind.push(entry.path());
            }
        }
    }

    let xml_definitions = if definitions_to_bind.is_empty() {
        XmlDefinitions::Directory(source_definitions_dir)
    } else {
        XmlDefinitions::Files(definitions_to_bind)
    };

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
