#![recursion_limit = "256"]

use std::env;
use std::fs::read_dir;
use std::path::Path;
use std::process::Command;

pub fn main() {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR"));

    // Update and init submodule
    if let Err(error) = Command::new("git")
        .arg("submodule")
        .arg("update")
        .arg("--init")
        .current_dir(src_dir)
        .status()
    {
        eprintln!("{error}");
    }

    // find & apply patches to XML definitions to avoid crashes
    let mut patch_dir = src_dir.to_path_buf();
    patch_dir.push("build/patches");
    let mut mavlink_dir = src_dir.to_path_buf();
    mavlink_dir.push("mavlink");

    if let Ok(dir) = read_dir(patch_dir) {
        for entry in dir.flatten() {
            if let Err(error) = Command::new("git")
                .arg("apply")
                .arg(entry.path().as_os_str())
                .current_dir(&mavlink_dir)
                .status()
            {
                eprintln!("{error}");
            }
        }
    }

    let mut definitions_dir = src_dir.to_path_buf();
    definitions_dir.push("mavlink/message_definitions/v1.0");

    let out_dir = env::var("OUT_DIR").unwrap();

    let result = mavlink_bindgen::generate(definitions_dir, out_dir)
        .expect("Failed to generate Rust MAVLink bindings");

    #[cfg(feature = "format-generated-code")]
    mavlink_bindgen::format_generated_code(&result);

    mavlink_bindgen::emit_cargo_build_messages(&result);
}
