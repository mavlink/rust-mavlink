use std::path::PathBuf;

use clap::Parser;
use mavlink_bindgen::{
    emit_cargo_build_messages, format_generated_code, generate, BindGenError, XmlDefinitions,
};

#[derive(Parser)]
/// Generate Rust bindings from MAVLink message dialect XML files.
struct Cli {
    /// Path to the directory containing the MAVLink dialect definitions.
    definitions_dir: PathBuf,
    /// Path to the directory where the code is generated into, must already exist.
    destination_dir: PathBuf,
    /// format code generated code, requires rustfmt to be installed
    #[arg(long)]
    format_generated_code: bool,
    /// prints cargo build messages indicating when the code has to be rebuild
    #[arg(long)]
    emit_cargo_build_messages: bool,
}

pub fn main() -> Result<(), BindGenError> {
    let args = Cli::parse();
    let result = generate(
        XmlDefinitions::Directory(args.definitions_dir),
        args.destination_dir,
    )?;

    if args.format_generated_code {
        format_generated_code(&result);
    }

    if args.emit_cargo_build_messages {
        emit_cargo_build_messages(&result);
    }

    Ok(())
}
