use std::path::PathBuf;

use clap::Parser;
use mavlink_bindgen::{emit_cargo_build_messages, format_generated_code, generate, BindGenError};

#[derive(Parser)]
struct Cli {
    definitions_dir: PathBuf,
    destination_dir: PathBuf,
    #[arg(long)]
    format_generated_code: bool,
    #[arg(long)]
    emit_cargo_build_messages: bool,
}

pub fn main() -> Result<(), BindGenError> {
    let args = Cli::parse();
    let result = generate(args.definitions_dir, args.destination_dir)?;

    if args.format_generated_code {
        format_generated_code(&result);
    }

    if args.emit_cargo_build_messages {
        emit_cargo_build_messages(&result);
    }

    Ok(())
}
