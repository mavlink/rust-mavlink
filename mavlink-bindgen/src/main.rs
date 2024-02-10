#![recursion_limit = "256"]

use std::path::PathBuf;

use clap::Parser;
use mavlink_bindgen::{emit_cargo_build_messages, format_generated_code, generate};

#[derive(Parser)]
struct Cli {
    definitions_dir: PathBuf,
    destination_dir: PathBuf,
    #[arg(long)]
    format_generated_code: bool,
    #[arg(long)]
    emit_cargo_build_messages: bool,
}

pub fn main() {
    let args = Cli::parse();
    let result = generate(args.definitions_dir, args.destination_dir)
        .expect("failed to generate MAVLink Rust bindings");

    if args.format_generated_code {
        format_generated_code(&result);
    }

    if args.emit_cargo_build_messages {
        emit_cargo_build_messages(&result);
    }
}
