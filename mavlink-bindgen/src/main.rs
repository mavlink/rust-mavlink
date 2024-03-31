#![recursion_limit = "256"]

use std::process::ExitCode;

#[cfg(feature = "cli")]
mod cli;

fn main() -> ExitCode {
    #[cfg(feature = "cli")]
    if let Err(e) = cli::main() {
        eprintln!("{e}");
        return ExitCode::FAILURE;
    }

    #[cfg(not(feature = "cli"))]
    panic!("Compiled without cli feature");

    #[cfg(feature = "cli")]
    ExitCode::SUCCESS
}
