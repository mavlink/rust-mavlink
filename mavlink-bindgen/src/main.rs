#![recursion_limit = "256"]

#[cfg(feature = "cli")]
mod cli;

pub fn main() {
    #[cfg(feature = "cli")]
    cli::main();
    #[cfg(not(feature = "cli"))]
    panic!("Compiled without cli feature");
}
