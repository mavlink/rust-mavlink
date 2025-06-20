use thiserror::Error;

#[derive(Error, Debug)]
pub enum BindGenError {
    /// Represents a failure to read the MAVLink definitions directory.
    #[error("Could not read definitions directory {}: {source}", path.display())]
    CouldNotReadDefinitionsDirectory {
        source: std::io::Error,
        path: std::path::PathBuf,
    },
    /// Represents a failure to read a MAVLink definition file.
    #[error("Could not read definition file {}: {source}", path.display())]
    CouldNotReadDefinitionFile {
        source: std::io::Error,
        path: std::path::PathBuf,
    },
    /// Represents a failure to read a directory entry in the MAVLink definitions directory.
    #[error("Could not read MAVLink definitions directory entry {}: {source}", path.display())]
    CouldNotReadDirectoryEntryInDefinitionsDirectory {
        source: std::io::Error,
        path: std::path::PathBuf,
    },
    /// Represents a failure to create a Rust file for the generated MAVLink bindings.
    #[error("Could not create Rust bindings file {}: {source}", dest_path.display())]
    CouldNotCreateRustBindingsFile {
        source: std::io::Error,
        dest_path: std::path::PathBuf,
    },
}
