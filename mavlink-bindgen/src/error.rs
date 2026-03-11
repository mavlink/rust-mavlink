use std::num::{ParseFloatError, ParseIntError};

use quick_xml::events::Event;
use thiserror::Error;

use crate::parser_new::MavXmlElement;

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
    /// Occurs when a MAVLink XML definition that is not valid
    #[error("MAVLink definition file {} is invalid: {source}", path.display())]
    InvalidDefinitionFile {
        source: MavXMLParseError,
        path: std::path::PathBuf,
    },
    /// Occurs when a MAVLink XML definition refences an enum that is not defined in it or its includes
    #[error("MAVLink definition file {} references undefined enum {enum_name}", path.display())]
    InvalidEnumReference {
        enum_name: String,
        path: std::path::PathBuf,
    },
    #[error("bitflag enum field {field_name} of {message_name} must be able to fit all possible values for {enum_name}")]
    BitflagEnumTypeToNarrow {
        field_name: String,
        message_name: String,
        enum_name: String,
    },
}

#[derive(Error, Debug)]
pub enum MavXMLParseError {
    #[error("{0}")]
    QuickXMLError(#[from] quick_xml::errors::Error),
    #[error("{0}")]
    ParseIntFailed(#[from] ParseIntError),
    #[error("{0}")]
    ParseFloatFailed(#[from] ParseFloatError),
    #[error("Unexpected XML element {event:?} in {parent:?}")]
    UnexpectedEvent {
        event: Event<'static>,
        parent: MavXmlElement,
    },
    #[error("Extentsion tags may not contain elements")]
    NoneSelfClosingExtTag,
    #[error("Unexpected tag {element:?}")]
    UnexpectedElement { element: MavXmlElement },
    #[error("Expected text")]
    ExpectedText,
    #[error("MAVLink definition file ends unexpectedly")]
    UnexpectedEnd,
    #[error("Param index must be between 1 and 7")]
    ParamIndexOutOfRange,
    #[error("Expected start of MAVLink definition")]
    ExpectedMAVLink,
    #[error("Invalid field type: {mav_type}")]
    InvalidType { mav_type: String },
    #[error("Duplicate field '{field}' found in message '{message}' while generating bindings")]
    DuplicateFieldName { field: String, message: String },
    #[error("Message '{message}' must have between 1 and 64 fields")]
    InvalidFieldCount { message: String },
}
