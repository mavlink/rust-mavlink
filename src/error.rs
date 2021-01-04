use core::fmt::{Display, Formatter};
#[cfg(feature = "std")]
use std::error::Error;

#[cfg(not(feature = "std"))]
use alloc::string::String;

#[derive(Debug)]
pub enum ParserError {
    InvalidFlag { flag_type: String, value: u32 },
    InvalidEnum { enum_type: String, value: u32 },
    UnknownMessage { id: u32 },
}

impl Display for ParserError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            ParserError::InvalidFlag { flag_type, value } => write!(
                f,
                "Invalid flag value for flag type {:?}, got {:?}",
                flag_type, value
            ),
            ParserError::InvalidEnum { enum_type, value } => write!(
                f,
                "Invalid enum value for enum type {:?}, got {:?}",
                enum_type, value
            ),
            ParserError::UnknownMessage { id } => write!(f, "Unknown message with ID {:?}", id),
        }
    }
}

#[cfg(feature = "std")]
impl Error for ParserError {}

#[derive(Debug)]
pub enum MessageReadError {
    #[cfg(feature = "std")]
    Io(std::io::Error),
    #[cfg(feature = "embedded")]
    Io,
    Parse(ParserError),
}

impl Display for MessageReadError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "std")]
            Self::Io(e) => write!(f, "Failed to read message: {:#?}", e),
            #[cfg(feature = "embedded")]
            Self::Io => write!(f, "Failed to read message"),
            Self::Parse(e) => write!(f, "Failed to read message: {:#?}", e),
        }
    }
}

#[cfg(feature = "std")]
impl Error for MessageReadError {}

#[cfg(feature = "std")]
impl From<std::io::Error> for MessageReadError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<ParserError> for MessageReadError {
    fn from(e: ParserError) -> Self {
        Self::Parse(e)
    }
}

#[derive(Debug)]
pub enum MessageWriteError {
    #[cfg(feature = "std")]
    Io(std::io::Error),
    #[cfg(feature = "embedded")]
    Io,
}

impl Display for MessageWriteError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "std")]
            Self::Io(e) => write!(f, "Failed to write message: {:#?}", e),
            #[cfg(feature = "embedded")]
            Self::Io => write!(f, "Failed to write message"),
        }
    }
}

#[cfg(feature = "std")]
impl Error for MessageWriteError {}

#[cfg(feature = "std")]
impl From<std::io::Error> for MessageWriteError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}
