use core::fmt::{Display, Formatter};
#[cfg(feature = "std")]
use std::error::Error;

/// Error while parsing a MAVLink message
#[derive(Debug)]
pub enum ParserError {
    /// Bit flag for this type is invalid
    InvalidFlag { flag_type: &'static str, value: u64 },
    /// Enum value for this enum type does not exist
    InvalidEnum { enum_type: &'static str, value: u64 },
    /// Message ID does not exist in this message set
    UnknownMessage { id: u32 },
}

impl Display for ParserError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidFlag { flag_type, value } => write!(
                f,
                "Invalid flag value for flag type {flag_type:?}, got {value:?}"
            ),
            Self::InvalidEnum { enum_type, value } => write!(
                f,
                "Invalid enum value for enum type {enum_type:?}, got {value:?}"
            ),
            Self::UnknownMessage { id } => write!(f, "Unknown message with ID {id:?}"),
        }
    }
}

#[cfg(feature = "std")]
impl Error for ParserError {}

/// Error while reading and parsing a MAVLink message
#[derive(Debug)]
pub enum MessageReadError {
    /// IO Error while reading
    #[cfg(feature = "std")]
    Io(std::io::Error),
    /// IO Error while reading
    #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
    Io,
    /// Error while parsing
    Parse(ParserError),
}

impl MessageReadError {
    pub fn eof() -> Self {
        #[cfg(feature = "std")]
        return Self::Io(std::io::ErrorKind::UnexpectedEof.into());
        #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
        return Self::Io;
    }
}

impl Display for MessageReadError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "std")]
            Self::Io(e) => write!(f, "Failed to read message: {e:#?}"),
            #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
            Self::Io => write!(f, "Failed to read message"),
            Self::Parse(e) => write!(f, "Failed to read message: {e:#?}"),
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

/// Error while writing a MAVLink message
#[derive(Debug)]
pub enum MessageWriteError {
    /// IO Error while writing
    #[cfg(feature = "std")]
    Io(std::io::Error),
    /// IO Error while writing
    #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
    Io,
    /// Message does not support MAVLink 1
    MAVLink2Only,
}

impl Display for MessageWriteError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "std")]
            Self::Io(e) => write!(f, "Failed to write message: {e:#?}"),
            #[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
            Self::Io => write!(f, "Failed to write message"),
            Self::MAVLink2Only => write!(f, "Message is not supported in MAVLink 1"),
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
