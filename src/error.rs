use std::error::Error;
use std::fmt::{Display, Formatter};

#[derive(Debug)]
pub enum ParserError {
    InvalidFlag { flag_type: String, value: u32 },
    InvalidEnum { enum_type: String, value: u32 },
    UnknownMessage { id: u32 },
}

impl Display for ParserError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
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

impl Error for ParserError {}

#[derive(Debug)]
pub enum MessageReadError {
    Io(std::io::Error),
    Parse(ParserError),
}

impl Display for MessageReadError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "Failed to read message: {:#?}", e),
            Self::Parse(e) => write!(f, "Failed to read message: {:#?}", e),
        }
    }
}

impl Error for MessageReadError {}

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
