//! This module implements a buffered/peekable reader.
//!
//! The purpose of the buffered/peekable reader is to allow for backtracking parsers.
//!
//! A reader implementing the standard library's [`std::io::BufRead`] trait seems like a good fit, but
//! it does not allow for peeking a specific number of bytes, so it provides no way to request
//! more data from the underlying reader without consuming the existing data.
//!
//! This API still tries to adhere to the [`std::io::BufRead`]'s trait philosophy.
//!
//! The main type `PeekReader`does not implement [`std::io::Read`] itself, as there is no added benefit
//! in doing so.
//!
#[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
use crate::embedded::Read;

#[cfg(feature = "std")]
use std::io::Read;

#[cfg(doc)]
use std::io::ErrorKind;

use crate::error::MessageReadError;

/// A buffered/peekable reader
///
/// This reader wraps a type implementing [`std::io::Read`] and adds buffering via an internal buffer.
///
/// It allows the user to `peek` a specified number of bytes (without consuming them),
/// to `read` bytes (consuming them), or to `consume` them after `peek`ing.
///
/// NOTE: This reader is generic over the size of the buffer, defaulting to MAVLink's current largest
/// possible message size of 280 bytes
///
pub struct PeekReader<R, const BUFFER_SIZE: usize = 280> {
    // Internal buffer
    buffer: [u8; BUFFER_SIZE],
    // The position of the next byte to read from the buffer.
    cursor: usize,
    // The position of the next byte to read into the buffer.
    top: usize,
    // The wrapped reader.
    reader: R,
}

impl<R: Read, const BUFFER_SIZE: usize> PeekReader<R, BUFFER_SIZE> {
    /// Instantiates a new [`PeekReader`], wrapping the provided [`std::io::Read`]er and using the default chunk size
    pub fn new(reader: R) -> Self {
        Self {
            buffer: [0; BUFFER_SIZE],
            cursor: 0,
            top: 0,
            reader,
        }
    }

    /// Peeks an exact amount of bytes from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`std::io::Read`]er until it does, an error occurs or no more data can be read (EOF).
    ///
    /// If an EOF occurs and the specified amount could not be read, this function will return an [`ErrorKind::UnexpectedEof`].
    ///
    /// This function does not consume data from the buffer, so subsequent calls to `peek` or `read` functions
    /// will still return the peeked data.
    ///
    pub fn peek_exact(&mut self, amount: usize) -> Result<&[u8], MessageReadError> {
        let result = self.fetch(amount, false);
        result
    }

    /// Reads a specified amount of bytes from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`std::io::Read`]er until it does, an error occurs or no more data can be read (EOF).
    ///
    /// If an EOF occurs and the specified amount could not be read, this function will return an [`ErrorKind::UnexpectedEof`].
    ///
    /// This function consumes the data from the buffer, unless an error occurs, in which case no data is consumed.
    ///
    pub fn read_exact(&mut self, amount: usize) -> Result<&[u8], MessageReadError> {
        self.fetch(amount, true)
    }

    /// Reads a byte from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`std::io::Read`]er until it does, an error occurs or no more data can be read (EOF).
    ///
    /// If an EOF occurs and the specified amount could not be read, this function will return an [`ErrorKind::UnexpectedEof`].
    ///
    /// This function consumes the data from the buffer, unless an error occurs, in which case no data is consumed.
    ///
    pub fn read_u8(&mut self) -> Result<u8, MessageReadError> {
        let buf = self.read_exact(1)?;
        Ok(buf[0])
    }

    /// Consumes a specified amount of bytes from the buffer
    ///
    /// If the internal buffer does not contain enough data, this function will consume as much data as is buffered.
    ///
    pub fn consume(&mut self, amount: usize) -> usize {
        let amount = amount.min(self.top - self.cursor);
        self.cursor += amount;
        amount
    }

    /// Returns an immutable reference to the underlying [`std::io::Read`]er
    ///
    /// Reading directly from the underlying reader will cause data loss
    pub fn reader_ref(&self) -> &R {
        &self.reader
    }

    /// Returns a mutable reference to the underlying [`std::io::Read`]er
    ///
    /// Reading directly from the underlying reader will cause data loss
    pub fn reader_mut(&mut self) -> &mut R {
        &mut self.reader
    }

    /// Internal function to fetch data from the internal buffer and/or reader
    fn fetch(&mut self, amount: usize, consume: bool) -> Result<&[u8], MessageReadError> {
        loop {
            let buffered = self.top - self.cursor;

            if buffered >= amount {
                break;
            }

            // the caller requested more bytes than we have buffered, fetch them from the reader
            let bytes_to_read = amount - buffered;
            assert!(bytes_to_read < BUFFER_SIZE);

            // Check if we need to compact the buffer first
            if self.top + bytes_to_read > BUFFER_SIZE {
                // Move unread data to the beginning of the buffer
                self.buffer.copy_within(self.cursor..self.top, 0);
                self.top = buffered;
                self.cursor = 0;
            }

            // Now we can safely read directly into the buffer
            let end_pos = self.top + bytes_to_read;

            // read needed bytes from reader
            let bytes_read = self.reader.read(&mut self.buffer[self.top..end_pos])?;

            if bytes_read == 0 {
                return Err(MessageReadError::eof());
            }

            self.top += bytes_read;
        }

        let result = &self.buffer[self.cursor..self.cursor + amount];
        if consume {
            self.cursor += amount;
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::io::Cursor;
    use std::io::{self};

    #[test]
    fn test_read_and_peek() {
        let data = b"Hello, World!";
        let cursor = Cursor::new(data);
        let mut reader = PeekReader::<_, 280>::new(cursor);

        let peeked = reader.peek_exact(5).unwrap();
        assert_eq!(peeked, b"Hello");

        let read = reader.read_exact(5).unwrap();
        assert_eq!(read, b"Hello");

        // Make sure `PeekReader::read_exact` consumed the first 5 bytes.
        let read = reader.read_exact(8).unwrap();
        assert_eq!(read, b", World!");

        match reader.read_u8().unwrap_err() {
            MessageReadError::Io(io_err) => {
                assert_eq!(io_err.kind(), io::ErrorKind::UnexpectedEof);
            }
            _ => panic!("Expected Io error with UnexpectedEof"),
        }
    }
}
