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
    /// Reading directly from the underlying stream will cause data loss
    pub fn reader_ref(&mut self) -> &R {
        &self.reader
    }

    /// Returns a mutable reference to the underlying [`std::io::Read`]er
    ///
    /// Reading directly from the underlying stream will cause data loss
    pub fn reader_mut(&mut self) -> &mut R {
        &mut self.reader
    }

    /// Internal function to fetch data from the internal buffer and/or reader
    fn fetch(&mut self, amount: usize, consume: bool) -> Result<&[u8], MessageReadError> {
        let buffered = self.top - self.cursor;

        // the caller requested more bytes than we have buffered, fetch them from the reader
        if buffered < amount {
            let bytes_read = amount - buffered;
            assert!(bytes_read < BUFFER_SIZE);
            let mut buf = [0u8; BUFFER_SIZE];

            // read needed bytes from reader
            self.reader.read_exact(&mut buf[..bytes_read])?;

            // if some bytes were read, add them to the buffer

            if self.buffer.len() - self.top < bytes_read {
                // reallocate
                self.buffer.copy_within(self.cursor..self.top, 0);
                self.cursor = 0;
                self.top = buffered;
            }
            self.buffer[self.top..self.top + bytes_read].copy_from_slice(&buf[..bytes_read]);
            self.top += bytes_read;
        }

        let result = &self.buffer[self.cursor..self.cursor + amount];
        if consume {
            self.cursor += amount;
        }
        Ok(result)
    }
}
