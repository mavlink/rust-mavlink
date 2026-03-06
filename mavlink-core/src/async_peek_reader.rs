//! This module implements a buffered/peekable reader using async I/O.
//!
//! The purpose of the buffered/peekable reader is to allow for backtracking parsers.
//!
//! This is the async version of [`crate::peek_reader::PeekReader`].
//! A reader implementing the [`futures::io::AsyncBufRead`]/[`futures::io::AsyncBufReadExt`] traits seems like a good fit, but
//! it does not allow for peeking a specific number of bytes, so it provides no way to request
//! more data from the underlying reader without consuming the existing data.
//!
//! This API still tries to adhere to the [`futures::io::AsyncBufRead`]'s trait philosophy.
//!
//! The main type [`AsyncPeekReader`] does not implement [`futures::io::AsyncBufReadExt`] itself, as there is no added benefit
//! in doing so.
//!

#[cfg(doc)]
use std::io::ErrorKind;

use futures::io::AsyncReadExt;

use crate::error::MessageReadError;

/// A buffered/peekable reader
///
/// This reader wraps a type implementing [`futures::io::AsyncRead`] and adds buffering via an internal buffer.
///
/// It allows the user to `peek` a specified number of bytes (without consuming them),
/// to `read` bytes (consuming them), or to `consume` them after `peek`ing.
///
/// NOTE: This reader is generic over the size of the buffer, defaulting to MAVLink's current largest
/// possible message size of 280 bytes
///
pub struct AsyncPeekReader<R, const BUFFER_SIZE: usize = 280> {
    // Internal buffer
    buffer: [u8; BUFFER_SIZE],
    // The position of the next byte to read from the buffer.
    cursor: usize,
    // The position of the next byte to read into the buffer.
    top: usize,
    // The wrapped reader.
    reader: R,
}

impl<R: futures::io::AsyncRead + Unpin, const BUFFER_SIZE: usize> AsyncPeekReader<R, BUFFER_SIZE> {
    /// Instantiates a new [`AsyncPeekReader`], wrapping the provided [`futures::io::AsyncRead`] and using the default chunk size
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
    /// from the underlying [`futures::io::AsyncRead`] until it does, an error occurs or no more data can be read (EOF).
    ///
    /// This function does not consume data from the buffer, so subsequent calls to `peek` or `read` functions
    /// will still return the peeked data.
    ///
    /// # Errors
    ///
    /// - If any error occurs while reading from the underlying [`futures::io::AsyncRead`] it is returned
    /// - If an EOF occurs and the specified amount could not be read, this function will return an [`ErrorKind::UnexpectedEof`].
    ///
    /// # Panics
    ///
    /// Will panic when attempting to read more bytes then `BUFFER_SIZE`
    pub async fn peek_exact(&mut self, amount: usize) -> Result<&[u8], MessageReadError> {
        self.fetch(amount, false).await
    }

    /// Reads a specified amount of bytes from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`futures::io::AsyncRead`] until it does, an error occurs or no more data can be read (EOF).
    ///
    /// This function consumes the data from the buffer, unless an error occurs, in which case no data is consumed.
    ///
    /// # Errors
    ///
    /// - If any error occurs while reading from the underlying [`futures::io::AsyncRead`] it is returned
    /// - If an EOF occurs and the specified amount could not be read, this function will return an [`ErrorKind::UnexpectedEof`].
    ///
    /// # Panics
    ///
    /// Will panic when attempting to read more bytes then `BUFFER_SIZE`
    pub async fn read_exact(&mut self, amount: usize) -> Result<&[u8], MessageReadError> {
        self.fetch(amount, true).await
    }

    /// Reads a byte from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`tokio::io::AsyncReadExt`] until it does, an error occurs or no more data can be read (EOF).
    ///
    /// This function consumes the data from the buffer, unless an error occurs, in which case no data is consumed.
    ///
    /// # Errors
    ///
    /// - If any error occurs while reading from the underlying [`tokio::io::AsyncReadExt`] it is returned
    /// - If an EOF occurs before a byte could be read, this function will return an [`ErrorKind::UnexpectedEof`].
    ///
    /// # Panics
    ///
    /// Will panic if this `AsyncPeekReader`'s `BUFFER_SIZE` is 0.  
    pub async fn read_u8(&mut self) -> Result<u8, MessageReadError> {
        let buf = self.read_exact(1).await?;
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

    /// Returns an immutable reference to the underlying [`futures::io::AsyncRead`]
    ///
    /// Reading directly from the underlying reader will cause data loss
    pub fn reader_ref(&mut self) -> &R {
        &self.reader
    }

    /// Returns a mutable reference to the underlying [`futures::io::AsyncRead`]
    ///
    /// Reading directly from the underlying reader will cause data loss
    pub fn reader_mut(&mut self) -> &mut R {
        &mut self.reader
    }

    /// Internal function to fetch data from the internal buffer and/or reader
    async fn fetch(&mut self, amount: usize, consume: bool) -> Result<&[u8], MessageReadError> {
        assert!(BUFFER_SIZE >= amount);

        let buffered = self.top - self.cursor;

        // the caller requested more bytes than we have buffered, fetch them from the reader
        if buffered < amount {
            let bytes_needed = amount - buffered;

            // Check if we have space at the tail. If not, compact the buffer.
            if self.top + bytes_needed > self.buffer.len() {
                // Move active data to the beginning of the buffer
                self.buffer.copy_within(self.cursor..self.top, 0);
                self.cursor = 0;
                self.top = buffered;
            }

            // Read directly into the internal buffer.
            let dest = &mut self.buffer[self.top..self.top + bytes_needed];
            self.reader.read_exact(dest).await?;

            self.top += bytes_needed;
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

    #[tokio::test]
    #[should_panic(expected = "assertion failed")]
    async fn test_peek_exact_panics_when_amount_exceeds_buffer_size() {
        let data = b"abcd";
        let mut reader = AsyncPeekReader::<_, 4>::new(&data[..]);
        let _ = reader.peek_exact(5).await;
    }

    #[tokio::test]
    #[should_panic(expected = "assertion failed")]
    async fn test_read_exact_panics_when_amount_exceeds_buffer_size() {
        let data = b"abcd";
        let mut reader = AsyncPeekReader::<_, 4>::new(&data[..]);
        let _ = reader.read_exact(5).await;
    }
}
