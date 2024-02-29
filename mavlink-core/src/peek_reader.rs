//! This module implements a buffered/peekable reader.
//!
//! The purpose of the buffered/peekable reader is to allow for backtracking parsers.
//!
//! A reader implementing the standard librairy's [`std::io::BufRead`] trait seems like a good fit, but
//! it does not allow for peeking a specific number of bytes, so it provides no way to request
//! more data from the underlying reader without consuming the existing data.
//!
//! This API still tries to adhere to the [`std::io::BufRead`]'s trait philosophy.
//!
//! The main type `PeekReader`does not implement [`std::io::Read`] itself, as there is no added benefit
//! in doing so.
//!

use std::io::{self, ErrorKind, Read};

// The default chunk size to read from the underlying reader
const DEFAULT_CHUNK_SIZE: usize = 1024;

/// A buffered/peekable reader
///
/// This reader wraps a type implementing [`std::io::Read`] and adds buffering via an internal buffer.
///
/// It allows the user to `peek` a specified number of bytes (without consuming them),
/// to `read` bytes (consuming them), or to `consume` them after `peek`ing.
///
pub struct PeekReader<R> {
    // Internal buffer
    buffer: Vec<u8>,
    // The position of the next byte to read in the buffer.
    cursor: usize,
    // The preferred chunk size.  This is just a hint.
    preferred_chunk_size: usize,
    // The wrapped reader.
    reader: R,
    // Stashed error, if any.
    error: Option<io::Error>,
    // Whether we hit EOF on the underlying reader.
    eof: bool,
}

impl<R: Read> PeekReader<R> {
    /// Instanciates a new [`PeekReader`], wrapping the provided [`std::io::Read`]er and using the default chunk size
    pub fn new(reader: R) -> Self {
        Self::with_chunk_size(reader, DEFAULT_CHUNK_SIZE)
    }

    /// Instanciates a new [`PeekReader`], wrapping the provided [`std::io::Read`]er and using the supplied chunk size
    pub fn with_chunk_size(reader: R, preferred_chunk_size: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(preferred_chunk_size),
            cursor: 0,
            preferred_chunk_size,
            reader,
            error: None,
            eof: false,
        }
    }

    /// Peeks a specified amount of bytes from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`std::io::Read`]er until it does, an error occurs or no more data can be read (EOF).
    ///
    /// This function does not consume  data from the buffer, so subsequent calls to `peek` or `read` functions
    /// will still return the peeked data.
    ///
    pub fn peek(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, false, false)
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
    pub fn peek_exact(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, true, false)
    }

    /// Consumes a specified amount of bytes from the buffer
    ///
    /// If the internal buffer does not contain enough data, this function will consume as much data as is buffered.
    ///
    pub fn consume(&mut self, amount: usize) -> usize {
        let amount = amount.min(self.buffer.len() - self.cursor);
        self.cursor += amount;
        amount
    }

    /// Reads a specified amount of bytes from the internal buffer
    ///
    /// If the internal buffer does not contain enough data, this function will read
    /// from the underlying [`std::io::Read`]er until it does, an error occurs or no more data can be read (EOF).
    ///
    /// This function consumes the data from the buffer, unless an error occurs, in which case no data is consumed.
    ///
    pub fn read(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, false, true)
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
    pub fn read_exact(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, true, true)
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
    pub fn read_u8(&mut self) -> io::Result<u8> {
        let buf = self.read_exact(1)?;
        Ok(buf[0])
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
    fn fetch(&mut self, amount: usize, exact: bool, consume: bool) -> io::Result<&[u8]> {
        let previous_len = self.buffer.len();
        let mut buffered = previous_len - self.cursor;

        // the caller requested more bytes tha we have buffered, fetch them from the reader
        if buffered < amount {
            // if we got an earlier EOF, return it
            if self.eof {
                return Err(io::Error::new(
                    ErrorKind::UnexpectedEof,
                    "Unexpected EOF already returned in previous call to reader",
                ));
            }
            // if we have a stashed error, return it (and clear it)
            if let Some(e) = self.error.take() {
                if e.kind() == ErrorKind::UnexpectedEof {
                    self.eof = true;
                }
                return Err(e);
            }

            let needed = amount - buffered;
            let chunk_size = self.preferred_chunk_size.max(needed);
            let mut buf = vec![0u8; chunk_size];

            // read needed bytes from reader
            let mut read = 0;
            while read < needed {
                match self.reader.read(&mut buf[read..]) {
                    Ok(n) => {
                        if n == 0 {
                            break;
                        }
                        read += n;
                    }
                    Err(e) => {
                        self.error = Some(e);
                        break;
                    }
                }
            }
            // if some bytes were read, add them to the buffer
            if read > 0 {
                if self.buffer.capacity() - previous_len < read {
                    // reallocate
                    self.buffer
                        .copy_within(self.cursor..self.cursor + buffered, 0);
                    self.buffer.truncate(buffered);
                    self.cursor = 0;
                }
                self.buffer.extend_from_slice(&buf[..read]);
                buffered += read;
            }

            if buffered == 0 && self.error.is_some() {
                return Err(self.error.take().unwrap());
            }
        }
        if exact && buffered < amount {
            return Err(io::Error::new(ErrorKind::UnexpectedEof, "Unexpected EOF"));
        }

        let result_len = amount.min(buffered);
        let result = &self.buffer[self.cursor..self.cursor + result_len];
        if consume {
            self.cursor += result_len;
        }
        Ok(result)
    }
}
