use std::io::{self, ErrorKind, Read};

const DEFAULT_CHUNK_SIZE: usize = 1024;

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
    /// Whether we hit EOF on the underlying reader.
    eof: bool,
}

impl<R: Read> PeekReader<R> {
    pub fn new(reader: R) -> Self {
        Self::with_chunk_size(reader, DEFAULT_CHUNK_SIZE)
    }

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

    pub fn peek(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, false, false)
    }

    pub fn peek_exact(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, true, false)
    }

    pub fn consume(&mut self, amount: usize) -> usize {
        let amount = amount.min(self.buffer.len() - self.cursor);
        self.cursor += amount;
        amount
    }

    pub fn read(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, false, true)
    }

    pub fn read_exact(&mut self, amount: usize) -> io::Result<&[u8]> {
        self.fetch(amount, true, true)
    }

    pub fn read_u8(&mut self) -> io::Result<u8> {
        let buf = self.read_exact(1)?;
        Ok(buf[0])
    }

    pub fn reader_ref(&mut self) -> &R {
        &self.reader
    }

    pub fn reader_mut(&mut self) -> &mut R {
        &mut self.reader
    }

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
