use crate::error::*;

/// Replacement for std::io::Read + byteorder::ReadBytesExt in no_std envs
pub trait Read {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, MessageReadError> {
        self.read_exact(buf).map(|_| buf.len())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), MessageReadError>;
}

impl<R: embedded_io::Read> Read for R {
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), MessageReadError> {
        embedded_io::Read::read_exact(self, buf).map_err(|_| MessageReadError::Io)
    }
}

/// Replacement for std::io::Write + byteorder::WriteBytesExt in no_std envs
pub trait Write {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError>;
}

impl<W: embedded_io::Write> Write for W {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError> {
        embedded_io::Write::write_all(self, buf).map_err(|_| MessageWriteError::Io)
    }
}
