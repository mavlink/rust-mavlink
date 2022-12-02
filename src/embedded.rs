use crate::error::*;

/// Replacement for std::io::Read + byteorder::ReadBytesExt in no_std envs
pub trait Read {
    fn read_u8(&mut self) -> Result<u8, MessageReadError>;

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), MessageReadError> {
        for i in 0..buf.len() {
            buf[i] = self.read_u8()?;
        }

        Ok(())
    }
}

impl<R: embedded_hal::serial::Read<u8>> Read for R {
    fn read_u8(&mut self) -> Result<u8, MessageReadError> {
        nb::block!(self.read()).map_err(|_| MessageReadError::Io)
    }
}

/// Replacement for std::io::Write + byteorder::WriteBytesExt in no_std envs
pub trait Write {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError>;
}

impl<W: embedded_hal::serial::Write<u8>> Write for W {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError> {
        for i in 0..buf.len() {
            nb::block!(self.write(buf[i])).map_err(|_| MessageWriteError::Io)?;
        }

        Ok(())
    }
}
