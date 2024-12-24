use crate::error::*;

#[cfg(all(feature = "embedded", feature = "embedded-hal-02"))]
const _: () = panic!("Only one of 'embedded' and 'embedded-hal-02' features can be enabled.");

/// Replacement for std::io::Read + byteorder::ReadBytesExt in no_std envs
pub trait Read {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, MessageReadError> {
        self.read_exact(buf).map(|_| buf.len())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), MessageReadError>;
}

#[cfg(all(feature = "embedded", not(feature = "embedded-hal-02")))]
impl<R: embedded_io::Read> Read for R {
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), MessageReadError> {
        embedded_io::Read::read_exact(self, buf).map_err(|_| MessageReadError::Io)
    }
}

#[cfg(all(feature = "embedded-hal-02", not(feature = "embedded")))]
impl<R: embedded_hal_02::serial::Read<u8>> Read for R {
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), MessageReadError> {
        for byte in buf {
            *byte = nb::block!(self.read()).map_err(|_| MessageReadError::Io)?;
        }

        Ok(())
    }
}

/// Replacement for std::io::Write + byteorder::WriteBytesExt in no_std envs
pub trait Write {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError>;
}

#[cfg(all(feature = "embedded", not(feature = "embedded-hal-02")))]
impl<W: embedded_io::Write> Write for W {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError> {
        embedded_io::Write::write_all(self, buf).map_err(|_| MessageWriteError::Io)
    }
}

#[cfg(all(feature = "embedded-hal-02", not(feature = "embedded")))]
impl<W: embedded_hal_02::serial::Write<u8>> Write for W {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), MessageWriteError> {
        for byte in buf {
            nb::block!(self.write(*byte)).map_err(|_| MessageWriteError::Io)?;
        }

        Ok(())
    }
}
