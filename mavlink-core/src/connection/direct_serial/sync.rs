//! Serial MAVLINK connection

use super::config::SerialConfig;

use crate::Dialect;
use crate::MavlinkReader;
use crate::connection::sync::{Connectable, DialectConnection, SyncTransport, Transport};
use serialport::{DataBits, FlowControl, Parity, StopBits};
use std::io;
use std::sync::Mutex;
use std::time::Duration;

#[cfg(windows)]
use serialport::COMPort as NativePort;
#[cfg(unix)]
use serialport::TTYPort as NativePort;

pub struct SerialConnection {
    // Separate ports for reading and writing as it's safe to use concurrently.
    // See the official ref: https://github.com/serialport/serialport-rs/blob/321f85e1886eaa1302aef8a600a631bc1c88703a/examples/duplex.rs
    read_port: Mutex<MavlinkReader<NativePort>>,
    write_port: Mutex<NativePort>,
}

impl SyncTransport for SerialConnection {
    type Reader = NativePort;
    type Writer = NativePort;

    fn reader(&self) -> std::sync::MutexGuard<'_, MavlinkReader<Self::Reader>> {
        self.read_port.lock().unwrap()
    }

    fn lock_writer(&self) -> Option<std::sync::MutexGuard<'_, Self::Writer>> {
        Some(self.write_port.lock().unwrap())
    }

    fn retry_receive(&self, error: &crate::error::MessageReadError) -> bool {
        !matches!(error, crate::error::MessageReadError::Io(error) if error.kind() == io::ErrorKind::UnexpectedEof)
    }
}

impl SerialConfig {
    pub(crate) fn open(&self) -> io::Result<SerialConnection> {
        let read_port = serialport::new(&self.port_name, self.baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(self.timeout.unwrap_or(Duration::from_millis(1)))
            .open_native()?;
        let write_port = read_port.try_clone_native()?;
        Ok(SerialConnection {
            read_port: Mutex::new(MavlinkReader::with_capacity(
                self.buffer_capacity(),
                read_port,
            )),
            write_port: Mutex::new(write_port),
        })
    }
}

impl Connectable for SerialConfig {
    fn connect_with_dialect<D: Dialect + Send + Sync>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        Ok(DialectConnection::new(
            Transport::Serial(self.open()?),
            dialect,
        ))
    }
}
