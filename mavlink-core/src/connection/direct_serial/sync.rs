//! Serial MAVLINK connection

use super::config::SerialConfig;

use crate::connection::sync::{
    Connectable, Connection, ConnectionCore, DialectConnection, SyncTransport,
};
use crate::connection_shared::next_atomic_send_header;
use crate::peek_reader::PeekReader;
use crate::{Dialect, MavHeader};
use core::sync::atomic::AtomicU8;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, BufReader};
use std::sync::Mutex;
use std::time::Duration;

pub struct SerialConnection {
    // Separate ports for reading and writing as it's safe to use concurrently.
    // See the official ref: https://github.com/serialport/serialport-rs/blob/321f85e1886eaa1302aef8a600a631bc1c88703a/examples/duplex.rs
    read_port: Mutex<PeekReader<BufReader<Box<dyn SerialPort>>>>,
    write_port: Mutex<Box<dyn SerialPort>>,
    sequence: AtomicU8,
}

impl SyncTransport for SerialConnection {
    type Reader = BufReader<Box<dyn SerialPort>>;
    type Writer = Box<dyn SerialPort>;

    fn reader(&self) -> std::sync::MutexGuard<'_, PeekReader<Self::Reader>> {
        self.read_port.lock().unwrap()
    }

    fn writer(&self) -> Option<std::sync::MutexGuard<'_, Self::Writer>> {
        Some(self.write_port.lock().unwrap())
    }

    fn retry_receive(&self, error: &crate::error::MessageReadError) -> bool {
        !matches!(error, crate::error::MessageReadError::Io(error) if error.kind() == io::ErrorKind::UnexpectedEof)
    }

    fn next_send_header(&self, _: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        next_atomic_send_header(&self.sequence, header)
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
            .open()?;
        let write_port = read_port.try_clone()?;
        Ok(SerialConnection {
            read_port: Mutex::new(PeekReader::new(BufReader::with_capacity(
                self.buffer_capacity(),
                read_port,
            ))),
            write_port: Mutex::new(write_port),
            sequence: AtomicU8::new(0),
        })
    }
}

impl Connectable for SerialConfig {
    fn connect<M: crate::Message>(&self) -> io::Result<Connection<M>> {
        Ok(Box::new(ConnectionCore::new_static(self.open()?)))
    }

    fn connect_with_dialect<D: Dialect + Send + Sync + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        Ok(Box::new(ConnectionCore::new(self.open()?, dialect)))
    }
}
