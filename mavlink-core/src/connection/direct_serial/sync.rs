//! Serial MAVLINK connection

use crate::Connectable;
use crate::connection::{Connection, MavConnection};
use crate::connection_shared::{
    ConnectionState, next_atomic_send_header, read_message, read_raw_message, write_message,
};
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::{MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use core::sync::atomic::AtomicU8;
use std::io::{self, BufReader};
use std::sync::Mutex;

use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

use super::config::SerialConfig;

pub struct SerialConnection {
    // Separate ports for reading and writing as it's safe to use concurrently.
    // See the official ref: https://github.com/serialport/serialport-rs/blob/321f85e1886eaa1302aef8a600a631bc1c88703a/examples/duplex.rs
    read_port: Mutex<PeekReader<BufReader<Box<dyn SerialPort>>>>,
    write_port: Mutex<Box<dyn SerialPort>>,
    sequence: AtomicU8,
    state: ConnectionState,
}

impl<M: Message> MavConnection<M> for SerialConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut port = self.read_port.lock().unwrap();

        loop {
            let result = read_message::<M, _>(port.deref_mut(), &self.state);
            match result {
                ok @ Ok(..) => {
                    return ok;
                }
                Err(MessageReadError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    return Err(MessageReadError::Io(e));
                }
                _ => {}
            }
        }
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        let mut port = self.read_port.lock().unwrap();

        loop {
            let result = read_raw_message::<M, _>(port.deref_mut(), &self.state);
            match result {
                ok @ Ok(..) => {
                    return ok;
                }
                Err(MessageReadError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    return Err(MessageReadError::Io(e));
                }
                _ => {}
            }
        }
    }

    fn try_recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut port = self.read_port.lock().unwrap();
        read_message::<M, _>(port.deref_mut(), &self.state)
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError> {
        let mut port = self.write_port.lock().unwrap();

        let header = next_atomic_send_header(&self.sequence, header);
        write_message(port.deref_mut(), &self.state, header, data)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.state.set_protocol_version(version);
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.state.protocol_version()
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.state.set_allow_recv_any_version(allow);
    }

    fn allow_recv_any_version(&self) -> bool {
        self.state.allow_recv_any_version()
    }

    #[cfg(feature = "mav2-message-signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.state.setup_signing(signing_data);
    }
}

impl Connectable for SerialConfig {
    fn connect<M: Message>(&self) -> io::Result<Connection<M>> {
        let read_port = serialport::new(&self.port_name, self.baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .open()?;

        let write_port = read_port.try_clone()?;

        let read_buffer_capacity = self.buffer_capacity();
        let buf_reader = BufReader::with_capacity(read_buffer_capacity, read_port);

        Ok(SerialConnection {
            read_port: Mutex::new(PeekReader::new(buf_reader)),
            write_port: Mutex::new(write_port),
            sequence: AtomicU8::new(0),
            state: ConnectionState::new(),
        }
        .into())
    }
}
