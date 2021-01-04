extern crate serial;

use crate::connection::MavConnection;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use std::io::{self};
use std::sync::Mutex;

//TODO why is this import so hairy?
use crate::connection::direct_serial::serial::prelude::*;
use crate::error::{MessageReadError, MessageWriteError};

/// Serial MAVLINK connection

pub fn open(settings: &str) -> io::Result<SerialConnection> {
    let settings_toks: Vec<&str> = settings.split(":").collect();
    if settings_toks.len() < 2 {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Incomplete port settings",
        ));
    }

    let baud_opt = settings_toks[1].parse::<usize>();
    if baud_opt.is_err() {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Invalid baud rate",
        ));
    }

    let baud = serial::core::BaudRate::from_speed(baud_opt.unwrap());

    let settings = serial::core::PortSettings {
        baud_rate: baud,
        char_size: serial::Bits8,
        parity: serial::ParityNone,
        stop_bits: serial::Stop1,
        flow_control: serial::FlowNone,
    };

    let port_name = settings_toks[0];
    let mut port = serial::open(port_name)?;
    port.configure(&settings)?;

    Ok(SerialConnection {
        port: Mutex::new(port),
        sequence: Mutex::new(0),
        protocol_version: MavlinkVersion::V2,
    })
}

pub struct SerialConnection {
    port: Mutex<serial::SystemPort>,
    sequence: Mutex<u8>,
    protocol_version: MavlinkVersion,
}

impl<M: Message> MavConnection<M> for SerialConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut port = self.port.lock().unwrap();

        loop {
            match read_versioned_msg(&mut *port, self.protocol_version) {
                ok @ Ok(..) => {
                    return ok;
                }
                Err(MessageReadError::Io(e)) => {
                    if e.kind() == io::ErrorKind::UnexpectedEof {
                        return Err(MessageReadError::Io(e));
                    }
                }
                _ => {}
            }
        }
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<(), MessageWriteError> {
        let mut port = self.port.lock().unwrap();
        let mut sequence = self.sequence.lock().unwrap();

        let header = MavHeader {
            sequence: *sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        *sequence = sequence.wrapping_add(1);

        write_versioned_msg(&mut *port, self.protocol_version, header, data)?;
        Ok(())
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }
}
