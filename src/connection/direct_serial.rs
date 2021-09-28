use crate::connection::MavConnection;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use std::io;
use std::sync::Mutex;

use crate::error::{MessageReadError, MessageWriteError};

use serialport::{self, SerialPort};

/// Serial MAVLINK connection

pub fn open(settings: &str) -> io::Result<SerialConnection> {
    let settings_toks: Vec<&str> = settings.split(":").collect();
    if settings_toks.len() < 2 {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Incomplete port settings",
        ));
    }

    let baud_opt = settings_toks[1].parse::<u32>();
    if baud_opt.is_err() {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Invalid baud rate",
        ));
    }

    let port_name = settings_toks[0];
    let builder = serialport::new(port_name, baud_opt.unwrap())
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None);
    let reader = builder.open()?;
    let writer = reader.try_clone()?;

    Ok(SerialConnection {
        reader: Mutex::new(reader),
        writer: Mutex::new(writer),
        sequence: Mutex::new(0),
        protocol_version: MavlinkVersion::V2,
    })
}

pub struct SerialConnection {
    reader: Mutex<Box<dyn SerialPort>>,
    writer: Mutex<Box<dyn SerialPort>>,
    sequence: Mutex<u8>,
    protocol_version: MavlinkVersion,
}

impl<M: Message> MavConnection<M> for SerialConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut reader = self.reader.lock().unwrap();
        loop {
            match read_versioned_msg(&mut *reader, self.protocol_version) {
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

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError> {
        let mut sequence = self.sequence.lock().unwrap();
        let mut writer = self.writer.lock().unwrap();

        let header = MavHeader {
            sequence: *sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        *sequence = sequence.wrapping_add(1);

        write_versioned_msg(&mut *writer, self.protocol_version, header, data)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }
}
