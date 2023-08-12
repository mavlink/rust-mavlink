use crate::connection::MavConnection;
use crate::error::{MessageReadError, MessageWriteError};
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use serialport;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io;
use std::sync::Mutex;
use std::time::Duration;

/// Serial MAVLINK connection

pub fn open(settings: &str) -> io::Result<SerialConnection> {
    let settings_toks: Vec<&str> = settings.split(':').collect();
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
    let baud_rate = baud_opt.unwrap();
    let port_builder = serialport::new(port_name, baud_rate)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(Duration::from_secs(1));

    let reader_side = port_builder.open()?;
    let writer_side = reader_side
        .try_clone()
        .expect("Could not open the serial port in full duplex.");

    Ok(SerialConnection {
        reader: Mutex::new(reader_side),
        writer: Mutex::new(writer_side),
        sequence: Mutex::new(0),
        protocol_version: MavlinkVersion::V2,
        id: settings.to_string(),
    })
}

pub struct SerialConnection {
    pub(crate) reader: Mutex<Box<dyn SerialPort>>,
    pub(crate) writer: Mutex<Box<dyn SerialPort>>,
    pub(crate) sequence: Mutex<u8>,
    pub(crate) protocol_version: MavlinkVersion,
    pub(crate) id: String,
}

impl<M: Message> MavConnection<M> for SerialConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut port = self.reader.lock().unwrap();

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

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError> {
        let mut port = self.writer.lock().unwrap();
        let mut sequence = self.sequence.lock().unwrap();

        let header = MavHeader {
            sequence: *sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        *sequence = sequence.wrapping_add(1);

        write_versioned_msg(&mut *port, self.protocol_version, header, data)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }
}
