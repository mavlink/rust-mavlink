//! Serial MAVLINK connection

use crate::connection::MavConnection;
use crate::peek_reader::PeekReader;
use crate::{MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::io;
use std::sync::Mutex;

use crate::error::{MessageReadError, MessageWriteError};
use serial::{prelude::*, SystemPort};

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg, write_versioned_msg};
#[cfg(feature = "signing")]
use crate::{read_versioned_msg_signed, write_versioned_msg_signed, SigningConfig, SigningData};

pub fn open(settings: &str) -> io::Result<SerialConnection> {
    let settings_toks: Vec<&str> = settings.split(':').collect();
    if settings_toks.len() < 2 {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Incomplete port settings",
        ));
    }

    let Ok(baud) = settings_toks[1]
        .parse::<usize>()
        .map(serial::core::BaudRate::from_speed)
    else {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Invalid baud rate",
        ));
    };

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
        port: Mutex::new(PeekReader::new(port)),
        sequence: Mutex::new(0),
        protocol_version: MavlinkVersion::V2,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub struct SerialConnection {
    port: Mutex<PeekReader<SystemPort>>,
    sequence: Mutex<u8>,
    protocol_version: MavlinkVersion,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

impl<M: Message> MavConnection<M> for SerialConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut port = self.port.lock().unwrap();
        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg(port.deref_mut(), self.protocol_version);
            #[cfg(feature = "signing")]
            let result = read_versioned_msg_signed(
                port.deref_mut(),
                self.protocol_version,
                self.signing_data.as_ref(),
            );
            match result {
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
        let mut port = self.port.lock().unwrap();
        let mut sequence = self.sequence.lock().unwrap();

        let header = MavHeader {
            sequence: *sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        *sequence = sequence.wrapping_add(1);

        #[cfg(not(feature = "signing"))]
        let result = write_versioned_msg(port.reader_mut(), self.protocol_version, header, data);
        #[cfg(feature = "signing")]
        let result = write_versioned_msg_signed(
            port.reader_mut(),
            self.protocol_version,
            header,
            data,
            self.signing_data.as_ref(),
        );
        result
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(|cfg| SigningData::from_config(cfg))
    }
}
