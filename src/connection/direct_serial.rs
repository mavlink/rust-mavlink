
extern crate serial;

use std::sync::Mutex;
use std::io::{self};
use crate::connection::MavConnection;
use crate::common::MavMessage;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion};

//TODO why is this import so hairy?
use crate::connection::direct_serial::serial::prelude::*;


/// Serial MAVLINK connection


pub fn open(settings: &str) -> io::Result<SerialConnection> {
    let settings_toks: Vec<&str> = settings.split(":").collect();
    if settings_toks.len() < 2 {
        return Err(
            io::Error::new(
                io::ErrorKind::AddrNotAvailable,
                "Incomplete port settings",
            ))
    }

    let baud_opt = settings_toks[1].parse::<usize>();
    if baud_opt.is_err() {
        return Err(
            io::Error::new(
                io::ErrorKind::AddrNotAvailable,
                "Invalid baud rate",
            ))
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
    let mut port  = serial::open(port_name)?;
    port.configure(&settings)?;

    Ok(SerialConnection {
        port: Mutex::new(port),
        sequence: Mutex::new(0),
        protocol_version: MavlinkVersion::V2
    })

}



pub struct SerialConnection {
    port: Mutex<serial::SystemPort>,
    sequence: Mutex<u8>,
    protocol_version: MavlinkVersion,
}

impl MavConnection for SerialConnection {
    fn recv(&self) -> io::Result<(MavHeader, MavMessage)> {
        let mut port = self.port.lock().unwrap();

        loop {
            match read_versioned_msg(&mut *port, self.protocol_version) {
                Ok((h, m)) => {
                    return Ok( (h,m) );
                }
                Err(e) => {
                    //println!("{:?}",e);
                    match e.kind() {
                        io::ErrorKind::UnexpectedEof => {
                            return Err(e);
                        }
                        _ => {},
                    }
                }
            }
        }
    }

    fn send(&self, header: &MavHeader, data: &MavMessage) -> io::Result<()> {
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
