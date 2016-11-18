extern crate byteorder;
extern crate crc16;

use std::io;
use byteorder::{ LittleEndian, ReadBytesExt, WriteBytesExt };
use std::io::prelude::*;

mod connection;
pub use connection::{ MavConnection, Tcp, Udp, connect };

/// The MAVLink combined (common + ardupilotmega) message set
///
/// https://pixhawk.ethz.ch/mavlink/
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod combined {
    include!(concat!(env!("OUT_DIR"), "/combined.rs"));
}

use combined::MavMessage;

const MAV_STX: u8 = 0xFE;

/// Metadata from a MAVLink packet header
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Header {
    pub sequence: u8,
    pub system_id: u8,
    pub component_id: u8,
}

/// Read a MAVLink message from a Read stream.
pub fn read<R: Read>(r: &mut R) -> io::Result<(Header, MavMessage)> {
    loop {
        if try!(r.read_u8()) != MAV_STX {
            continue;
        }
        let len    =  try!(r.read_u8()) as usize;
        let seq    =  try!(r.read_u8());
        let sysid  =  try!(r.read_u8());
        let compid =  try!(r.read_u8());
        let msgid  =  try!(r.read_u8());

        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..len];
        try!(r.read_exact(payload));

        let crc = try!(r.read_u16::<LittleEndian>());

        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();
        crc_calc.update(&[len as u8, seq, sysid, compid, msgid]);
        crc_calc.update(payload);
        crc_calc.update(&[MavMessage::extra_crc(msgid)]);
        if crc_calc.get() != crc {
            continue;
        }

        if let Some(msg) = MavMessage::parse(msgid, payload) {
            return Ok((Header { sequence: seq, system_id: sysid, component_id: compid }, msg));
        }
    }
}

/// Write a MAVLink message to a Write stream.
pub fn write<W: Write>(w: &mut W, header: Header, data: &MavMessage) -> io::Result<()> {
    let msgid = data.message_id();
    let payload = data.serialize();

    let header = &[
        MAV_STX,
        payload.len() as u8,
        header.sequence,
        header.system_id,
        header.component_id,
        msgid,
    ];

    let mut crc = crc16::State::<crc16::MCRF4XX>::new();
    crc.update(&header[1..]);
    crc.update(&payload[..]);
    crc.update(&[MavMessage::extra_crc(msgid)]);

    try!(w.write_all(header));
    try!(w.write_all(&payload[..]));
    try!(w.write_u16::<LittleEndian>(crc.get()));

    Ok(())
}

/// Create a heartbeat message
pub fn heartbeat_message() -> combined::MavMessage {
    combined::MavMessage::HEARTBEAT(combined::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: 6,
        autopilot: 8,
        base_mode: 0,
        system_status: 0,
        mavlink_version: 0x3,
    })
}

/// Create a message requesting the parameters list
pub fn request_parameters() -> combined::MavMessage {
    combined::MavMessage::PARAM_REQUEST_LIST(combined::PARAM_REQUEST_LIST_DATA {
        target_system: 0,
        target_component: 0,
    })
}

/// Create a message enabling data streaming
pub fn request_stream() -> combined::MavMessage {
    combined::MavMessage::REQUEST_DATA_STREAM(combined::REQUEST_DATA_STREAM_DATA {
        target_system: 0,
        target_component: 0,
        req_stream_id: 0,
        req_message_rate: 10,
        start_stop: 1,
    })
}

#[cfg(test)]
mod test_message {
    use super::*;
    pub const HEARTBEAT: &'static[u8] = &[0xfe, 0x09, 0xef, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03, 0xf1, 0xd7];
    pub const HEARTBEAT_HEADER: Header = Header { sequence: 239, system_id: 1, component_id: 1 };
    pub const HEARTBEAT_MSG: combined::HEARTBEAT_DATA = combined::HEARTBEAT_DATA { custom_mode: 5, mavtype: 2, autopilot: 3, base_mode: 89, system_status: 3, mavlink_version: 3 };

    #[test]
    pub fn test_read() {
        let mut r = HEARTBEAT;
        let (header, msg) = read(&mut r).expect("Failed to parse message");

        println!("{:?}, {:?}", header, msg);

        assert_eq!(header, HEARTBEAT_HEADER);

        if let combined::MavMessage::HEARTBEAT(msg) = msg {
            assert_eq!(msg.custom_mode, HEARTBEAT_MSG.custom_mode);
            assert_eq!(msg.mavtype, HEARTBEAT_MSG.mavtype);
            assert_eq!(msg.autopilot, HEARTBEAT_MSG.autopilot);
            assert_eq!(msg.base_mode, HEARTBEAT_MSG.base_mode);
            assert_eq!(msg.system_status, HEARTBEAT_MSG.system_status);
            assert_eq!(msg.mavlink_version, HEARTBEAT_MSG.mavlink_version);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    pub fn test_write() {
        let mut v = vec![];
        write(&mut v, HEARTBEAT_HEADER, &combined::MavMessage::HEARTBEAT(HEARTBEAT_MSG.clone()))
            .expect("Failed to write message");

        assert_eq!(&v[..], HEARTBEAT);
    }

}
