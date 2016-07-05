extern crate byteorder;
extern crate crc16;
#[macro_use] extern crate log;

use std::io;
use byteorder::{ ByteOrder, LittleEndian, ReadBytesExt, WriteBytesExt };
use std::io::prelude::*;

mod connection;
pub use connection::{ MavConnection, Tcp, Udp, connect };

/// The MAVLink common message set
///
/// https://pixhawk.ethz.ch/mavlink/
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod common {
    include!(concat!(env!("OUT_DIR"), "/common.rs"));
}

use common::MavMessage;

const MAV_STX: u8 = 0xFE;

/// Metadata from a MAVLink packet header
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
        
        trace!("Packet id={}, len={}", msgid, len);
                    
        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();
        crc_calc.update(&[len as u8, seq, sysid, compid, msgid]);
        crc_calc.update(payload);
        crc_calc.update(&[MavMessage::extra_crc(msgid)]);
        if crc_calc.get() != crc {
            trace!("CRC failure");
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
    crc.update(header);
    crc.update(&payload[..]);
    crc.update(&[MavMessage::extra_crc(msgid)]);
    
    try!(w.write_all(header));
    try!(w.write_all(&payload[..]));
    try!(w.write_u16::<LittleEndian>(crc.get()));

    Ok(())
}

/// Create a heartbeat message
pub fn heartbeat_message() -> common::MavMessage {
    common::MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: 6,
        autopilot: 8,
        base_mode: 0,
        system_status: 0,
        mavlink_version: 0x3,
    })
}

/// Create a message requesting the parameters list
pub fn request_parameters() -> common::MavMessage {
    common::MavMessage::PARAM_REQUEST_LIST(common::PARAM_REQUEST_LIST_DATA {
        target_system: 0,
        target_component: 0,
    })
}

/// Create a message enabling data streaming
pub fn request_stream() -> common::MavMessage {
    common::MavMessage::REQUEST_DATA_STREAM(common::REQUEST_DATA_STREAM_DATA {
        target_system: 0,
        target_component: 0,
        req_stream_id: 0,
        req_message_rate: 10,
        start_stop: 1,
    })
}
