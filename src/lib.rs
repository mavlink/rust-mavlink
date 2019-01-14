//! The MAVLink common message set
//!
//! TODO: a parser for no_std environments
#![cfg_attr(not(feature = "std"), feature(alloc))]
#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(feature = "std")]
use std::io::{Read, Result, Write};

#[cfg(feature = "std")]
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

#[cfg(feature = "std")]
mod connection;
#[cfg(feature = "std")]
pub use self::connection::{connect, MavConnection, Serial, Tcp, Udp};

use bytes::{Buf, Bytes, IntoBuf};

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod common {
    include!(concat!(env!("OUT_DIR"), "/common.rs"));
}

pub use self::common::MavMessage as MavMessage;

/// Metadata from a MAVLink packet header
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct MavHeader {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
}

#[allow(dead_code)]
const MAV_STX: u8 = 0xFE;

impl MavHeader {
    pub fn get_default_header() -> MavHeader {
        MavHeader {
            system_id: 255,
            component_id: 0,
            sequence: 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MavFrame {
    header: MavHeader,
    msg: MavMessage,
}

impl MavFrame {
    pub fn ser(&self) -> Vec<u8> {
        let mut v = vec![];

        // serialize header
        v.push(self.header.system_id);
        v.push(self.header.component_id);
        v.push(self.header.sequence);

        // message id
        v.push(self.msg.message_id());

        // serialize message
        v.append(&mut self.msg.ser());

        v
    }

    pub fn deser(input: &[u8]) -> Option<Self> {
        let mut buf = Bytes::from(input).into_buf();

        let system_id = buf.get_u8();
        let component_id = buf.get_u8();
        let sequence = buf.get_u8();
        let header = MavHeader{system_id,component_id,sequence};

        let msg_id = buf.get_u8();

        if let Some(msg) = MavMessage::parse(msg_id, &buf.collect::<Vec<u8>>()) {
            Some(MavFrame {header, msg})
        } else {
            None
        }
    }
}

/// Read a MAVLink message from a Read stream.
#[cfg(feature = "std")]
pub fn read<R: Read>(r: &mut R) -> Result<(MavHeader, MavMessage)> {
    loop {
        if r.read_u8()? != MAV_STX {
            continue;
        }
        let len = r.read_u8()? as usize;
        let seq = r.read_u8()?;
        let sysid = r.read_u8()?;
        let compid = r.read_u8()?;
        let msgid = r.read_u8()?;

        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..len];
        r.read_exact(payload)?;

        let crc = r.read_u16::<LittleEndian>()?;

        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();
        crc_calc.update(&[len as u8, seq, sysid, compid, msgid]);
        crc_calc.update(payload);
        crc_calc.update(&[MavMessage::extra_crc(msgid)]);
        if crc_calc.get() != crc {
            continue;
        }

        if let Some(msg) = MavMessage::parse(msgid, payload) {
            return Ok((
                MavHeader {
                    sequence: seq,
                    system_id: sysid,
                    component_id: compid,
                },
                msg,
            ));
        }
    }
}

/// Write a MAVLink message to a Write stream.
#[cfg(feature = "std")]
pub fn write<W: Write>(w: &mut W, header: MavHeader, data: &MavMessage) -> Result<()> {
    let msgid = data.message_id();
    let payload = data.ser();

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

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_u16::<LittleEndian>(crc.get())?;

    Ok(())
}

#[cfg(test)]
mod test_message {
    use super::*;
    pub const HEARTBEAT: &'static [u8] = &[
        0xfe, 0x09, 0xef, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03,
        0xf1, 0xd7,
    ];
    pub const HEARTBEAT_HEADER: MavHeader = MavHeader {
        sequence: 239,
        system_id: 1,
        component_id: 1,
    };

    fn get_heartbeat_msg() -> common::HEARTBEAT_DATA {
        common::HEARTBEAT_DATA {
            custom_mode: 5,
            mavtype: common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        }
    }

    #[test]
    #[cfg(feature = "std")]
    pub fn test_read() {
        let mut r = HEARTBEAT;
        let (header, msg) = read(&mut r).expect("Failed to parse message");

        println!("{:?}, {:?}", header, msg);

        assert_eq!(header, HEARTBEAT_HEADER);
        let heartbeat_msg = get_heartbeat_msg();

        if let common::MavMessage::HEARTBEAT(msg) = msg {
            assert_eq!(msg.custom_mode, heartbeat_msg.custom_mode);
            assert_eq!(msg.mavtype, heartbeat_msg.mavtype);
            assert_eq!(msg.autopilot, heartbeat_msg.autopilot);
            assert_eq!(msg.base_mode, heartbeat_msg.base_mode);
            assert_eq!(msg.system_status, heartbeat_msg.system_status);
            assert_eq!(msg.mavlink_version, heartbeat_msg.mavlink_version);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    #[cfg(feature = "std")]
    pub fn test_write() {
        let mut v = vec![];
        let heartbeat_msg = get_heartbeat_msg();
        write(
            &mut v,
            HEARTBEAT_HEADER,
            &common::MavMessage::HEARTBEAT(heartbeat_msg.clone()),
        )
        .expect("Failed to write message");

        assert_eq!(&v[..], HEARTBEAT);
    }

    #[cfg(feature = "std")]
    use std::fs::File;

    /// Note this test fails, likely because the log file is truncated
    #[test]
    #[ignore]
    #[cfg(feature = "std")]
    pub fn test_log_file() {
        let path = "test.tlog";
        let mut f = File::open(path).unwrap();

        loop {
            match self::read(&mut f) {
                Ok((_, msg)) => {
                    println!("{:#?}", msg);
                }
                Err(e) => {
                    println!("Error: {}", e);
                    break;
                }
            }
        }
    }
}
