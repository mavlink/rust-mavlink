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

#[cfg(all(feature = "std", feature="mavlink2"))]
use std::mem::transmute;

#[cfg(all(not(feature = "std"), feature="mavlink2"))]
use core::mem::transmute;

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod common {
    include!(concat!(env!("OUT_DIR"), "/common.rs"));
}

/// Encapsulation of all possible Mavlink messages
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

#[allow(dead_code)]
const MAV_STX_V2: u8 = 0xFD;

impl MavHeader {
    /// Return a default GCS header, seq is replaced by the connector
    /// so it can be ignored. Set `component_id` to your desired component ID.
    pub fn get_default_header() -> MavHeader {
        MavHeader {
            system_id: 255,
            component_id: 0,
            sequence: 0,
        }
    }
}

/// Encapsulation of the Mavlink message and the header,
/// important to preserve information about the sender system
/// and component id
#[derive(Debug, Clone)]
pub struct MavFrame {
    pub header: MavHeader,
    pub msg: MavMessage,
}

impl MavFrame {
    /// Create a new frame with given message
    pub fn new(msg: MavMessage) -> MavFrame {
        MavFrame {
            header: MavHeader::get_default_header(),
            msg
        }
    }

    /// Serialize frame into a vector, so it can be send
    /// over a socket for example
    pub fn ser(&self) -> Vec<u8> {
        let mut v = vec![];

        // serialize header
        v.push(self.header.system_id);
        v.push(self.header.component_id);
        v.push(self.header.sequence);

        // message id
        #[cfg(feature="mavlink2")]
        {
            let bytes: [u8; 4] = unsafe { transmute(self.msg.message_id().to_le()) };
            v.extend_from_slice(&bytes);
        }
        #[cfg(not(feature="mavlink2"))]
        v.push(self.msg.message_id());

        // serialize message
        v.append(&mut self.msg.ser());

        v
    }

    /// Deserialize MavFrame from a slice that has been received from
    /// for example a socket.
    pub fn deser(input: &[u8]) -> Option<Self> {
        let mut buf = Bytes::from(input).into_buf();

        let system_id = buf.get_u8();
        let component_id = buf.get_u8();
        let sequence = buf.get_u8();
        let header = MavHeader{system_id,component_id,sequence};

        #[cfg(not(feature="mavlink2"))]
        let msg_id = buf.get_u8();

        #[cfg(feature="mavlink2")]
        let msg_id = buf.get_u32_le();

        if let Some(msg) = MavMessage::parse(msg_id, &buf.collect::<Vec<u8>>()) {
            Some(MavFrame {header, msg})
        } else {
            None
        }
    }

    /// Return the frame header
    pub fn header(&self) -> MavHeader {
        self.header
    }
}

/// Read a MAVLink v1  message from a Read stream.
#[cfg(all(feature = "std", not(feature="mavlink2")))]
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

#[cfg(feature="mavlink2")]
const MAVLINK_IFLAG_SIGNED: u8 = 0x01;

/// Read a MAVLink v2  message from a Read stream.
#[cfg(all(feature = "std", feature="mavlink2"))]
pub fn read<R: Read>(r: &mut R) -> Result<(MavHeader, MavMessage)> {
 loop {
        if r.read_u8()? != MAV_STX_V2 {
            continue;
        }
        println!("Got STX");
        let len = r.read_u8()? as usize;
        println!("Got len: {}", len);
        let incompat_flags = r.read_u8()?;
        println!("Got incompat flags: {}", incompat_flags);
        let _compat_flags = r.read_u8()?;
        println!("Got compat flags: {}", _compat_flags);
        let seq = r.read_u8()?;
        println!("Got seq: {}", seq);
        let sysid = r.read_u8()?;
        println!("Got sysid: {}", sysid);
        let compid = r.read_u8()?;
        println!("Got compid: {}", compid);

        let mut msgid_buf = [0;4];
        msgid_buf[0] = r.read_u8()?;
        msgid_buf[1] = r.read_u8()?;
        msgid_buf[2] = r.read_u8()?;
        println!("Got msgid buf: {:?}", msgid_buf);

        let msgid: u32 = unsafe { transmute(msgid_buf) };
        println!("Got msgid: {}", msgid);

        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..len];
        r.read_exact(payload)?;

        let crc = r.read_u16::<LittleEndian>()?;

        if (incompat_flags & 0x01) == MAVLINK_IFLAG_SIGNED {
            let mut sign = [0;13];
            r.read_exact(&mut sign)?;
        }

        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();
        crc_calc.update(&[len as u8, seq, sysid, compid, msgid_buf[0],msgid_buf[1],msgid_buf[2]]);
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

/// Write a MAVLink v2 message to a Write stream.
#[cfg(all(feature = "std", feature="mavlink2"))]
pub fn write<W: Write>(w: &mut W, header: MavHeader, data: &MavMessage) -> Result<()> {
    let msgid = data.message_id();
    let payload = data.ser();

    let header = &[
        MAV_STX_V2,
        payload.len() as u8,
        header.sequence,
        header.system_id,
        header.component_id,
        msgid as u8,
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

/// Write a MAVLink v1 message to a Write stream.
#[cfg(all(feature = "std", not(feature="mavlink2")))]
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
    #[cfg(all(feature = "std", not(feature="mavlink2")))]
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
    #[cfg(all(feature = "std", not(feature="mavlink2")))]
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

    #[test]
    #[cfg(all(feature = "std", not(feature="mavlink2")))]
    pub fn test_log_file() {
        let path = "test.tlog";
        let mut f = File::open(path).unwrap();

        loop {
            match self::read(&mut f) {
                Ok((_, msg)) => {
                    println!("{:#?}", msg);
                }
                Err(e) => {
                    println!("Error: {:?}", e);
                    match e.kind() {
                        std::io::ErrorKind::UnexpectedEof => {
                            break;
                        },
                        _ => {
                            panic!("Unexpected error");
                        }
                    }
                }
            }
        }
    }
}
