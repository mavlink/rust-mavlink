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
extern crate byteorder;
#[cfg(feature = "std")]
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

#[cfg(feature = "std")]
mod connection;
#[cfg(feature = "std")]
pub use self::connection::{connect, MavConnection};
#[cfg(feature = "serial")]
pub use self::connection::{Serial};
#[cfg(feature = "udp")]
pub use self::connection::{Udp};
#[cfg(feature = "tcp")]
pub use self::connection::{Tcp};


extern crate bytes;
use bytes::{Buf, Bytes, IntoBuf};

#[cfg(all(feature = "std", feature="mavlink2"))]
use std::mem::transmute;

#[cfg(all(not(feature = "std"), feature="mavlink2"))]
use core::mem::transmute;

extern crate num_traits;
extern crate num_derive;
extern crate bitflags;
#[macro_use]

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


/// Message framing marker for mavlink v1
pub const MAV_STX: u8 = 0xFE;

/// Message framing marker for mavlink v2
pub const MAV_STX_V2: u8 = 0xFD;


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
pub fn read_msg<R: Read>(r: &mut R) -> Result<(MavHeader, MavMessage)> {
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
        let recvd_crc = crc_calc.get();
        if recvd_crc != crc {
            println!("msg id {} len {} , crc got {} expected {}", msgid, len, crc, recvd_crc );
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

///
/// Read a MAVLink v2  message from a Read stream.
#[cfg(all(feature = "std", feature="mavlink2"))]
pub fn read_msg<R: Read>(r: &mut R) -> Result<(MavHeader, MavMessage)> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        if r.read_u8()? != MAV_STX_V2 {
            continue;
        }

//        println!("Got STX2");
        let payload_len = r.read_u8()? as usize;
//        println!("Got payload_len: {}", payload_len);
        let incompat_flags = r.read_u8()?;
//        println!("Got incompat flags: {}", incompat_flags);
        let compat_flags = r.read_u8()?;
//        println!("Got compat flags: {}", compat_flags);

        let seq = r.read_u8()?;
//        println!("Got seq: {}", seq);

        let sysid = r.read_u8()?;
//        println!("Got sysid: {}", sysid);

        let compid = r.read_u8()?;
//        println!("Got compid: {}", compid);

        let mut msgid_buf = [0;4];
        msgid_buf[0] = r.read_u8()?;
        msgid_buf[1] = r.read_u8()?;
        msgid_buf[2] = r.read_u8()?;

        let header_buf = &[payload_len as u8,
            incompat_flags, compat_flags,
            seq, sysid, compid,
            msgid_buf[0],msgid_buf[1],msgid_buf[2]];

        let msgid: u32 = unsafe { transmute(msgid_buf) };
//        println!("Got msgid: {}", msgid);

        //provide a buffer that is the maximum payload size
        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..payload_len];

        r.read_exact(payload)?;

        let crc = r.read_u16::<LittleEndian>()?;

        if (incompat_flags & 0x01) == MAVLINK_IFLAG_SIGNED {
            let mut sign = [0;13];
            r.read_exact(&mut sign)?;
        }

        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();
        crc_calc.update(header_buf);
        crc_calc.update(payload);
        let extra_crc = MavMessage::extra_crc(msgid);

        crc_calc.update(&[extra_crc]);
        let recvd_crc = crc_calc.get();
        if recvd_crc != crc {
//            println!("msg id {} payload_len {} , crc got {} expected {}", msgid, payload_len, crc, recvd_crc );
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
        else {
            println!("invalid MavMessage::parse");
        }
    }
}

/// Write a MAVLink v2 message to a Write stream.
#[cfg(all(feature = "std", feature="mavlink2"))]
pub fn write_msg<W: Write>(w: &mut W, header: MavHeader, data: &MavMessage) -> Result<()> {
    let msgid = data.message_id();
    let payload = data.ser();
//    println!("write payload_len : {}", payload.len());

    let header = &[
        MAV_STX_V2,
        payload.len() as u8,
        0, //incompat_flags
        0, //compat_flags
        header.sequence,
        header.system_id,
        header.component_id,
        (msgid & 0x0000FF) as u8,
        ((msgid & 0x00FF00) >> 8) as u8 ,
        ((msgid & 0xFF0000) >> 16) as u8,
    ];

//    println!("write H: {:?}",header );


    let mut crc = crc16::State::<crc16::MCRF4XX>::new();
    crc.update(&header[1..]);
//    let header_crc = crc.get();
    crc.update(&payload[..]);
//    let base_crc = crc.get();
    let extra_crc = MavMessage::extra_crc(msgid);
//    println!("write header_crc: {} base_crc: {} extra_crc: {}",
//             header_crc, base_crc, extra_crc);
    crc.update(&[extra_crc]);

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_u16::<LittleEndian>(crc.get())?;

    Ok(())
}

/// Write a MAVLink v1 message to a Write stream.
#[cfg(all(feature = "std", not(feature="mavlink2")))]
pub fn write_msg<W: Write>(w: &mut W, header: MavHeader, data: &MavMessage) -> Result<()> {
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







