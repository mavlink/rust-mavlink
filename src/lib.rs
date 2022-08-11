//! The MAVLink message set.
//!
//! # Message sets and the `Message` trait
//! Each message set has its own module with corresponding data types, including a `MavMessage` enum
//! that represents all possible messages in that message set. The [`Message`] trait is used to
//! represent messages in an abstract way, and each `MavMessage` enum implements this trait (for
//! example, [`ardupilotmega::MavMessage`]). This is then monomorphized to the specific message
//! set you are using in your application at compile-time via type parameters. If you expect
//! ArduPilotMega-flavored messages, then you will need a `MavConnection<ardupilotmega::MavMessage>`
//! and you will receive `ardupilotmega::MavMessage`s from it.
//!
//! Some message sets include others. For example, all message sets except `common` include the
//! common message set. This is represented with extra values in the `MavMessage` enum: a message
//! in the common message set received on an ArduPilotMega connection will be an
//! `ardupilotmega::MavMessage::common(common::MavMessage)`.
//!
//! Please note that if you want to enable a given message set, you must also enable the
//! feature for the message sets that it includes. For example, you cannot use the `ardupilotmega`
//! feature without also using the `uavionix` and `icarous` features.
//!
#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::vec;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

use core::result::Result;

#[cfg(feature = "std")]
use std::io::{Read, Write};

extern crate byteorder;
use byteorder::LittleEndian;
#[cfg(feature = "std")]
use byteorder::{ReadBytesExt, WriteBytesExt};

#[cfg(feature = "std")]
mod connection;
#[cfg(feature = "std")]
pub use self::connection::{connect, MavConnection};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

extern crate bytes;
use crate::error::ParserError;
use bytes::{Buf, BytesMut};

extern crate bitflags;
extern crate num_derive;
extern crate num_traits;

use crc_any::CRCu16;

// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub mod error;

#[cfg(feature = "embedded")]
mod embedded;
#[cfg(feature = "embedded")]
use embedded::{Read, Write};

pub trait Message
where
    Self: Sized,
{
    fn message_id(&self) -> u32;
    fn message_name(&self) -> &'static str;
    fn ser(&self) -> Vec<u8>;

    fn parse(
        version: MavlinkVersion,
        msgid: u32,
        payload: &[u8],
    ) -> Result<Self, error::ParserError>;

    fn message_id_from_name(name: &str) -> Result<u32, &'static str>;
    fn default_message_from_id(id: u32) -> Result<Self, &'static str>;
    fn extra_crc(id: u32) -> u8;
}

/// Metadata from a MAVLink packet header
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavHeader {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
}

/// Versions of the Mavlink protocol that we support
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
pub enum MavlinkVersion {
    V1,
    V2,
}

/// Message framing marker for mavlink v1
pub const MAV_STX: u8 = 0xFE;

/// Message framing marker for mavlink v2
pub const MAV_STX_V2: u8 = 0xFD;

/// Return a default GCS header, seq is replaced by the connector
/// so it can be ignored. Set `component_id` to your desired component ID.
impl Default for MavHeader {
    fn default() -> Self {
        Self {
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
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct MavFrame<M: Message> {
    pub header: MavHeader,
    pub msg: M,
    pub protocol_version: MavlinkVersion,
}

impl<M: Message> MavFrame<M> {
    /// Create a new frame with given message
    //    pub fn new(msg: MavMessage) -> MavFrame {
    //        MavFrame {
    //            header: MavHeader::get_default_header(),
    //            msg
    //        }
    //    }

    /// Serialize MavFrame into a vector, so it can be sent over a socket, for example.
    pub fn ser(&self) -> Vec<u8> {
        let mut v = vec![];

        // serialize header
        v.push(self.header.system_id);
        v.push(self.header.component_id);
        v.push(self.header.sequence);

        // message id
        match self.protocol_version {
            MavlinkVersion::V2 => {
                let bytes: [u8; 4] = self.msg.message_id().to_le_bytes();
                v.extend_from_slice(&bytes);
            }
            MavlinkVersion::V1 => {
                v.push(self.msg.message_id() as u8); //TODO check
            }
        }

        // serialize message
        v.append(&mut self.msg.ser());

        v
    }

    /// Deserialize MavFrame from a slice that has been received from, for example, a socket.
    pub fn deser(version: MavlinkVersion, input: &[u8]) -> Result<Self, ParserError> {
        let mut buf = BytesMut::from(input);

        let system_id = buf.get_u8();
        let component_id = buf.get_u8();
        let sequence = buf.get_u8();
        let header = MavHeader {
            system_id,
            component_id,
            sequence,
        };

        let msg_id = match version {
            MavlinkVersion::V2 => buf.get_u32_le(),
            MavlinkVersion::V1 => buf.get_u8() as u32,
        };

        match M::parse(version, msg_id, &buf.into_iter().collect::<Vec<u8>>()) {
            Ok(msg) => Ok(MavFrame {
                header,
                msg,
                protocol_version: version,
            }),
            Err(err) => Err(err),
        }
    }

    /// Return the frame header
    pub fn header(&self) -> MavHeader {
        self.header
    }
}

pub fn read_versioned_msg<M: Message, R: Read>(
    r: &mut R,
    version: MavlinkVersion,
) -> Result<(MavHeader, M), error::MessageReadError> {
    match version {
        MavlinkVersion::V2 => read_v2_msg(r),
        MavlinkVersion::V1 => read_v1_msg(r),
    }
}

/// Read a MAVLink v1  message from a Read stream.
pub fn read_v1_msg<M: Message, R: Read>(
    r: &mut R,
) -> Result<(MavHeader, M), error::MessageReadError> {
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

        let mut crc_calc = CRCu16::crc16mcrf4cc();
        crc_calc.digest(&[len as u8, seq, sysid, compid, msgid]);
        crc_calc.digest(payload);
        crc_calc.digest(&[M::extra_crc(msgid.into())]);
        let recvd_crc = crc_calc.get_crc();
        if recvd_crc != crc {
            // bad crc: ignore message
            //println!("msg id {} len {} , crc got {} expected {}", msgid, len, crc, recvd_crc );
            continue;
        }

        return M::parse(MavlinkVersion::V1, msgid as u32, payload)
            .map(|msg| {
                (
                    MavHeader {
                        sequence: seq,
                        system_id: sysid,
                        component_id: compid,
                    },
                    msg,
                )
            })
            .map_err(|err| err.into());
    }
}

const MAVLINK_IFLAG_SIGNED: u8 = 0x01;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
// Follow protocol definition: https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format
pub struct MAVLinkV2MessageRaw {
    pub start: u8, // = MAV_STX_V2
    pub payload_length: u8,
    pub incompatibility_flags: u8,
    pub compatibility_flags: u8,
    pub sequence: u8,
    pub system_id: u8,
    pub component_id: u8,
    pub message_id: u32,
    pub payload_buffer: [u8; 255],
    pub checksum: u16,
}

impl MAVLinkV2MessageRaw {
    pub fn payload(&self) -> &[u8] {
        return &self.payload_buffer[..self.payload_length as usize];
    }

    pub fn mut_payload(&mut self) -> &mut [u8] {
        return &mut self.payload_buffer[..self.payload_length as usize];
    }

    pub fn calculate_crc<M: Message>(&self) -> u16 {
        let mut crc_calculator = CRCu16::crc16mcrf4cc();
        crc_calculator.digest(&[
            self.payload_length,
            self.incompatibility_flags,
            self.compatibility_flags,
            self.sequence,
            self.system_id,
            self.component_id,
        ]);
        let payload = self.payload();
        crc_calculator.digest(&self.message_id.to_le_bytes()[..3]);
        crc_calculator.digest(payload);
        let extra_crc = M::extra_crc(self.message_id);

        crc_calculator.digest(&[extra_crc]);
        return crc_calculator.get_crc();
    }

    pub fn has_valid_crc<M: Message>(&self) -> bool {
        return self.checksum == self.calculate_crc::<M>();
    }
}

/// Return a raw buffer with the mavlink message
/// V2 maximum size is 280 bytes: https://mavlink.io/en/guide/serialization.html
pub fn read_v2_raw_message<R: Read>(
    reader: &mut R,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        if reader.read_u8()? == MAV_STX_V2 {
            break;
        }
    }

    let mut message = MAVLinkV2MessageRaw {
        start: MAV_STX_V2,
        payload_length: reader.read_u8()?,
        incompatibility_flags: reader.read_u8()?,
        compatibility_flags: reader.read_u8()?,
        sequence: reader.read_u8()?,
        system_id: reader.read_u8()?,
        component_id: reader.read_u8()?,
        message_id: 0,
        payload_buffer: [0; 255],
        checksum: 0,
    };

    message.message_id =
        u32::from_le_bytes([reader.read_u8()?, reader.read_u8()?, reader.read_u8()?, 0]);

    reader.read_exact(message.mut_payload())?;

    message.checksum = reader.read_u16::<LittleEndian>()?;

    // Signature to ensure the link is tamper-proof.
    if (message.incompatibility_flags & 0x01) == MAVLINK_IFLAG_SIGNED {
        let mut sign = [0; 13];
        reader.read_exact(&mut sign)?;
    }

    return Ok(message);
}

/// Read a MAVLink v2  message from a Read stream.
pub fn read_v2_msg<M: Message, R: Read>(
    read: &mut R,
) -> Result<(MavHeader, M), error::MessageReadError> {
    loop {
        let message = read_v2_raw_message(read)?;
        if !message.has_valid_crc::<M>() {
            // bad crc: ignore message
            continue;
        }

        return M::parse(MavlinkVersion::V2, message.message_id, message.payload())
            .map(|msg| {
                (
                    MavHeader {
                        sequence: message.sequence,
                        system_id: message.system_id,
                        component_id: message.component_id,
                    },
                    msg,
                )
            })
            .map_err(|err| err.into());
    }
}

/// Write a message using the given mavlink version
pub fn write_versioned_msg<M: Message, W: Write>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg(w, header, data),
        MavlinkVersion::V1 => write_v1_msg(w, header, data),
    }
}

/// Write a MAVLink v2 message to a Write stream.
pub fn write_v2_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
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
        ((msgid & 0x00FF00) >> 8) as u8,
        ((msgid & 0xFF0000) >> 16) as u8,
    ];

    //    println!("write H: {:?}",header );

    let mut crc = CRCu16::crc16mcrf4cc();
    crc.digest(&header[1..]);
    //    let header_crc = crc.get_crc();
    crc.digest(&payload[..]);
    //    let base_crc = crc.get_crc();
    let extra_crc = M::extra_crc(msgid);
    //    println!("write header_crc: {} base_crc: {} extra_crc: {}",
    //             header_crc, base_crc, extra_crc);
    crc.digest(&[extra_crc]);

    let crc_bytes = crc.get_crc().to_le_bytes();

    let len = payload.len() + header.len() + crc_bytes.len();

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_all(&crc_bytes)?;

    Ok(len)
}

/// Write a MAVLink v1 message to a Write stream.
pub fn write_v1_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let msgid = data.message_id();
    let payload = data.ser();

    let header = &[
        MAV_STX,
        payload.len() as u8,
        header.sequence,
        header.system_id,
        header.component_id,
        msgid as u8,
    ];

    let mut crc = CRCu16::crc16mcrf4cc();
    crc.digest(&header[1..]);
    crc.digest(&payload[..]);
    crc.digest(&[M::extra_crc(msgid)]);

    let crc_bytes = crc.get_crc().to_le_bytes();

    let len = payload.len() + header.len() + crc_bytes.len();

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_all(&crc_bytes)?;

    Ok(len)
}
