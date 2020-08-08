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
// TODO: a parser for no_std environments
#![cfg_attr(not(feature = "std"), feature(alloc))]
#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;

use core::result::Result;

#[cfg(feature = "std")]
use std::io::{Read, Write};

#[cfg(feature = "std")]
extern crate byteorder;
#[cfg(feature = "std")]
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

#[cfg(feature = "std")]
mod connection;
#[cfg(feature = "std")]
pub use self::connection::{connect, MavConnection};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

extern crate bytes;
use crate::error::ParserError;
use bytes::{Buf, Bytes, IntoBuf};

extern crate bitflags;
extern crate num_derive;
extern crate num_traits;

use crc_any::CRCu16;

// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub mod error;

pub trait Message
where
    Self: Sized,
{
    fn message_id(&self) -> u32;
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
        let mut buf = Bytes::from(input).into_buf();

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

        match M::parse(version, msg_id, &buf.collect::<Vec<u8>>()) {
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

trait MavlinkPacketFormat {
    fn checksum(&self) -> u16;

    fn set_checksum(&mut self, value: u16);

    fn calc_checksum<M: Message>(&self) -> u16;

    fn update_checksum<M: Message>(&mut self) {
        self.set_checksum(self.calc_checksum::<M>());
    }

    fn validate_checksum<M: Message>(&self) -> bool {
        return self.checksum() == self.calc_checksum::<M>();
    }
}

/// MAVLink V1 packet format: https://mavlink.io/en/guide/serialization.html#v1_packet_format
struct MavlinkV1PacketFormat {
    pub magic: u8,
    pub len: u8,
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub msgid: u8,
    pub payload: Vec<u8>,
    pub checksum: u16,
}

impl Default for MavlinkV1PacketFormat {
    fn default() -> Self {
        MavlinkV1PacketFormat {
            magic: MAV_STX,
            ..Default::default()
        }
    }
}

impl MavlinkPacketFormat for MavlinkV1PacketFormat {
    fn checksum(&self) -> u16 {
        return self.checksum;
    }

    fn set_checksum(&mut self, value: u16){
        self.checksum = value;
    }

    fn calc_checksum<M: Message>(&self) -> u16 {
        let mut crc_calculator = CRCu16::crc16mcrf4cc();
        crc_calculator.digest(&[self.len, self.seq, self.sysid, self.compid, self.msgid]);
        crc_calculator.digest(&self.payload);
        crc_calculator.digest(&[M::extra_crc(self.msgid.into())]);

        return crc_calculator.get_crc();
    }
}

/// MAVLink V2 packet format: https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format
struct MavlinkV2PacketFormat {
    pub magic: u8,
    pub len: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub msgid: [u8; 3],
    pub payload: Vec<u8>,
    pub checksum: u16,
    pub signature: [u8; 13],
}

impl Default for MavlinkV2PacketFormat {
    fn default() -> Self {
        MavlinkV2PacketFormat {
            magic: MAV_STX_V2,
            ..Default::default()
        }
    }
}

impl MavlinkPacketFormat for MavlinkV2PacketFormat {
    fn checksum(&self) -> u16 {
        return self.checksum;
    }

    fn set_checksum(&mut self, value: u16){
        self.checksum = value;
    }

    fn calc_checksum<M: Message>(&self) -> u16 {
        let mut crc_calculator = CRCu16::crc16mcrf4cc();
        crc_calculator.digest(&[
            self.len,
            self.incompat_flags,
            self.compat_flags,
            self.seq,
            self.sysid,
            self.compid,
        ]);
        crc_calculator.digest(&self.msgid);
        crc_calculator.digest(&self.payload);

        let mut msgid_buffer = [0 as u8; 4];
        msgid_buffer[..3].copy_from_slice(&self.msgid);
        let msgid = u32::from_le_bytes(msgid_buffer);

        crc_calculator.digest(&[M::extra_crc(msgid)]);
        return crc_calculator.get_crc();
    }
}

trait MavlinkParser {
    fn version(&self) -> MavlinkVersion;

    fn read<M: Message, R: Read>(
        &mut self,
        reader: &mut R,
    ) -> Result<(MavHeader, M), error::MessageReadError>;
}

enum MavlinkV1ParserState {
    Magic,
    Len,
    Seq,
    SysId,
    CompId,
    MsgId,
    Payload,
    Crc1,
    Crc2,
    Done,
}

struct MavlinkV1Parser {
    format: MavlinkV1PacketFormat,
    state: MavlinkV1ParserState,
    payload_parsed: u8,
}

impl MavlinkV1Parser {
    pub fn new() -> Self {
        MavlinkV1Parser {
            format: Default::default(),
            state: MavlinkV1ParserState::Len,
            payload_parsed: 0,
        }
    }
}

enum MavlinkV2ParserState {
    Magic,
    Len,
    IncompatFlags,
    CompatFlags,
    Seq,
    SysId,
    CompId,
    MsgId1,
    MsgId2,
    MsgId3,
    Payload,
    Crc1,
    Crc2,
    Signature,
    Done,
}

struct MavlinkV2Parser {
    format: MavlinkV2PacketFormat,
    state: MavlinkV2ParserState,
    payload_parsed: u8,
    signature_parsed: u8,
}

impl Default for MavlinkV2Parser {
    fn default() -> Self {
        MavlinkV2Parser {
            format: Default::default(),
            state: MavlinkV2ParserState::Len,
            payload_parsed: 0,
            signature_parsed: 0,
        }
    }
}

impl MavlinkParser for MavlinkV1Parser {
    fn version(&self) -> MavlinkVersion {
        MavlinkVersion::V1
    }

    fn read<M: Message, R: Read>(
        &mut self,
        reader: &mut R,
    ) -> Result<(MavHeader, M), error::MessageReadError> {
        loop {
            let mut buffer = [0; 1];
            let size = match reader.read(&mut buffer) {
                Ok(size) => size,
                _ => {
                    return Err(error::MessageReadError::Io(std::io::Error::new(
                        std::io::ErrorKind::TimedOut,
                        "No data avaiable.",
                    )));
                }
            };

            use MavlinkV1ParserState::*;
            match self.state {
                Magic => {
                    if buffer[0] == self.format.magic {
                        self.state = Len;
                    }
                }
                Len => {
                    self.format.len = buffer[0];
                    self.format.payload = vec![0; self.format.len as usize];
                    self.state = Seq;
                }
                Seq => {
                    self.format.seq = buffer[0];
                    self.state = SysId;
                }
                SysId => {
                    self.format.sysid = buffer[0];
                    self.state = CompId;
                }
                CompId => {
                    self.format.compid = buffer[0];
                    self.state = MsgId;
                }
                MsgId => {
                    self.format.msgid = buffer[0];
                    self.state = Payload;
                }
                Payload => {
                    self.format.payload[self.payload_parsed as usize] = buffer[0];
                    self.payload_parsed += 1;
                    if self.payload_parsed == self.format.len {
                        self.state = Crc1;
                    }
                }
                Crc1 => {
                    let mut value = self.format.checksum.to_le_bytes();
                    value[0] = buffer[0];
                    self.format.checksum = u16::from_le_bytes(value);
                    self.state = Crc2;
                }
                Crc2 => {
                    let mut value = self.format.checksum.to_le_bytes();
                    value[1] = buffer[0];
                    self.format.checksum = u16::from_le_bytes(value);
                    self.state = Done;

                    if self.format.validate_checksum::<M>() {
                        return M::parse(
                            self.version(),
                            self.format.msgid as u32,
                            &self.format.payload,
                        )
                        .map(|msg| {
                            (
                                MavHeader {
                                    sequence: self.format.seq,
                                    system_id: self.format.sysid,
                                    component_id: self.format.compid,
                                },
                                msg,
                            )
                        })
                        .map_err(|err| err.into());
                    }

                    return Err(error::MessageReadError::Parse(ParserError::WrongCrc {
                        expected: self.format.calc_checksum::<M>(),
                        received: self.format.checksum,
                    }));
                }
                Done => {
                    self.state = Magic;
                }
            }
        }
    }
}

/*
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

/// Read a MAVLink v2  message from a Read stream.
pub fn read_v2_msg<M: Message, R: Read>(
    r: &mut R,
) -> Result<(MavHeader, M), error::MessageReadError> {
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

        let mut msgid_buf = [0; 4];
        msgid_buf[0] = r.read_u8()?;
        msgid_buf[1] = r.read_u8()?;
        msgid_buf[2] = r.read_u8()?;

        let header_buf = &[
            payload_len as u8,
            incompat_flags,
            compat_flags,
            seq,
            sysid,
            compid,
            msgid_buf[0],
            msgid_buf[1],
            msgid_buf[2],
        ];

        let msgid: u32 = u32::from_le_bytes(msgid_buf);
        //        println!("Got msgid: {}", msgid);

        //provide a buffer that is the maximum payload size
        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..payload_len];

        r.read_exact(payload)?;

        let crc = r.read_u16::<LittleEndian>()?;

        if (incompat_flags & 0x01) == MAVLINK_IFLAG_SIGNED {
            let mut sign = [0; 13];
            r.read_exact(&mut sign)?;
        }

        let mut crc_calc = CRCu16::crc16mcrf4cc();
        crc_calc.digest(header_buf);
        crc_calc.digest(payload);
        let extra_crc = M::extra_crc(msgid);

        crc_calc.digest(&[extra_crc]);
        let recvd_crc = crc_calc.get_crc();
        if recvd_crc != crc {
            // bad crc: ignore message
            // println!("msg id {} payload_len {} , crc got {} expected {}", msgid, payload_len, crc, recvd_crc );
            continue;
        }

        return M::parse(MavlinkVersion::V2, msgid, payload)
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
*/

/// Write a message using the given mavlink version
pub fn write_versioned_msg<M: Message, W: Write>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> std::io::Result<()> {
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
) -> std::io::Result<()> {
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

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_u16::<LittleEndian>(crc.get_crc())?;

    Ok(())
}

/// Write a MAVLink v1 message to a Write stream.
pub fn write_v1_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> std::io::Result<()> {
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

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_u16::<LittleEndian>(crc.get_crc())?;

    Ok(())
}
