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
#![cfg_attr(all(any(docsrs, doc), not(doctest)), feature(doc_auto_cfg))]
#![deny(clippy::all)]
#![warn(clippy::use_self)]

use core::result::Result;

#[cfg(feature = "std")]
use std::io::{Read, Write};

pub mod utils;
#[allow(unused_imports)]
use utils::{remove_trailing_zeroes, RustDefault};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub mod peek_reader;
use peek_reader::PeekReader;

use crate::{bytes::Bytes, error::ParserError};

use crc_any::CRCu16;

pub mod bytes;
pub mod bytes_mut;
#[cfg(feature = "std")]
mod connection;
pub mod error;
#[cfg(feature = "std")]
pub use self::connection::{connect, Connectable, MavConnection};

#[cfg(feature = "tokio-1")]
mod async_connection;
#[cfg(feature = "tokio-1")]
pub use self::async_connection::{connect_async, AsyncConnectable, AsyncMavConnection};

#[cfg(feature = "tokio-1")]
pub mod async_peek_reader;
#[cfg(feature = "tokio-1")]
use async_peek_reader::AsyncPeekReader;

#[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
pub mod embedded;
#[cfg(any(feature = "embedded", feature = "embedded-hal-02"))]
use embedded::{Read, Write};

#[cfg(not(feature = "signing"))]
type SigningData = ();
#[cfg(feature = "signing")]
mod signing;
#[cfg(feature = "signing")]
pub use self::signing::{SigningConfig, SigningData};
#[cfg(feature = "signing")]
use sha2::{Digest, Sha256};

#[cfg(any(feature = "std", feature = "tokio-1"))]
mod connectable;
#[cfg(any(feature = "std", feature = "tokio-1"))]
pub use connectable::{
    ConnectionAddress, FileConnectable, SerialConnectable, TcpConnectable, UdpConnectable,
};

pub const MAX_FRAME_SIZE: usize = 280;

pub trait Message
where
    Self: Sized,
{
    fn message_id(&self) -> u32;
    fn message_name(&self) -> &'static str;

    /// Serialize **Message** into byte slice and return count of bytes written
    fn ser(&self, version: MavlinkVersion, bytes: &mut [u8]) -> usize;

    fn parse(
        version: MavlinkVersion,
        msgid: u32,
        payload: &[u8],
    ) -> Result<Self, error::ParserError>;

    fn message_id_from_name(name: &str) -> Result<u32, &'static str>;
    fn default_message_from_id(id: u32) -> Result<Self, &'static str>;
    fn extra_crc(id: u32) -> u8;
}

pub trait MessageData: Sized {
    type Message: Message;

    const ID: u32;
    const NAME: &'static str;
    const EXTRA_CRC: u8;
    const ENCODED_LEN: usize;

    fn ser(&self, version: MavlinkVersion, payload: &mut [u8]) -> usize;
    fn deser(version: MavlinkVersion, payload: &[u8]) -> Result<Self, ParserError>;
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
/// and component id.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct MavFrame<M: Message> {
    pub header: MavHeader,
    pub msg: M,
    pub protocol_version: MavlinkVersion,
}

impl<M: Message> MavFrame<M> {
    /// Serialize MavFrame into a vector, so it can be sent over a socket, for example.
    /// The resulting buffer will start with the sequence field of the Mavlink frame
    /// and will not include the initial packet marker, length field, and flags.
    pub fn ser(&self, buf: &mut [u8]) -> usize {
        let mut buf = bytes_mut::BytesMut::new(buf);

        // serialize message
        let mut payload_buf = [0u8; 255];
        let payload_len = self.msg.ser(self.protocol_version, &mut payload_buf);

        // Currently expects a buffer with the sequence field at the start.
        // If this is updated to include the initial packet marker, length field, and flags,
        // uncomment.
        //
        // match self.protocol_version {
        //     MavlinkVersion::V2 => {
        //         buf.put_u8(MAV_STX_V2);
        //         buf.put_u8(payload_len as u8);
        //         but.put_u8(0); // incompatibility flags
        //         buf.put_u8(0); // compatibility flags
        //     }
        //     MavlinkVersion::V1 => {
        //         buf.put_u8(MAV_STX);
        //         buf.put_u8(payload_len as u8);
        //     }
        // }

        // serialize header
        buf.put_u8(self.header.sequence);
        buf.put_u8(self.header.system_id);
        buf.put_u8(self.header.component_id);

        // message id
        match self.protocol_version {
            MavlinkVersion::V2 => {
                let bytes: [u8; 4] = self.msg.message_id().to_le_bytes();
                buf.put_slice(&bytes[..3]);
            }
            MavlinkVersion::V1 => {
                buf.put_u8(self.msg.message_id() as u8); //TODO check
            }
        }

        buf.put_slice(&payload_buf[..payload_len]);
        buf.len()
    }

    /// Deserialize MavFrame from a slice that has been received from, for example, a socket.
    /// The input buffer should start with the sequence field of the Mavlink frame. The
    /// initial packet marker, length field, and flag fields should be excluded.
    pub fn deser(version: MavlinkVersion, input: &[u8]) -> Result<Self, ParserError> {
        let mut buf = Bytes::new(input);

        // Currently expects a buffer with the sequence field at the start.
        // If this is updated to include the initial packet marker, length field, and flags,
        // uncomment.
        // <https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format>
        // match version {
        //     MavlinkVersion::V2 => buf.get_u32_le(),
        //     MavlinkVersion::V1 => buf.get_u16_le().into(),
        // };

        let sequence = buf.get_u8();
        let system_id = buf.get_u8();
        let component_id = buf.get_u8();
        let header = MavHeader {
            system_id,
            component_id,
            sequence,
        };

        let msg_id = match version {
            MavlinkVersion::V2 => buf.get_u24_le(),
            MavlinkVersion::V1 => buf.get_u8().into(),
        };

        M::parse(version, msg_id, buf.remaining_bytes()).map(|msg| Self {
            header,
            msg,
            protocol_version: version,
        })
    }

    /// Return the frame header
    pub fn header(&self) -> MavHeader {
        self.header
    }
}

pub fn calculate_crc(data: &[u8], extra_crc: u8) -> u16 {
    let mut crc_calculator = CRCu16::crc16mcrf4cc();
    crc_calculator.digest(data);

    crc_calculator.digest(&[extra_crc]);
    crc_calculator.get_crc()
}

pub fn read_versioned_msg<M: Message, R: Read>(
    r: &mut PeekReader<R>,
    version: MavlinkVersion,
) -> Result<(MavHeader, M), error::MessageReadError> {
    match version {
        MavlinkVersion::V2 => read_v2_msg(r),
        MavlinkVersion::V1 => read_v1_msg(r),
    }
}

#[cfg(feature = "tokio-1")]
pub async fn read_versioned_msg_async<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    r: &mut AsyncPeekReader<R>,
    version: MavlinkVersion,
) -> Result<(MavHeader, M), error::MessageReadError> {
    match version {
        MavlinkVersion::V2 => read_v2_msg_async(r).await,
        MavlinkVersion::V1 => read_v1_msg_async(r).await,
    }
}

#[cfg(feature = "signing")]
pub fn read_versioned_msg_signed<M: Message, R: Read>(
    r: &mut PeekReader<R>,
    version: MavlinkVersion,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    match version {
        MavlinkVersion::V2 => read_v2_msg_inner(r, signing_data),
        MavlinkVersion::V1 => read_v1_msg(r),
    }
}

#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_versioned_msg_async_signed<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    r: &mut AsyncPeekReader<R>,
    version: MavlinkVersion,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    match version {
        MavlinkVersion::V2 => read_v2_msg_async_inner(r, signing_data).await,
        MavlinkVersion::V1 => read_v1_msg_async(r).await,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
// Follow protocol definition: `<https://mavlink.io/en/guide/serialization.html#v1_packet_format>`
pub struct MAVLinkV1MessageRaw([u8; 1 + Self::HEADER_SIZE + 255 + 2]);

impl Default for MAVLinkV1MessageRaw {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkV1MessageRaw {
    const HEADER_SIZE: usize = 5;

    pub const fn new() -> Self {
        Self([0; 1 + Self::HEADER_SIZE + 255 + 2])
    }

    #[inline]
    pub fn header(&self) -> &[u8] {
        &self.0[1..=Self::HEADER_SIZE]
    }

    #[inline]
    fn mut_header(&mut self) -> &mut [u8] {
        &mut self.0[1..=Self::HEADER_SIZE]
    }

    #[inline]
    pub fn payload_length(&self) -> u8 {
        self.0[1]
    }

    #[inline]
    pub fn sequence(&self) -> u8 {
        self.0[2]
    }

    #[inline]
    pub fn system_id(&self) -> u8 {
        self.0[3]
    }

    #[inline]
    pub fn component_id(&self) -> u8 {
        self.0[4]
    }

    #[inline]
    pub fn message_id(&self) -> u8 {
        self.0[5]
    }

    #[inline]
    pub fn payload(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        &self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + payload_length)]
    }

    #[inline]
    pub fn checksum(&self) -> u16 {
        let payload_length: usize = self.payload_length().into();
        u16::from_le_bytes([
            self.0[1 + Self::HEADER_SIZE + payload_length],
            self.0[1 + Self::HEADER_SIZE + payload_length + 1],
        ])
    }

    #[inline]
    fn mut_payload_and_checksum(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();
        &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + payload_length + 2)]
    }

    #[inline]
    pub fn has_valid_crc<M: Message>(&self) -> bool {
        let payload_length: usize = self.payload_length().into();
        self.checksum()
            == calculate_crc(
                &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
                M::extra_crc(self.message_id().into()),
            )
    }

    pub fn raw_bytes(&self) -> &[u8] {
        let payload_length = self.payload_length() as usize;
        &self.0[..(1 + Self::HEADER_SIZE + payload_length + 2)]
    }

    fn serialize_stx_and_header_and_crc(
        &mut self,
        header: MavHeader,
        msgid: u32,
        payload_length: usize,
        extra_crc: u8,
    ) {
        self.0[0] = MAV_STX;

        let header_buf = self.mut_header();
        header_buf.copy_from_slice(&[
            payload_length as u8,
            header.sequence,
            header.system_id,
            header.component_id,
            msgid as u8,
        ]);

        let crc = calculate_crc(
            &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
            extra_crc,
        );
        self.0[(1 + Self::HEADER_SIZE + payload_length)
            ..(1 + Self::HEADER_SIZE + payload_length + 2)]
            .copy_from_slice(&crc.to_le_bytes());
    }

    pub fn serialize_message<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message.ser(MavlinkVersion::V1, payload_buf);

        let message_id = message.message_id();
        self.serialize_stx_and_header_and_crc(
            header,
            message_id,
            payload_length,
            M::extra_crc(message_id),
        );
    }

    pub fn serialize_message_data<D: MessageData>(&mut self, header: MavHeader, message_data: &D) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message_data.ser(MavlinkVersion::V1, payload_buf);

        self.serialize_stx_and_header_and_crc(header, D::ID, payload_length, D::EXTRA_CRC);
    }
}

/// Return a raw buffer with the mavlink message
/// V1 maximum size is 263 bytes: `<https://mavlink.io/en/guide/serialization.html>`
pub fn read_v1_raw_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
) -> Result<MAVLinkV1MessageRaw, error::MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        while reader.peek_exact(1)?[0] != MAV_STX {
            reader.consume(1);
        }

        let mut message = MAVLinkV1MessageRaw::new();
        let whole_header_size = MAVLinkV1MessageRaw::HEADER_SIZE + 1;

        message.0[0] = MAV_STX;
        let header = &reader.peek_exact(whole_header_size)?[1..whole_header_size];
        message.mut_header().copy_from_slice(header);
        let packet_length = message.raw_bytes().len();
        let payload_and_checksum =
            &reader.peek_exact(packet_length)?[whole_header_size..packet_length];
        message
            .mut_payload_and_checksum()
            .copy_from_slice(payload_and_checksum);

        // retry if CRC failed after previous STX
        // (an STX byte may appear in the middle of a message)
        if message.has_valid_crc::<M>() {
            reader.consume(message.raw_bytes().len());
            return Ok(message);
        }

        reader.consume(1);
    }
}

/// Return a raw buffer with the mavlink message
/// V1 maximum size is 263 bytes: `<https://mavlink.io/en/guide/serialization.html>`
#[cfg(feature = "tokio-1")]
pub async fn read_v1_raw_message_async<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    reader: &mut AsyncPeekReader<R>,
) -> Result<MAVLinkV1MessageRaw, error::MessageReadError> {
    loop {
        loop {
            // search for the magic framing value indicating start of mavlink message
            if reader.read_u8().await? == MAV_STX {
                break;
            }
        }

        let mut message = MAVLinkV1MessageRaw::new();

        message.0[0] = MAV_STX;
        let header = &reader.peek_exact(MAVLinkV1MessageRaw::HEADER_SIZE).await?
            [..MAVLinkV1MessageRaw::HEADER_SIZE];
        message.mut_header().copy_from_slice(header);
        let packet_length = message.raw_bytes().len() - 1;
        let payload_and_checksum = &reader.peek_exact(packet_length).await?
            [MAVLinkV1MessageRaw::HEADER_SIZE..packet_length];
        message
            .mut_payload_and_checksum()
            .copy_from_slice(payload_and_checksum);

        // retry if CRC failed after previous STX
        // (an STX byte may appear in the middle of a message)
        if message.has_valid_crc::<M>() {
            reader.consume(message.raw_bytes().len() - 1);
            return Ok(message);
        }
    }
}

/// Async read a raw buffer with the mavlink message
/// V1 maximum size is 263 bytes: `<https://mavlink.io/en/guide/serialization.html>`
///
/// # Example
/// See mavlink/examples/embedded-async-read full example for details.
#[cfg(feature = "embedded")]
pub async fn read_v1_raw_message_async<M: Message>(
    reader: &mut impl embedded_io_async::Read,
) -> Result<MAVLinkV1MessageRaw, error::MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        let mut byte = [0u8];
        loop {
            reader
                .read_exact(&mut byte)
                .await
                .map_err(|_| error::MessageReadError::Io)?;
            if byte[0] == MAV_STX {
                break;
            }
        }

        let mut message = MAVLinkV1MessageRaw::new();

        message.0[0] = MAV_STX;
        reader
            .read_exact(message.mut_header())
            .await
            .map_err(|_| error::MessageReadError::Io)?;
        reader
            .read_exact(message.mut_payload_and_checksum())
            .await
            .map_err(|_| error::MessageReadError::Io)?;

        // retry if CRC failed after previous STX
        // (an STX byte may appear in the middle of a message)
        if message.has_valid_crc::<M>() {
            return Ok(message);
        }
    }
}

/// Read a MAVLink v1 message from a Read stream.
pub fn read_v1_msg<M: Message, R: Read>(
    r: &mut PeekReader<R>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    let message = read_v1_raw_message::<M, _>(r)?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(
            MavlinkVersion::V1,
            u32::from(message.message_id()),
            message.payload(),
        )?,
    ))
}

/// Read a MAVLink v1 message from a Read stream.
#[cfg(feature = "tokio-1")]
pub async fn read_v1_msg_async<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    r: &mut AsyncPeekReader<R>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    let message = read_v1_raw_message_async::<M, _>(r).await?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(
            MavlinkVersion::V1,
            u32::from(message.message_id()),
            message.payload(),
        )?,
    ))
}

/// Async read a MAVLink v1 message from a Read stream.
///
/// NOTE: it will be add ~80KB to firmware flash size because all *_DATA::deser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn read_v1_msg_async<M: Message>(
    r: &mut impl embedded_io_async::Read,
) -> Result<(MavHeader, M), error::MessageReadError> {
    let message = read_v1_raw_message_async::<M>(r).await?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(
            MavlinkVersion::V1,
            u32::from(message.message_id()),
            message.payload(),
        )?,
    ))
}

const MAVLINK_IFLAG_SIGNED: u8 = 0x01;
const MAVLINK_SUPPORTED_IFLAGS: u8 = MAVLINK_IFLAG_SIGNED;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
// Follow protocol definition: `<https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format>`
pub struct MAVLinkV2MessageRaw([u8; 1 + Self::HEADER_SIZE + 255 + 2 + Self::SIGNATURE_SIZE]);

impl Default for MAVLinkV2MessageRaw {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkV2MessageRaw {
    const HEADER_SIZE: usize = 9;
    const SIGNATURE_SIZE: usize = 13;

    pub const fn new() -> Self {
        Self([0; 1 + Self::HEADER_SIZE + 255 + 2 + Self::SIGNATURE_SIZE])
    }

    #[inline]
    pub fn header(&self) -> &[u8] {
        &self.0[1..=Self::HEADER_SIZE]
    }

    #[inline]
    fn mut_header(&mut self) -> &mut [u8] {
        &mut self.0[1..=Self::HEADER_SIZE]
    }

    #[inline]
    pub fn payload_length(&self) -> u8 {
        self.0[1]
    }

    #[inline]
    pub fn incompatibility_flags(&self) -> u8 {
        self.0[2]
    }

    #[inline]
    pub fn incompatibility_flags_mut(&mut self) -> &mut u8 {
        &mut self.0[2]
    }

    #[inline]
    pub fn compatibility_flags(&self) -> u8 {
        self.0[3]
    }

    #[inline]
    pub fn sequence(&self) -> u8 {
        self.0[4]
    }

    #[inline]
    pub fn system_id(&self) -> u8 {
        self.0[5]
    }

    #[inline]
    pub fn component_id(&self) -> u8 {
        self.0[6]
    }

    #[inline]
    pub fn message_id(&self) -> u32 {
        u32::from_le_bytes([self.0[7], self.0[8], self.0[9], 0])
    }

    #[inline]
    pub fn payload(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        &self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + payload_length)]
    }

    #[inline]
    pub fn checksum(&self) -> u16 {
        let payload_length: usize = self.payload_length().into();
        u16::from_le_bytes([
            self.0[1 + Self::HEADER_SIZE + payload_length],
            self.0[1 + Self::HEADER_SIZE + payload_length + 1],
        ])
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn checksum_bytes(&self) -> &[u8] {
        let checksum_offset = 1 + Self::HEADER_SIZE + self.payload_length() as usize;
        &self.0[checksum_offset..(checksum_offset + 2)]
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_link_id(&self) -> u8 {
        let payload_length: usize = self.payload_length().into();
        self.0[1 + Self::HEADER_SIZE + payload_length + 2]
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_link_id_mut(&mut self) -> &mut u8 {
        let payload_length: usize = self.payload_length().into();
        &mut self.0[1 + Self::HEADER_SIZE + payload_length + 2]
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_timestamp_bytes(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let timestamp_start = 1 + Self::HEADER_SIZE + payload_length + 3;
        &self.0[timestamp_start..(timestamp_start + 6)]
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_timestamp_bytes_mut(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();
        let timestamp_start = 1 + Self::HEADER_SIZE + payload_length + 3;
        &mut self.0[timestamp_start..(timestamp_start + 6)]
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_timestamp(&self) -> u64 {
        let mut timestamp_bytes = [0u8; 8];
        timestamp_bytes[0..6].copy_from_slice(self.signature_timestamp_bytes());
        u64::from_le_bytes(timestamp_bytes)
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_value(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let signature_start = 1 + Self::HEADER_SIZE + payload_length + 3 + 6;
        &self.0[signature_start..(signature_start + 6)]
    }

    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_value_mut(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();
        let signature_start = 1 + Self::HEADER_SIZE + payload_length + 3 + 6;
        &mut self.0[signature_start..(signature_start + 6)]
    }

    fn mut_payload_and_checksum_and_sign(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();

        // Signature to ensure the link is tamper-proof.
        let signature_size = if (self.incompatibility_flags() & MAVLINK_IFLAG_SIGNED) == 0 {
            0
        } else {
            Self::SIGNATURE_SIZE
        };

        &mut self.0
            [(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + payload_length + signature_size + 2)]
    }

    #[inline]
    pub fn has_valid_crc<M: Message>(&self) -> bool {
        let payload_length: usize = self.payload_length().into();
        self.checksum()
            == calculate_crc(
                &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
                M::extra_crc(self.message_id()),
            )
    }

    #[cfg(feature = "signing")]
    pub fn calculate_signature(&self, secret_key: &[u8], target_buffer: &mut [u8; 6]) {
        let mut hasher = Sha256::new();
        hasher.update(secret_key);
        hasher.update([MAV_STX_V2]);
        hasher.update(self.header());
        hasher.update(self.payload());
        hasher.update(self.checksum_bytes());
        hasher.update([self.signature_link_id()]);
        hasher.update(self.signature_timestamp_bytes());
        target_buffer.copy_from_slice(&hasher.finalize()[0..6]);
    }

    pub fn raw_bytes(&self) -> &[u8] {
        let payload_length = self.payload_length() as usize;

        let signature_size = if (self.incompatibility_flags() & MAVLINK_IFLAG_SIGNED) == 0 {
            0
        } else {
            Self::SIGNATURE_SIZE
        };

        &self.0[..(1 + Self::HEADER_SIZE + payload_length + signature_size + 2)]
    }

    fn serialize_stx_and_header_and_crc(
        &mut self,
        header: MavHeader,
        msgid: u32,
        payload_length: usize,
        extra_crc: u8,
        incompat_flags: u8,
    ) {
        self.0[0] = MAV_STX_V2;
        let msgid_bytes = msgid.to_le_bytes();

        let header_buf = self.mut_header();
        header_buf.copy_from_slice(&[
            payload_length as u8,
            incompat_flags,
            0, //compat_flags
            header.sequence,
            header.system_id,
            header.component_id,
            msgid_bytes[0],
            msgid_bytes[1],
            msgid_bytes[2],
        ]);

        let crc = calculate_crc(
            &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
            extra_crc,
        );
        self.0[(1 + Self::HEADER_SIZE + payload_length)
            ..(1 + Self::HEADER_SIZE + payload_length + 2)]
            .copy_from_slice(&crc.to_le_bytes());
    }

    pub fn serialize_message<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message.ser(MavlinkVersion::V2, payload_buf);

        let message_id = message.message_id();
        self.serialize_stx_and_header_and_crc(
            header,
            message_id,
            payload_length,
            M::extra_crc(message_id),
            0,
        );
    }

    pub fn serialize_message_for_signing<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message.ser(MavlinkVersion::V2, payload_buf);

        let message_id = message.message_id();
        self.serialize_stx_and_header_and_crc(
            header,
            message_id,
            payload_length,
            M::extra_crc(message_id),
            0x01,
        );
    }

    pub fn serialize_message_data<D: MessageData>(&mut self, header: MavHeader, message_data: &D) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message_data.ser(MavlinkVersion::V2, payload_buf);

        self.serialize_stx_and_header_and_crc(header, D::ID, payload_length, D::EXTRA_CRC, 0);
    }
}

/// Return a raw buffer with the mavlink message
///
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
#[inline]
pub fn read_v2_raw_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    read_v2_raw_message_inner::<M, R>(reader, None)
}

/// Return a raw buffer with the mavlink message with signing support
///
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
#[cfg(feature = "signing")]
#[inline]
pub fn read_v2_raw_message_signed<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    read_v2_raw_message_inner::<M, R>(reader, signing_data)
}

#[allow(unused_variables)]
fn read_v2_raw_message_inner<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        while reader.peek_exact(1)?[0] != MAV_STX_V2 {
            reader.consume(1);
        }

        let mut message = MAVLinkV2MessageRaw::new();
        let whole_header_size = MAVLinkV2MessageRaw::HEADER_SIZE + 1;

        message.0[0] = MAV_STX_V2;
        let header = &reader.peek_exact(whole_header_size)?[1..whole_header_size];
        message.mut_header().copy_from_slice(header);

        if message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
            // if there are incompatibility flags set that we do not know discard the message
            reader.consume(1);
            continue;
        }

        let packet_length = message.raw_bytes().len();
        let payload_and_checksum_and_sign =
            &reader.peek_exact(packet_length)?[whole_header_size..packet_length];
        message
            .mut_payload_and_checksum_and_sign()
            .copy_from_slice(payload_and_checksum_and_sign);

        if message.has_valid_crc::<M>() {
            // even if the signature turn out to be invalid the valid crc shows that the received data presents a valid message as opposed to random bytes
            reader.consume(message.raw_bytes().len());
        } else {
            reader.consume(1);
            continue;
        }

        #[cfg(feature = "signing")]
        if let Some(signing_data) = signing_data {
            if !signing_data.verify_signature(&message) {
                continue;
            }
        }

        return Ok(message);
    }
}

/// Async read a raw buffer with the mavlink message
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
#[cfg(feature = "tokio-1")]
pub async fn read_v2_raw_message_async<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    reader: &mut AsyncPeekReader<R>,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    read_v2_raw_message_async_inner::<M, R>(reader, None).await
}

/// Async read a raw buffer with the mavlink message
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
#[cfg(feature = "tokio-1")]
#[allow(unused_variables)]
async fn read_v2_raw_message_async_inner<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    loop {
        loop {
            // search for the magic framing value indicating start of mavlink message
            if reader.read_u8().await? == MAV_STX_V2 {
                break;
            }
        }

        let mut message = MAVLinkV2MessageRaw::new();

        message.0[0] = MAV_STX_V2;
        let header = &reader.peek_exact(MAVLinkV2MessageRaw::HEADER_SIZE).await?
            [..MAVLinkV2MessageRaw::HEADER_SIZE];
        message.mut_header().copy_from_slice(header);

        if message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
            // if there are incompatibility flags set that we do not know discard the message
            continue;
        }

        let packet_length = message.raw_bytes().len() - 1;
        let payload_and_checksum_and_sign = &reader.peek_exact(packet_length).await?
            [MAVLinkV2MessageRaw::HEADER_SIZE..packet_length];
        message
            .mut_payload_and_checksum_and_sign()
            .copy_from_slice(payload_and_checksum_and_sign);

        if message.has_valid_crc::<M>() {
            // even if the signature turn out to be invalid the valid crc shows that the received data presents a valid message as opposed to random bytes
            reader.consume(message.raw_bytes().len() - 1);
        } else {
            continue;
        }

        #[cfg(feature = "signing")]
        if let Some(signing_data) = signing_data {
            if !signing_data.verify_signature(&message) {
                continue;
            }
        }

        return Ok(message);
    }
}

/// Async read a raw buffer with the mavlink message with signing support
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_v2_raw_message_async_signed<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    read_v2_raw_message_async_inner::<M, R>(reader, signing_data).await
}

/// Async read a raw buffer with the mavlink message
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
///
/// # Example
/// See mavlink/examples/embedded-async-read full example for details.
#[cfg(feature = "embedded")]
pub async fn read_v2_raw_message_async<M: Message>(
    reader: &mut impl embedded_io_async::Read,
) -> Result<MAVLinkV2MessageRaw, error::MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        let mut byte = [0u8];
        loop {
            reader
                .read_exact(&mut byte)
                .await
                .map_err(|_| error::MessageReadError::Io)?;
            if byte[0] == MAV_STX_V2 {
                break;
            }
        }

        let mut message = MAVLinkV2MessageRaw::new();

        message.0[0] = MAV_STX_V2;
        reader
            .read_exact(message.mut_header())
            .await
            .map_err(|_| error::MessageReadError::Io)?;

        if message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
            // if there are incompatibility flags set that we do not know discard the message
            continue;
        }

        reader
            .read_exact(message.mut_payload_and_checksum_and_sign())
            .await
            .map_err(|_| error::MessageReadError::Io)?;

        // retry if CRC failed after previous STX
        // (an STX byte may appear in the middle of a message)
        if message.has_valid_crc::<M>() {
            return Ok(message);
        }
    }
}

/// Read a MAVLink v2  message from a Read stream.
#[inline]
pub fn read_v2_msg<M: Message, R: Read>(
    read: &mut PeekReader<R>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    read_v2_msg_inner(read, None)
}

/// Read a MAVLink v2 message from a Read stream.
#[cfg(feature = "signing")]
#[inline]
pub fn read_v2_msg_signed<M: Message, R: Read>(
    read: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    read_v2_msg_inner(read, signing_data)
}

fn read_v2_msg_inner<M: Message, R: Read>(
    read: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    let message = read_v2_raw_message_inner::<M, _>(read, signing_data)?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(MavlinkVersion::V2, message.message_id(), message.payload())?,
    ))
}

/// Async read a MAVLink v2  message from a Read stream.
#[cfg(feature = "tokio-1")]
pub async fn read_v2_msg_async<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    read: &mut AsyncPeekReader<R>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    read_v2_msg_async_inner(read, None).await
}

/// Async read a MAVLink v2  message from a Read stream.
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_v2_msg_async_signed<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    read: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    read_v2_msg_async_inner(read, signing_data).await
}

#[cfg(feature = "tokio-1")]
async fn read_v2_msg_async_inner<M: Message, R: tokio::io::AsyncReadExt + Unpin>(
    read: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), error::MessageReadError> {
    let message = read_v2_raw_message_async_inner::<M, _>(read, signing_data).await?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(MavlinkVersion::V2, message.message_id(), message.payload())?,
    ))
}

/// Async read a MAVLink v2  message from a Read stream.
///
/// NOTE: it will be add ~80KB to firmware flash size because all *_DATA::deser methods will be add to firmware.
/// Use `*_DATA::deser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn read_v2_msg_async<M: Message, R: embedded_io_async::Read>(
    r: &mut R,
) -> Result<(MavHeader, M), error::MessageReadError> {
    let message = read_v2_raw_message_async::<M>(r).await?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(
            MavlinkVersion::V2,
            u32::from(message.message_id()),
            message.payload(),
        )?,
    ))
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

/// Write a message with signing support using the given mavlink version
#[cfg(feature = "signing")]
pub fn write_versioned_msg_signed<M: Message, W: Write>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, error::MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_signed(w, header, data, signing_data),
        MavlinkVersion::V1 => write_v1_msg(w, header, data),
    }
}

/// Async write a message using the given mavlink version
#[cfg(feature = "tokio-1")]
pub async fn write_versioned_msg_async<M: Message, W: tokio::io::AsyncWriteExt + Unpin>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_async(w, header, data).await,
        MavlinkVersion::V1 => write_v1_msg_async(w, header, data).await,
    }
}

/// Async write a message with signing support using the given mavlink version
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn write_versioned_msg_async_signed<M: Message, W: tokio::io::AsyncWriteExt + Unpin>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, error::MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_async_signed(w, header, data, signing_data).await,
        MavlinkVersion::V1 => write_v1_msg_async(w, header, data).await,
    }
}

/// Async write a message using the given mavlink version
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn write_versioned_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_async(w, header, data).await,
        MavlinkVersion::V1 => write_v1_msg_async(w, header, data).await,
    }
}

/// Write a MAVLink v2 message to a Write stream.
pub fn write_v2_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Write a MAVLink v2 message to a Write stream with signing support.
#[cfg(feature = "signing")]
pub fn write_v2_msg_signed<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();

    let signature_len = if let Some(signing_data) = signing_data {
        if signing_data.config.sign_outgoing {
            message_raw.serialize_message_for_signing(header, data);
            signing_data.sign_message(&mut message_raw);
            MAVLinkV2MessageRaw::SIGNATURE_SIZE
        } else {
            message_raw.serialize_message(header, data);
            0
        }
    } else {
        message_raw.serialize_message(header, data);
        0
    };

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2 + signature_len;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Async write a MAVLink v2 message to a Write stream.
#[cfg(feature = "tokio-1")]
pub async fn write_v2_msg_async<M: Message, W: tokio::io::AsyncWriteExt + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Write a MAVLink v2 message to a Write stream with signing support.
#[cfg(feature = "signing")]
#[cfg(feature = "tokio-1")]
pub async fn write_v2_msg_async_signed<M: Message, W: tokio::io::AsyncWriteExt + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();

    let signature_len = if let Some(signing_data) = signing_data {
        if signing_data.config.sign_outgoing {
            message_raw.serialize_message_for_signing(header, data);
            signing_data.sign_message(&mut message_raw);
            MAVLinkV2MessageRaw::SIGNATURE_SIZE
        } else {
            message_raw.serialize_message(header, data);
            0
        }
    } else {
        message_raw.serialize_message(header, data);
        0
    };

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2 + signature_len;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Async write a MAVLink v2 message to a Write stream.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn write_v2_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])
        .await
        .map_err(|_| error::MessageWriteError::Io)?;

    Ok(len)
}

/// Write a MAVLink v1 message to a Write stream.
pub fn write_v1_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV1MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV1MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Async write a MAVLink v1 message to a Write stream.
#[cfg(feature = "tokio-1")]
pub async fn write_v1_msg_async<M: Message, W: tokio::io::AsyncWriteExt + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV1MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV1MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Write a MAVLink v1 message to a Write stream.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn write_v1_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    header: MavHeader,
    data: &M,
) -> Result<usize, error::MessageWriteError> {
    let mut message_raw = MAVLinkV1MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV1MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])
        .await
        .map_err(|_| error::MessageWriteError::Io)?;

    Ok(len)
}
