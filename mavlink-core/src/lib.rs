//! The MAVLink message set.
//!
//! # Message sets and the `Message` trait
//! Each message set has its own module with corresponding data types, including a `MavMessage` enum
//! that represents all possible messages in that message set. The [`Message`] trait is used to
//! represent messages in an abstract way, and each `MavMessage` enum implements this trait (for
//! example, [ardupilotmega::MavMessage]). This is then monomorphized to the specific message
//! set you are using in your application at compile-time via type parameters. If you expect
//! ArduPilotMega-flavored messages, then you will need a `MavConnection<ardupilotmega::MavMessage>`
//! and you will receive `ardupilotmega::MavMessage`s from it.
//!
//! Some message sets include others. For example, most message sets include the
//! common message set. These included values are not differently represented in the `MavMessage` enum: a message
//! in the common message set received on an ArduPilotMega connection will just be an
//! `ardupilotmega::MavMessage`.
//!
//! If you want to enable a given message set, you do not have to enable the
//! feature for the message sets that it includes. For example, you can use the `dialect-ardupilotmega`
//! feature without also using the `dialect-uavionix`, `dialect-icarous`, `dialect-common` features.
//!
//! [ardupilotmega::MavMessage]: https://docs.rs/mavlink/latest/mavlink/ardupilotmega/enum.MavMessage.html
//!
//! # Reading
//!
//! Use [`MavlinkReader`] for blocking input and `AsyncMavlinkReader` for
//! asynchronous input. Both readers retain partial frames and read-ahead data
//! between calls, discard invalid candidates, and verify message CRCs before
//! returning.
//!
//! [`MavlinkReader`]: crate::MavlinkReader
//!
//! # Write Functions
//!
//! The `write_` functions are used to write a MAVLink to a [`Write`]r.
//! They follow the pattern `write_(v1|v2|versioned)_msg[_async][_signed](..)`:
//!
//! - `v1` functions write messages using MAVLink 1 serialisation
//! - `v2` functions write messages using MAVLink 2 serialisation
//! - `versioned` functions write messages using the version specified in an aditional `version` parameter
//! - `_async` functions, which are only enabled with the `tokio` feature, are
//!   [async](https://doc.rust-lang.org/std/keyword.async.html) and write to a `tokio::io::AsyncWrite` instead.
//! - `_signed` functions, which are only enabled with the `mav2-message-signing` feature, have an `Option<&SigningData>` parameter that allows the use of MAVLink 2 message signing.
//!
//! ## Write errors
//!
//! All `write_` functions return `Result<_,` [`MessageWriteError`]`>`.
//!
//! - When an error other than `std::io::ErrorKind::Interrupted` occurs on the
//!   underlying writer, the function returns [`MessageWriteError::Io`]
//! - When attempting to serialize a message with an ID over 255 with MAVLink 1 a [`MessageWriteError::MAVLink2Only`] is returned
//!
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![deny(clippy::all)]
#![warn(clippy::use_self)]

use core::result::Result;

#[cfg(feature = "std")]
use std::io::Write;

pub mod utils;
#[allow(unused_imports)]
use utils::{RustDefault, remove_trailing_zeroes};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{
    bytes::Bytes,
    error::{MessageWriteError, ParserError},
};

use crc_any::CRCu16;

#[doc(hidden)]
pub mod bytes;
#[doc(hidden)]
pub mod bytes_mut;
#[cfg(any(feature = "std", feature = "tokio"))]
mod connection;
pub mod consts;
pub mod error;
mod frame_decoder;
mod reader;
pub mod types;
#[cfg(feature = "std")]
pub use self::connection::{Connectable, Connection, MavConnection, connect};
pub use reader::MavlinkReader;

#[cfg(feature = "tokio")]
pub use self::connection::{AsyncConnectable, AsyncMavConnection, connect_async};

#[cfg(feature = "tokio")]
use tokio::io::{AsyncWrite, AsyncWriteExt};

#[cfg(any(feature = "tokio", all(feature = "embedded", not(feature = "std"))))]
mod async_reader;
#[cfg(any(feature = "tokio", all(feature = "embedded", not(feature = "std"))))]
pub use async_reader::AsyncMavlinkReader;

#[cfg(all(feature = "embedded", not(feature = "std")))]
pub mod embedded;
#[cfg(all(feature = "embedded", not(feature = "std")))]
use embedded::Write;

#[cfg(not(feature = "mav2-message-signing"))]
type SigningData = ();
#[cfg(feature = "mav2-message-signing")]
mod signing;
#[cfg(feature = "mav2-message-signing")]
pub use self::signing::{SigningConfig, SigningData};
#[cfg(feature = "mav2-message-signing")]
use sha2::{Digest, Sha256};

#[cfg(feature = "arbitrary")]
use arbitrary::Arbitrary;

#[cfg(any(feature = "std", feature = "tokio"))]
mod connectable;

#[cfg(any(feature = "std", feature = "tokio"))]
mod connection_shared;

#[cfg(any(feature = "std", feature = "tokio"))]
pub use connectable::ConnectionAddress;

#[cfg(feature = "transport-direct-serial")]
pub use connection::direct_serial::config::SerialConfig;

#[cfg(feature = "transport-tcp")]
pub use connection::tcp::config::{TcpConfig, TcpMode};

#[cfg(feature = "transport-udp")]
pub use connection::udp::config::{UdpConfig, UdpMode};

#[cfg(feature = "std")]
pub use connection::file::config::FileConfig;

/// A MAVLink message payload
///
/// Each message sets `MavMessage` enum implements this trait. The [`Message`] trait is used to
/// represent messages in an abstract way (for example, `common::MavMessage`).
pub trait Message
where
    Self: Sized,
{
    /// MAVLink message ID
    fn message_id(&self) -> u32;

    /// MAVLink message name
    fn message_name(&self) -> &'static str;

    /// Target system ID if the message is directed to a specific system
    fn target_system_id(&self) -> Option<u8>;

    /// Target component ID if the message is directed to a specific component
    fn target_component_id(&self) -> Option<u8>;

    /// Serialize **Message** into byte slice and return count of bytes written
    ///
    /// # Panics
    ///
    /// Will panic if the buffer provided is to small to store this message
    fn ser(&self, version: MavlinkVersion, bytes: &mut [u8]) -> usize;

    /// Parse a Message from its message id and payload bytes
    ///
    /// # Errors
    ///
    /// - [`UnknownMessage`] if the given message id is not part of the dialect
    /// - any other [`ParserError`] returned by the individual message deserialization
    ///
    /// [`UnknownMessage`]: ParserError::UnknownMessage
    fn parse(version: MavlinkVersion, msgid: u32, payload: &[u8]) -> Result<Self, ParserError>;

    /// Return message id of specific message name
    fn message_id_from_name(name: &str) -> Option<u32>;
    /// Return a default message of the speicfied message id
    fn default_message_from_id(id: u32) -> Option<Self>;
    /// Return random valid message of the speicfied message id
    #[cfg(feature = "arbitrary")]
    fn random_message_from_id<R: rand::TryRng<Error = std::convert::Infallible>>(
        id: u32,
        rng: &mut R,
    ) -> Option<Self>;
    /// Return a message types [CRC_EXTRA byte](https://mavlink.io/en/guide/serialization.html#crc_extra)
    fn extra_crc(id: u32) -> u8;
}

pub trait MessageData: Sized {
    type Message: Message;

    const ID: u32;
    const NAME: &'static str;
    const EXTRA_CRC: u8;
    const ENCODED_LEN: usize;

    /// # Panics
    ///
    /// Will panic if the buffer provided is to small to hold the full message payload of the implementing message type
    fn ser(&self, version: MavlinkVersion, payload: &mut [u8]) -> usize;
    /// # Errors
    ///
    /// Will return [`ParserError::InvalidEnum`] on a nonexistent enum value and
    fn deser(version: MavlinkVersion, payload: &[u8]) -> Result<Self, ParserError>;
}

/// Metadata from a MAVLink packet header
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
pub struct MavHeader {
    /// Sender system ID
    pub system_id: u8,
    /// Sender component ID
    pub component_id: u8,
    /// Packet sequence number
    pub sequence: u8,
}

/// [Versions of the MAVLink](https://mavlink.io/en/guide/mavlink_version.html) protocol that we support
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
#[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
pub enum MavlinkVersion {
    /// Version v1.0
    V1,
    /// Version v2.0
    V2,
}

/// Message framing marker for MAVLink 1
pub const MAV_STX: u8 = 0xFE;

/// Message framing marker for MAVLink 2
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

/// Encapsulation of the MAVLink message and the header,
/// important to preserve information about the sender system
/// and component id.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize))]
#[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
pub struct MavFrame<M: Message> {
    /// Message header data
    pub header: MavHeader,
    /// Parsed [`Message`] payload
    pub msg: M,
    /// Messages MAVLink version
    pub protocol_version: MavlinkVersion,
}

impl<M: Message> MavFrame<M> {
    /// Serialize MavFrame into a byte slice, so it can be sent over a socket, for example.
    /// The resulting buffer will start with the sequence field of the MAVLink frame
    /// and will not include the initial packet marker, length field, and flags.
    ///
    /// # Panics
    ///
    /// - If the frame does not fit in the provided buffer
    /// - When attempting to serialize a message with an id greater then 255 with MAVLink 1
    pub fn ser(&self, buf: &mut [u8]) -> usize {
        let mut buf = bytes_mut::BytesMut::new(buf);

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
        buf.put_slice(&[
            self.header.sequence,
            self.header.system_id,
            self.header.component_id,
        ]);

        // message id
        match self.protocol_version {
            MavlinkVersion::V2 => buf.put_u24_le(self.msg.message_id()),
            MavlinkVersion::V1 => buf.put_u8(
                self.msg
                    .message_id()
                    .try_into()
                    .expect("message is MAVLink 2 only"),
            ),
        }

        let header_len = buf.len();
        // Serialize the payload straight into the destination buffer right after
        // the header and avoid an intermediate buffer and an extra copy.
        let payload_len = self.msg.ser(self.protocol_version, &mut buf[header_len..]);

        header_len + payload_len
    }

    /// Deserialize MavFrame from a slice that has been received from, for example, a socket.
    /// The input buffer should start with the sequence field of the MAVLink frame. The
    /// initial packet marker, length field, and flag fields should be excluded.
    ///
    /// # Errors
    ///
    /// Will return a [`ParserError`] if a message was found but could not be parsed
    /// or the if the buffer provided does not contain a full message
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

        let sequence = buf.get_u8()?;
        let system_id = buf.get_u8()?;
        let component_id = buf.get_u8()?;
        let header = MavHeader {
            system_id,
            component_id,
            sequence,
        };

        let msg_id = match version {
            MavlinkVersion::V2 => buf.get_u24_le()?,
            MavlinkVersion::V1 => buf.get_u8()?.into(),
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

/// Calculates the [CRC checksum](https://mavlink.io/en/guide/serialization.html#checksum) of a messages header, payload and the CRC_EXTRA byte.
pub fn calculate_crc(data: &[u8], extra_crc: u8) -> u16 {
    let mut crc_calculator = CRCu16::crc16mcrf4cc();
    crc_calculator.digest(data);

    crc_calculator.digest(&[extra_crc]);
    crc_calculator.get_crc()
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Byte buffer containing the raw representation of a MAVLink 1 message beginning with the STX marker.
///
/// Follow protocol definition: <https://mavlink.io/en/guide/serialization.html#v1_packet_format>.
/// Maximum size is 263 bytes.
pub struct MAVLinkV1MessageRaw([u8; consts::v1::FRAME_SIZE]);

impl Default for MAVLinkV1MessageRaw {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkV1MessageRaw {
    /// Create a new raw MAVLink 1 message filled with zeros.
    pub const fn new() -> Self {
        Self([0; consts::v1::FRAME_SIZE])
    }

    /// Create a new raw MAVLink 1 message from a given buffer.
    ///
    /// Note: This method does not guarantee that the constructed MAVLink message is valid.
    pub const fn from_bytes_unparsed(bytes: [u8; consts::v1::FRAME_SIZE]) -> Self {
        Self(bytes)
    }

    /// Read access to its internal buffer.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    /// Mutable reference to its internal buffer.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }

    /// Deconstruct the MAVLink message into its owned internal buffer.
    #[inline]
    pub fn into_inner(self) -> [u8; consts::v1::FRAME_SIZE] {
        self.0
    }

    /// Reference to the 5 byte header slice of the message
    #[inline]
    pub fn header(&self) -> &[u8] {
        &self.0[consts::STX_SIZE..(consts::STX_SIZE + consts::v1::HEADER_SIZE)]
    }

    /// Mutable reference to the 5 byte header slice of the message
    #[inline]
    fn mut_header(&mut self) -> &mut [u8] {
        &mut self.0[consts::STX_SIZE..(consts::STX_SIZE + consts::v1::HEADER_SIZE)]
    }

    /// Size of the payload of the message
    #[inline]
    pub fn payload_length(&self) -> u8 {
        self.0[consts::PAYLOAD_LEN_OFFSET]
    }

    /// Packet sequence number
    #[inline]
    pub fn sequence(&self) -> u8 {
        self.0[2]
    }

    /// Message sender System ID
    #[inline]
    pub fn system_id(&self) -> u8 {
        self.0[3]
    }

    /// Message sender Component ID
    #[inline]
    pub fn component_id(&self) -> u8 {
        self.0[4]
    }

    /// Message ID
    #[inline]
    pub fn message_id(&self) -> u8 {
        self.0[5]
    }

    /// Reference to the payload byte slice of the message
    #[inline]
    pub fn payload(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let payload_offset = consts::STX_SIZE + consts::v1::HEADER_SIZE;
        &self.0[payload_offset..(payload_offset + payload_length)]
    }

    /// [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) field of the message
    #[inline]
    pub fn checksum(&self) -> u16 {
        let payload_length: usize = self.payload_length().into();
        let checksum_offset = consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length;
        u16::from_le_bytes([self.0[checksum_offset], self.0[checksum_offset + 1]])
    }

    /// Checks wether the message’s [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) calculation matches its checksum field.
    #[inline]
    pub fn has_valid_crc<M: Message>(&self) -> bool {
        let payload_length: usize = self.payload_length().into();
        let crc_end = consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length;
        self.checksum()
            == calculate_crc(
                &self.0[consts::STX_SIZE..crc_end],
                M::extra_crc(self.message_id().into()),
            )
    }

    /// Raw byte slice of the message
    pub fn raw_bytes(&self) -> &[u8] {
        let payload_length = self.payload_length() as usize;
        let frame_len =
            consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;
        &self.0[..frame_len]
    }

    /// # Panics
    ///
    /// If the `msgid` parameter exceeds 255 and is therefore not supported for MAVLink 1
    fn serialize_stx_and_header_and_crc(
        &mut self,
        header: MavHeader,
        msgid: u32,
        payload_length: usize,
        extra_crc: u8,
    ) {
        self.0[consts::STX_OFFSET] = MAV_STX;

        let header_buf = self.mut_header();
        header_buf.copy_from_slice(&[
            payload_length as u8,
            header.sequence,
            header.system_id,
            header.component_id,
            msgid.try_into().unwrap(),
        ]);

        let crc = calculate_crc(
            &self.0
                [consts::STX_SIZE..(consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length)],
            extra_crc,
        );
        self.0[(consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length)
            ..(consts::STX_SIZE
                + consts::v1::HEADER_SIZE
                + payload_length
                + consts::CHECKSUM_SIZE)]
            .copy_from_slice(&crc.to_le_bytes());
    }

    /// Serialize a [`Message`] with a given header into this raw message buffer.
    ///
    /// # Panics
    ///
    /// If the message's id exceeds 255 and is therefore not supported for MAVLink 1
    pub fn serialize_message<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_offset = consts::STX_SIZE + consts::v1::HEADER_SIZE;
        let payload_buf = &mut self.0[payload_offset..(payload_offset + consts::MAX_PAYLOAD_LEN)];
        let payload_length = message.ser(MavlinkVersion::V1, payload_buf);

        let message_id = message.message_id();
        self.serialize_stx_and_header_and_crc(
            header,
            message_id,
            payload_length,
            M::extra_crc(message_id),
        );
    }

    /// # Panics
    ///
    /// If the `MessageData`'s `ID` exceeds 255 and is therefore not supported for MAVLink 1
    pub fn serialize_message_data<D: MessageData>(&mut self, header: MavHeader, message_data: &D) {
        let payload_offset = consts::STX_SIZE + consts::v1::HEADER_SIZE;
        let payload_buf = &mut self.0[payload_offset..(payload_offset + consts::MAX_PAYLOAD_LEN)];
        let payload_length = message_data.ser(MavlinkVersion::V1, payload_buf);

        self.serialize_stx_and_header_and_crc(header, D::ID, payload_length, D::EXTRA_CRC);
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Byte buffer containing the raw representation of a MAVLink 2 message beginning with the STX marker.
///
/// Follow protocol definition: <https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format>.
/// Maximum size is [280 bytes](consts::MAX_FRAME_SIZE).
pub struct MAVLinkV2MessageRaw([u8; consts::MAX_FRAME_SIZE]);

impl Default for MAVLinkV2MessageRaw {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkV2MessageRaw {
    /// Create a new raw MAVLink 2 message filled with zeros.
    pub const fn new() -> Self {
        Self([0; consts::MAX_FRAME_SIZE])
    }

    /// Create a new raw MAVLink 2 message from a given buffer.
    ///
    /// Note: This method does not guarantee that the constructed MAVLink message is valid.
    pub const fn from_bytes_unparsed(bytes: [u8; consts::MAX_FRAME_SIZE]) -> Self {
        Self(bytes)
    }

    /// Read access to its internal buffer.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    /// Mutable reference to its internal buffer.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }

    /// Deconstruct the MAVLink message into its owned internal buffer.
    #[inline]
    pub fn into_inner(self) -> [u8; consts::MAX_FRAME_SIZE] {
        self.0
    }

    /// Reference to the 9 byte header slice of the message
    #[inline]
    pub fn header(&self) -> &[u8] {
        &self.0[consts::STX_SIZE..(consts::STX_SIZE + consts::v2::HEADER_SIZE)]
    }

    /// Mutable reference to the header byte slice of the message
    #[inline]
    fn mut_header(&mut self) -> &mut [u8] {
        &mut self.0[consts::STX_SIZE..(consts::STX_SIZE + consts::v2::HEADER_SIZE)]
    }

    /// Size of the payload of the message
    #[inline]
    pub fn payload_length(&self) -> u8 {
        self.0[consts::PAYLOAD_LEN_OFFSET]
    }

    /// [Incompatiblity flags](https://mavlink.io/en/guide/serialization.html#incompat_flags) of the message
    ///
    /// Currently the only supported incompatebility flag is `consts::v2::IFLAG_SIGNED`.
    #[inline]
    pub fn incompatibility_flags(&self) -> u8 {
        self.0[consts::v2::INCOMPAT_FLAGS_OFFSET]
    }

    /// Mutable reference to the [incompatiblity flags](https://mavlink.io/en/guide/serialization.html#incompat_flags) of the message
    ///
    /// Currently the only supported incompatebility flag is `consts::v2::IFLAG_SIGNED`.
    #[inline]
    pub fn incompatibility_flags_mut(&mut self) -> &mut u8 {
        &mut self.0[consts::v2::INCOMPAT_FLAGS_OFFSET]
    }

    /// [Compatibility Flags](https://mavlink.io/en/guide/serialization.html#compat_flags) of the message
    #[inline]
    pub fn compatibility_flags(&self) -> u8 {
        self.0[3]
    }

    /// Packet sequence number
    #[inline]
    pub fn sequence(&self) -> u8 {
        self.0[4]
    }

    /// Message sender System ID
    #[inline]
    pub fn system_id(&self) -> u8 {
        self.0[5]
    }

    /// Message sender Component ID
    #[inline]
    pub fn component_id(&self) -> u8 {
        self.0[6]
    }

    /// Message ID
    #[inline]
    pub fn message_id(&self) -> u32 {
        u32::from_le_bytes([self.0[7], self.0[8], self.0[9], 0])
    }

    /// Reference to the payload byte slice of the message
    #[inline]
    pub fn payload(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let payload_offset = consts::STX_SIZE + consts::v2::HEADER_SIZE;
        &self.0[payload_offset..(payload_offset + payload_length)]
    }

    /// [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) field of the message
    #[inline]
    pub fn checksum(&self) -> u16 {
        let payload_length: usize = self.payload_length().into();
        let checksum_offset = consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length;
        u16::from_le_bytes([self.0[checksum_offset], self.0[checksum_offset + 1]])
    }

    /// Reference to the 2 checksum bytes of the message
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn checksum_bytes(&self) -> &[u8] {
        let checksum_offset =
            consts::STX_SIZE + consts::v2::HEADER_SIZE + self.payload_length() as usize;
        &self.0[checksum_offset..(checksum_offset + consts::CHECKSUM_SIZE)]
    }

    /// Signature [Link ID](https://mavlink.io/en/guide/message_signing.html#link_ids)
    ///
    /// If the message is not signed this 0.
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_link_id(&self) -> u8 {
        let payload_length: usize = self.payload_length().into();
        self.0[consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE]
    }

    /// Mutable reference to the signature [Link ID](https://mavlink.io/en/guide/message_signing.html#link_ids)
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_link_id_mut(&mut self) -> &mut u8 {
        let payload_length: usize = self.payload_length().into();
        &mut self.0
            [consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE]
    }

    /// Message [signature timestamp](https://mavlink.io/en/guide/message_signing.html#timestamp)
    ///
    /// The timestamp is a 48 bit number with units of 10 microseconds since 1st January 2015 GMT.
    /// The offset since 1st January 1970 (the unix epoch) is 1420070400 seconds.
    /// Since all timestamps generated must be at least 1 more than the previous timestamp this timestamp may get ahead of GMT time if there is a burst of packets at a rate of more than 100000 packets per second.
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_timestamp(&self) -> u64 {
        let mut timestamp_bytes = [0u8; 8];
        timestamp_bytes[0..consts::v2::SIGNATURE_TIMESTAMP_SIZE]
            .copy_from_slice(self.signature_timestamp_bytes());
        u64::from_le_bytes(timestamp_bytes)
    }

    /// 48 bit [signature timestamp](https://mavlink.io/en/guide/message_signing.html#timestamp) byte slice
    ///
    /// If the message is not signed this contains zeros.
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_timestamp_bytes(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let timestamp_start = consts::STX_SIZE
            + consts::v2::HEADER_SIZE
            + payload_length
            + consts::CHECKSUM_SIZE
            + consts::v2::SIGNATURE_LINK_ID_SIZE;
        &self.0[timestamp_start..(timestamp_start + consts::v2::SIGNATURE_TIMESTAMP_SIZE)]
    }

    /// Mutable reference to the 48 bit signature timestams byte slice
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_timestamp_bytes_mut(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();
        let timestamp_start = consts::STX_SIZE
            + consts::v2::HEADER_SIZE
            + payload_length
            + consts::CHECKSUM_SIZE
            + consts::v2::SIGNATURE_LINK_ID_SIZE;
        &mut self.0[timestamp_start..(timestamp_start + consts::v2::SIGNATURE_TIMESTAMP_SIZE)]
    }

    /// Reference to the 48 bit [message signature](https://mavlink.io/en/guide/message_signing.html#signature) byte slice
    ///
    /// If the message is not signed this contains zeros.
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_value(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let signature_start = consts::STX_SIZE
            + consts::v2::HEADER_SIZE
            + payload_length
            + consts::CHECKSUM_SIZE
            + consts::v2::SIGNATURE_LINK_ID_SIZE
            + consts::v2::SIGNATURE_TIMESTAMP_SIZE;
        &self.0[signature_start..(signature_start + consts::v2::SIGNATURE_VALUE_SIZE)]
    }

    /// Mutable reference to the 48 bit [message signature](https://mavlink.io/en/guide/message_signing.html#signature) byte slice
    #[cfg(feature = "mav2-message-signing")]
    #[inline]
    pub fn signature_value_mut(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();
        let signature_start = consts::STX_SIZE
            + consts::v2::HEADER_SIZE
            + payload_length
            + consts::CHECKSUM_SIZE
            + consts::v2::SIGNATURE_LINK_ID_SIZE
            + consts::v2::SIGNATURE_TIMESTAMP_SIZE;
        &mut self.0[signature_start..(signature_start + consts::v2::SIGNATURE_VALUE_SIZE)]
    }

    /// Checks wether the message's [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) calculation matches its checksum field.
    #[inline]
    pub fn has_valid_crc<M: Message>(&self) -> bool {
        let payload_length: usize = self.payload_length().into();
        let crc_end = consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length;
        self.checksum()
            == calculate_crc(
                &self.0[consts::STX_SIZE..crc_end],
                M::extra_crc(self.message_id()),
            )
    }

    /// Calculates the messages sha256_48 signature.
    ///
    /// This calculates the [SHA-256](https://en.wikipedia.org/wiki/SHA-2) checksum of messages appended to the 32 byte secret key and copies the first 6 bytes of the result into the target buffer.
    #[cfg(feature = "mav2-message-signing")]
    pub fn calculate_signature(
        &self,
        secret_key: &[u8],
        target_buffer: &mut [u8; consts::v2::SIGNATURE_VALUE_SIZE],
    ) {
        let mut hasher = Sha256::new();
        hasher.update(secret_key);
        hasher.update([MAV_STX_V2]);
        hasher.update(self.header());
        hasher.update(self.payload());
        hasher.update(self.checksum_bytes());
        hasher.update([self.signature_link_id()]);
        hasher.update(self.signature_timestamp_bytes());
        target_buffer.copy_from_slice(&hasher.finalize()[0..consts::v2::SIGNATURE_VALUE_SIZE]);
    }

    /// Raw byte slice of the message
    pub fn raw_bytes(&self) -> &[u8] {
        let payload_length = self.payload_length() as usize;

        let signature_size = if (self.incompatibility_flags() & consts::v2::IFLAG_SIGNED) == 0 {
            0
        } else {
            consts::v2::SIGNATURE_SIZE
        };

        let frame_len = consts::STX_SIZE
            + consts::v2::HEADER_SIZE
            + payload_length
            + signature_size
            + consts::CHECKSUM_SIZE;
        &self.0[..frame_len]
    }

    fn serialize_stx_and_header_and_crc(
        &mut self,
        header: MavHeader,
        msgid: u32,
        payload_length: usize,
        extra_crc: u8,
        incompat_flags: u8,
    ) {
        self.0[consts::STX_OFFSET] = MAV_STX_V2;
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
            &self.0
                [consts::STX_SIZE..(consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length)],
            extra_crc,
        );
        self.0[(consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length)
            ..(consts::STX_SIZE
                + consts::v2::HEADER_SIZE
                + payload_length
                + consts::CHECKSUM_SIZE)]
            .copy_from_slice(&crc.to_le_bytes());
    }

    /// Serialize a [Message] with a given header into this raw message buffer.
    ///
    /// This does not set any compatiblity or incompatiblity flags.
    pub fn serialize_message<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_offset = consts::STX_SIZE + consts::v2::HEADER_SIZE;
        let payload_buf = &mut self.0[payload_offset..(payload_offset + consts::MAX_PAYLOAD_LEN)];
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

    /// Serialize a [Message] with a given header into this raw message buffer and sets the `consts::v2::IFLAG_SIGNED` incompatiblity flag.
    ///
    /// This does not update the message's signature fields.
    /// This does not set any compatiblity flags.
    pub fn serialize_message_for_signing<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_offset = consts::STX_SIZE + consts::v2::HEADER_SIZE;
        let payload_buf = &mut self.0[payload_offset..(payload_offset + consts::MAX_PAYLOAD_LEN)];
        let payload_length = message.ser(MavlinkVersion::V2, payload_buf);

        let message_id = message.message_id();
        self.serialize_stx_and_header_and_crc(
            header,
            message_id,
            payload_length,
            M::extra_crc(message_id),
            consts::v2::IFLAG_SIGNED,
        );
    }

    pub fn serialize_message_data<D: MessageData>(&mut self, header: MavHeader, message_data: &D) {
        let payload_offset = consts::STX_SIZE + consts::v2::HEADER_SIZE;
        let payload_buf = &mut self.0[payload_offset..(payload_offset + consts::MAX_PAYLOAD_LEN)];
        let payload_length = message_data.ser(MavlinkVersion::V2, payload_buf);

        self.serialize_stx_and_header_and_crc(header, D::ID, payload_length, D::EXTRA_CRC, 0);
    }
}

/// Raw byte representation of a MAVLink message of either version
pub enum MAVLinkMessageRaw {
    V1(MAVLinkV1MessageRaw),
    V2(MAVLinkV2MessageRaw),
}

impl MAVLinkMessageRaw {
    pub fn payload(&self) -> &[u8] {
        match self {
            Self::V1(msg) => msg.payload(),
            Self::V2(msg) => msg.payload(),
        }
    }
    pub fn sequence(&self) -> u8 {
        match self {
            Self::V1(msg) => msg.sequence(),
            Self::V2(msg) => msg.sequence(),
        }
    }
    pub fn system_id(&self) -> u8 {
        match self {
            Self::V1(msg) => msg.system_id(),
            Self::V2(msg) => msg.system_id(),
        }
    }
    pub fn component_id(&self) -> u8 {
        match self {
            Self::V1(msg) => msg.component_id(),
            Self::V2(msg) => msg.component_id(),
        }
    }
    pub fn message_id(&self) -> u32 {
        match self {
            Self::V1(msg) => u32::from(msg.message_id()),
            Self::V2(msg) => msg.message_id(),
        }
    }
    pub fn version(&self) -> MavlinkVersion {
        match self {
            Self::V1(_) => MavlinkVersion::V1,
            Self::V2(_) => MavlinkVersion::V2,
        }
    }
}

/// Write a MAVLink message using the given mavlink version to a [`Write`]r.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
pub fn write_versioned_msg<M: Message, W: Write>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg(w, header, data),
        MavlinkVersion::V1 => write_v1_msg(w, header, data),
    }
}

/// Write a MAVLink message using the given mavlink version to a [`Write`]r with signing support.
///
/// When using [`MavlinkVersion::V1`] signing is ignored.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "mav2-message-signing")]
pub fn write_versioned_msg_signed<M: Message, W: Write>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_signed(w, header, data, signing_data),
        MavlinkVersion::V1 => write_v1_msg(w, header, data),
    }
}

/// Asynchronously write a MAVLink message using the given MAVLink version to a [`AsyncWrite`]r.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "tokio")]
pub async fn write_versioned_msg_async<M: Message, W: AsyncWrite + Unpin>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_async(w, header, data).await,
        MavlinkVersion::V1 => write_v1_msg_async(w, header, data).await,
    }
}

/// Asynchronously write a MAVLink message using the given MAVLink version to a [`AsyncWrite`]r with signing support.
///
/// When using [`MavlinkVersion::V1`] signing is ignored.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(all(feature = "tokio", feature = "mav2-message-signing"))]
pub async fn write_versioned_msg_async_signed<M: Message, W: AsyncWrite + Unpin>(
    w: &mut W,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_async_signed(w, header, data, signing_data).await,
        MavlinkVersion::V1 => write_v1_msg_async(w, header, data).await,
    }
}

/// Asynchronously write a MAVLink message using the given MAVLink version to a [`embedded_io_async::Write`]r.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(all(feature = "embedded", not(feature = "std")))]
pub async fn write_versioned_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    version: MavlinkVersion,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    match version {
        MavlinkVersion::V2 => write_v2_msg_async(w, header, data).await,
        MavlinkVersion::V1 => write_v1_msg_async(w, header, data).await,
    }
}

/// Write a MAVLink 2 message to a [`Write`]r.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
pub fn write_v2_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Write a MAVLink 2 message to a [`Write`]r with signing support.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "mav2-message-signing")]
pub fn write_v2_msg_signed<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();

    let signature_len = if let Some(signing_data) = signing_data {
        if signing_data.config.sign_outgoing {
            message_raw.serialize_message_for_signing(header, data);
            signing_data.sign_message(&mut message_raw);
            consts::v2::SIGNATURE_SIZE
        } else {
            message_raw.serialize_message(header, data);
            0
        }
    } else {
        message_raw.serialize_message(header, data);
        0
    };

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE
        + consts::v2::HEADER_SIZE
        + payload_length
        + consts::CHECKSUM_SIZE
        + signature_len;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Asynchronously write a MAVLink 2 message to a [`AsyncWrite`]r.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "tokio")]
pub async fn write_v2_msg_async<M: Message, W: AsyncWrite + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Write a MAVLink 2 message to a [`AsyncWrite`]r with signing support.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "mav2-message-signing")]
#[cfg(feature = "tokio")]
pub async fn write_v2_msg_async_signed<M: Message, W: AsyncWrite + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
    signing_data: Option<&SigningData>,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();

    let signature_len = if let Some(signing_data) = signing_data {
        if signing_data.config.sign_outgoing {
            message_raw.serialize_message_for_signing(header, data);
            signing_data.sign_message(&mut message_raw);
            consts::v2::SIGNATURE_SIZE
        } else {
            message_raw.serialize_message(header, data);
            0
        }
    } else {
        message_raw.serialize_message(header, data);
        0
    };

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE
        + consts::v2::HEADER_SIZE
        + payload_length
        + consts::CHECKSUM_SIZE
        + signature_len;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Asynchronously write a MAVLink 2 message to a [`embedded_io_async::Write`]r.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
///
/// # Errors
///
/// Returns the first error that occurs when writing to the [`embedded_io_async::Write`]r.
#[cfg(all(feature = "embedded", not(feature = "std")))]
pub async fn write_v2_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE + consts::v2::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;

    w.write_all(&message_raw.0[..len])
        .await
        .map_err(|_| MessageWriteError::Io)?;

    Ok(len)
}

/// Write a MAVLink 1 message to a [`Write`]r.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
pub fn write_v1_msg<M: Message, W: Write>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    if data.message_id() > u8::MAX.into() {
        return Err(MessageWriteError::MAVLink2Only);
    }
    let mut message_raw = MAVLinkV1MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Asynchronously write a MAVLink 1 message to a [`AsyncWrite`]r.
///
/// # Errors
///
/// Returns the first error that occurs when writing to the [`AsyncWrite`]r.
#[cfg(feature = "tokio")]
pub async fn write_v1_msg_async<M: Message, W: AsyncWrite + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    if data.message_id() > u8::MAX.into() {
        return Err(MessageWriteError::MAVLink2Only);
    }
    let mut message_raw = MAVLinkV1MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Write a MAVLink 1 message to a [`embedded_io_async::Write`]r.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(all(feature = "embedded", not(feature = "std")))]
pub async fn write_v1_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    if data.message_id() > u8::MAX.into() {
        return Err(MessageWriteError::MAVLink2Only);
    }
    let mut message_raw = MAVLinkV1MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = consts::STX_SIZE + consts::v1::HEADER_SIZE + payload_length + consts::CHECKSUM_SIZE;

    w.write_all(&message_raw.0[..len])
        .await
        .map_err(|_| MessageWriteError::Io)?;

    Ok(len)
}
