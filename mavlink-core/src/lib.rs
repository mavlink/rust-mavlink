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
//! feature for the message sets that it includes. For example, you can use the `ardupilotmega`
//! feature without also using the `uavionix`, `icarous`, `common` features.
//!
//! [ardupilotmega::MavMessage]: https://docs.rs/mavlink/latest/mavlink/ardupilotmega/enum.MavMessage.html
//!
//! # Read Functions
//!
//! The `read_*` functions can be used to read a MAVLink message for a [`PeakReader`] wrapping a `[Read]`er.
//!
//! They follow the pattern `read_(v1|v2|any|versioned)_(raw_message|msg)[_async][_signed]<M, _>(..)`.
//! All read functions check for a valid `STX` marker of the corresponding MAVLink version and verify that the message CRC checksum is correct.
//! They attempt to read until either a whole MAVLink message is read or an error occurrs.
//! While doing so data without STX marker, with an invalid CRC chechsum or invalid signature (if applicable) is discarded.
//! To determine for which dialect the message CRC should be verified it must be specified
//! by using the `Message` enum of the dialect as the generic `M`.
//!
//! Unless further specified all combinations of the function name components exist. The components are described bellow:
//!
//! - `v1` functions read only MAVLink 1 messages
//! - `v2` functions read only MAVLink 2 messages
//! - `any` functions read messages of either MAVLink version
//! - `versioned` functions read messages of the version specified in an aditional `version` parameter
//! - `raw_message` functions return an unparsed message as [`MAVLinkV1MessageRaw`], [`MAVLinkV2MessageRaw`] or [`MAVLinkMessageRaw`]
//! - `msg` functions return a parsed message as a tupel of [`MavHeader`] and the `Message` of the specified dialect
//! - `_async` functions, which are only enabled with the `tokio-1` feature, are [async](https://doc.rust-lang.org/std/keyword.async.html) and read from an [`AsyncPeakReader`] instead.
//! - `_signed` functions, which are only enabled with the `signing` feature, have an `Option<&SigningData>` parameter that allows the use of MAVLink 2 message signing.
//!   MAVLink 1 exclusive functions do not have a `_signed` variant and functions that allow both MAVLink 1 and 2 messages treat MAVLink 1 messages as unsigned.
//!   When an invalidly signed message is received it is ignored.
//!
//! ## Read Errors
//! All `read_` functions return `Result<_,` [`MessageReadError`]`>`.
//!
//! - All functions will return [`MessageReadError::Io`] of [`UnexpectedEof`] when EOF is encountered before a message could be read.
//! - All functions will return [`MessageReadError::Io`] when an error occurs on the underlying [`Read`]er or [`AsyncRead`]er.
//!   
//! - Functions that parse the received message will return [`MessageReadError::Parse`] when the read data could
//!   not be parsed as a MAVLink message
//!
//! # Write Functions
//!
//! The `write_` functions are used to write a MAVLink to a [`Write`]r.
//! They follow the pattern `write_(v1|v2|versioned)_msg[_async][_signed](..)`:
//!
//! - `v1` functions write messages using MAVLink 1 serialisation
//! - `v2` functions write messages using MAVLink 2 serialisation
//! - `versioned` functions write messages using the version specified in an aditional `version` parameter
//! - `_async` functions, which are only enabled with the `tokio-1` feature, are
//!   [async](https://doc.rust-lang.org/std/keyword.async.html) and write from an [`tokio::io::AsyncWrite`]r instead.
//! - `_signed` functions, which are only enabled with the `signing` feature, have an `Option<&SigningData>` parameter that allows the use of MAVLink 2 message signing.
//!
//! ## Write errors
//!
//! All `write_` functions return `Result<_,` [`MessageWriteError`]`>`.
//!
//! - When an error occurs on the underlying [`Write`]er or [`AsyncWrite`]er other then
//!   [`Interrupted`] the function returns [`MessageWriteError::Io`]
//! - When attempting to serialize a message with an ID over 255 with MAVLink 1 a [`MessageWriteError::MAVLink2Only`] is returned
//!
//! [`PeakReader`]: peek_reader::PeekReader
//! [`AsyncPeakReader`]: async_peek_reader::AsyncPeekReader
//! [`UnexpectedEof`]: std::io::ErrorKind::UnexpectedEof
//! [`AsyncRead`]: tokio::io::AsyncRead
//! [`AsyncWrite`]: tokio::io::AsyncWrite
//! [`Interrupted`]: std::io::ErrorKind::Interrupted
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
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

use crate::{
    bytes::Bytes,
    error::{MessageReadError, MessageWriteError, ParserError},
};

use crc_any::CRCu16;

#[doc(hidden)]
pub mod bytes;
#[doc(hidden)]
pub mod bytes_mut;
#[cfg(feature = "std")]
mod connection;
pub mod error;
pub mod types;
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
#[cfg(feature = "tokio-1")]
use tokio::io::{AsyncWrite, AsyncWriteExt};

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

#[cfg(feature = "arbitrary")]
use arbitrary::Arbitrary;

#[cfg(any(feature = "std", feature = "tokio-1"))]
mod connectable;

#[cfg(any(feature = "std", feature = "tokio-1"))]
pub use connectable::ConnectionAddress;

#[cfg(feature = "direct-serial")]
pub use connection::direct_serial::config::SerialConfig;

#[cfg(feature = "tcp")]
pub use connection::tcp::config::{TcpConfig, TcpMode};

#[cfg(feature = "udp")]
pub use connection::udp::config::{UdpConfig, UdpMode};

#[cfg(feature = "std")]
pub use connection::file::config::FileConfig;

/// Maximum size of any MAVLink frame in bytes.
///
/// This is a v2 frame with maximum payload size and a signature: <https://mavlink.io/en/guide/serialization.html>
pub const MAX_FRAME_SIZE: usize = 280;

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
    fn random_message_from_id<R: rand::RngCore>(id: u32, rng: &mut R) -> Option<Self>;
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
    /// [`ParserError::InvalidFlag`] on an invalid bitflag value
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
    /// Will panic if frame does not fit in the provided buffer.
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
    /// The input buffer should start with the sequence field of the MAVLink frame. The
    /// initial packet marker, length field, and flag fields should be excluded.
    ///
    /// # Panics
    ///
    /// Will panic if the buffer provided does not contain a full message
    ///
    /// # Errors
    ///
    /// Will return a [`ParserError`] if a message was found but could not be parsed  
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

/// Calculates the [CRC checksum](https://mavlink.io/en/guide/serialization.html#checksum) of a messages header, payload and the CRC_EXTRA byte.
pub fn calculate_crc(data: &[u8], extra_crc: u8) -> u16 {
    let mut crc_calculator = CRCu16::crc16mcrf4cc();
    crc_calculator.digest(data);

    crc_calculator.digest(&[extra_crc]);
    crc_calculator.get_crc()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// MAVLink Version selection when attempting to read
pub enum ReadVersion {
    /// Only attempt to read using a single MAVLink version
    Single(MavlinkVersion),
    /// Attempt to read messages from both MAVLink versions
    Any,
}

impl ReadVersion {
    #[cfg(feature = "std")]
    fn from_conn_cfg<C: MavConnection<M>, M: Message>(conn: &C) -> Self {
        if conn.allow_recv_any_version() {
            Self::Any
        } else {
            conn.protocol_version().into()
        }
    }
    #[cfg(feature = "tokio-1")]
    fn from_async_conn_cfg<C: AsyncMavConnection<M>, M: Message + Sync + Send>(conn: &C) -> Self {
        if conn.allow_recv_any_version() {
            Self::Any
        } else {
            conn.protocol_version().into()
        }
    }
}

impl From<MavlinkVersion> for ReadVersion {
    fn from(value: MavlinkVersion) -> Self {
        Self::Single(value)
    }
}

/// Read and parse a MAVLink message of the specified version from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
pub fn read_versioned_msg<M: Message, R: Read>(
    r: &mut PeekReader<R>,
    version: ReadVersion,
) -> Result<(MavHeader, M), MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => read_v2_msg(r),
        ReadVersion::Single(MavlinkVersion::V1) => read_v1_msg(r),
        ReadVersion::Any => read_any_msg(r),
    }
}

/// Read and parse a MAVLink message of the specified version from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
pub fn read_versioned_raw_message<M: Message, R: Read>(
    r: &mut PeekReader<R>,
    version: ReadVersion,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => {
            Ok(MAVLinkMessageRaw::V2(read_v2_raw_message::<M, _>(r)?))
        }
        ReadVersion::Single(MavlinkVersion::V1) => {
            Ok(MAVLinkMessageRaw::V1(read_v1_raw_message::<M, _>(r)?))
        }
        ReadVersion::Any => read_any_raw_message::<M, _>(r),
    }
}

/// Asynchronously read and parse a MAVLink message of the specified version from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_versioned_msg_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    r: &mut AsyncPeekReader<R>,
    version: ReadVersion,
) -> Result<(MavHeader, M), MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => read_v2_msg_async(r).await,
        ReadVersion::Single(MavlinkVersion::V1) => read_v1_msg_async(r).await,
        ReadVersion::Any => read_any_msg_async(r).await,
    }
}

/// Asynchronously read and parse a MAVLinkMessageRaw of the specified version from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_versioned_raw_message_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    r: &mut AsyncPeekReader<R>,
    version: ReadVersion,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => Ok(MAVLinkMessageRaw::V2(
            read_v2_raw_message_async::<M, _>(r).await?,
        )),
        ReadVersion::Single(MavlinkVersion::V1) => Ok(MAVLinkMessageRaw::V1(
            read_v1_raw_message_async::<M, _>(r).await?,
        )),
        ReadVersion::Any => read_any_raw_message_async::<M, _>(r).await,
    }
}

/// Read and parse a MAVLinkMessageRaw of the specified version from a [`PeekReader`] with signing support.
///
/// When using [`ReadVersion::Single`]`(`[`MavlinkVersion::V1`]`)` signing is ignored.
/// When using [`ReadVersion::Any`] MAVlink 1 messages are treated as unsigned.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "signing")]
pub fn read_versioned_raw_message_signed<M: Message, R: Read>(
    r: &mut PeekReader<R>,
    version: ReadVersion,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => Ok(MAVLinkMessageRaw::V2(
            read_v2_raw_message_inner::<M, _>(r, signing_data)?,
        )),
        ReadVersion::Single(MavlinkVersion::V1) => {
            Ok(MAVLinkMessageRaw::V1(read_v1_raw_message::<M, _>(r)?))
        }
        ReadVersion::Any => read_any_raw_message_inner::<M, _>(r, signing_data),
    }
}

/// Read and parse a MAVLink message of the specified version from a [`PeekReader`] with signing support.
///
/// When using [`ReadVersion::Single`]`(`[`MavlinkVersion::V1`]`)` signing is ignored.
/// When using [`ReadVersion::Any`] MAVlink 1 messages are treated as unsigned.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "signing")]
pub fn read_versioned_msg_signed<M: Message, R: Read>(
    r: &mut PeekReader<R>,
    version: ReadVersion,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => read_v2_msg_inner(r, signing_data),
        ReadVersion::Single(MavlinkVersion::V1) => read_v1_msg(r),
        ReadVersion::Any => read_any_msg_inner(r, signing_data),
    }
}

/// Asynchronously read and parse a MAVLinkMessageRaw of the specified version from a [`AsyncPeekReader`] with signing support.
///
/// When using [`ReadVersion::Single`]`(`[`MavlinkVersion::V1`]`)` signing is ignored.
/// When using [`ReadVersion::Any`] MAVlink 1 messages are treated as unsigned.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_versioned_raw_message_async_signed<
    M: Message,
    R: tokio::io::AsyncRead + Unpin,
>(
    r: &mut AsyncPeekReader<R>,
    version: ReadVersion,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => Ok(MAVLinkMessageRaw::V2(
            read_v2_raw_message_async_inner::<M, _>(r, signing_data).await?,
        )),
        ReadVersion::Single(MavlinkVersion::V1) => Ok(MAVLinkMessageRaw::V1(
            read_v1_raw_message_async::<M, _>(r).await?,
        )),
        ReadVersion::Any => read_any_raw_message_async_inner::<M, _>(r, signing_data).await,
    }
}

/// Asynchronously read and parse a MAVLink message of the specified version from a [`AsyncPeekReader`] with signing support.
///
/// When using [`ReadVersion::Single`]`(`[`MavlinkVersion::V1`]`)` signing is ignored.
/// When using [`ReadVersion::Any`] MAVlink 1 messages are treated as unsigned.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_versioned_msg_async_signed<M: Message, R: tokio::io::AsyncRead + Unpin>(
    r: &mut AsyncPeekReader<R>,
    version: ReadVersion,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    match version {
        ReadVersion::Single(MavlinkVersion::V2) => read_v2_msg_async_inner(r, signing_data).await,
        ReadVersion::Single(MavlinkVersion::V1) => read_v1_msg_async(r).await,
        ReadVersion::Any => read_any_msg_async_inner(r, signing_data).await,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Byte buffer containing the raw representation of a MAVLink 1 message beginning with the STX marker.
///
/// Follow protocol definition: <https://mavlink.io/en/guide/serialization.html#v1_packet_format>.
/// Maximum size is 263 bytes.
pub struct MAVLinkV1MessageRaw([u8; 1 + Self::HEADER_SIZE + 255 + 2]);

impl Default for MAVLinkV1MessageRaw {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkV1MessageRaw {
    const HEADER_SIZE: usize = 5;

    /// Create a new raw MAVLink 1 message filled with zeros.
    pub const fn new() -> Self {
        Self([0; 1 + Self::HEADER_SIZE + 255 + 2])
    }

    /// Create a new raw MAVLink 1 message from a given buffer.
    ///
    /// Note: This method does not guarantee that the constructed MAVLink message is valid.
    pub const fn from_bytes_unparsed(bytes: [u8; 1 + Self::HEADER_SIZE + 255 + 2]) -> Self {
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
    pub fn into_inner(self) -> [u8; 1 + Self::HEADER_SIZE + 255 + 2] {
        self.0
    }

    /// Reference to the 5 byte header slice of the message
    #[inline]
    pub fn header(&self) -> &[u8] {
        &self.0[1..=Self::HEADER_SIZE]
    }

    /// Mutable reference to the 5 byte header slice of the message
    #[inline]
    fn mut_header(&mut self) -> &mut [u8] {
        &mut self.0[1..=Self::HEADER_SIZE]
    }

    /// Size of the payload of the message
    #[inline]
    pub fn payload_length(&self) -> u8 {
        self.0[1]
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
        &self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + payload_length)]
    }

    /// [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) field of the message
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

    /// Checks wether the message’s [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) calculation matches its checksum field.
    #[inline]
    pub fn has_valid_crc<M: Message>(&self) -> bool {
        let payload_length: usize = self.payload_length().into();
        self.checksum()
            == calculate_crc(
                &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
                M::extra_crc(self.message_id().into()),
            )
    }

    /// Raw byte slice of the message
    pub fn raw_bytes(&self) -> &[u8] {
        let payload_length = self.payload_length() as usize;
        &self.0[..(1 + Self::HEADER_SIZE + payload_length + 2)]
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
        self.0[0] = MAV_STX;

        let header_buf = self.mut_header();
        header_buf.copy_from_slice(&[
            payload_length as u8,
            header.sequence,
            header.system_id,
            header.component_id,
            msgid.try_into().unwrap(),
        ]);

        let crc = calculate_crc(
            &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
            extra_crc,
        );
        self.0[(1 + Self::HEADER_SIZE + payload_length)
            ..(1 + Self::HEADER_SIZE + payload_length + 2)]
            .copy_from_slice(&crc.to_le_bytes());
    }

    /// Serialize a [`Message`] with a given header into this raw message buffer.
    ///
    /// # Panics
    ///
    /// If the message's id exceeds 255 and is therefore not supported for MAVLink 1
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

    /// # Panics
    ///
    /// If the `MessageData`'s `ID` exceeds 255 and is therefore not supported for MAVLink 1
    pub fn serialize_message_data<D: MessageData>(&mut self, header: MavHeader, message_data: &D) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message_data.ser(MavlinkVersion::V1, payload_buf);

        self.serialize_stx_and_header_and_crc(header, D::ID, payload_length, D::EXTRA_CRC);
    }
}

fn try_decode_v1<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
) -> Result<Option<MAVLinkV1MessageRaw>, MessageReadError> {
    let mut message = MAVLinkV1MessageRaw::new();
    let whole_header_size = MAVLinkV1MessageRaw::HEADER_SIZE + 1;

    message.0[0] = MAV_STX;
    let header = &reader.peek_exact(whole_header_size)?[1..whole_header_size];
    message.mut_header().copy_from_slice(header);
    let packet_length = message.raw_bytes().len();
    let payload_and_checksum = &reader.peek_exact(packet_length)?[whole_header_size..packet_length];
    message
        .mut_payload_and_checksum()
        .copy_from_slice(payload_and_checksum);

    // retry if CRC failed after previous STX
    // (an STX byte may appear in the middle of a message)
    if message.has_valid_crc::<M>() {
        reader.consume(message.raw_bytes().len());
        Ok(Some(message))
    } else {
        Ok(None)
    }
}

#[cfg(feature = "tokio-1")]
// other then the blocking version the STX is read not peeked, this changed some sizes
async fn try_decode_v1_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
) -> Result<Option<MAVLinkV1MessageRaw>, MessageReadError> {
    let mut message = MAVLinkV1MessageRaw::new();

    message.0[0] = MAV_STX;
    let header = &reader.peek_exact(MAVLinkV1MessageRaw::HEADER_SIZE).await?
        [..MAVLinkV1MessageRaw::HEADER_SIZE];
    message.mut_header().copy_from_slice(header);
    let packet_length = message.raw_bytes().len() - 1;
    let payload_and_checksum =
        &reader.peek_exact(packet_length).await?[MAVLinkV1MessageRaw::HEADER_SIZE..packet_length];
    message
        .mut_payload_and_checksum()
        .copy_from_slice(payload_and_checksum);

    // retry if CRC failed after previous STX
    // (an STX byte may appear in the middle of a message)
    if message.has_valid_crc::<M>() {
        reader.consume(message.raw_bytes().len() - 1);
        Ok(Some(message))
    } else {
        Ok(None)
    }
}

/// Read a raw MAVLink 1 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
pub fn read_v1_raw_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
) -> Result<MAVLinkV1MessageRaw, MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        while reader.peek_exact(1)?[0] != MAV_STX {
            reader.consume(1);
        }

        if let Some(msg) = try_decode_v1::<M, _>(reader)? {
            return Ok(msg);
        }

        reader.consume(1);
    }
}

/// Asynchronously read a raw MAVLink 1 message from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_v1_raw_message_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
) -> Result<MAVLinkV1MessageRaw, MessageReadError> {
    loop {
        loop {
            // search for the magic framing value indicating start of mavlink message
            if reader.read_u8().await? == MAV_STX {
                break;
            }
        }

        if let Some(message) = try_decode_v1_async::<M, _>(reader).await? {
            return Ok(message);
        }
    }
}

/// Async read a raw buffer with the mavlink message
/// V1 maximum size is 263 bytes: `<https://mavlink.io/en/guide/serialization.html>`
///
/// # Example
///
/// See mavlink/examples/embedded-async-read full example for details.
#[cfg(feature = "embedded")]
pub async fn read_v1_raw_message_async<M: Message>(
    reader: &mut impl embedded_io_async::Read,
) -> Result<MAVLinkV1MessageRaw, MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        let mut byte = [0u8];
        loop {
            reader
                .read_exact(&mut byte)
                .await
                .map_err(|_| MessageReadError::Io)?;
            if byte[0] == MAV_STX {
                break;
            }
        }

        let mut message = MAVLinkV1MessageRaw::new();

        message.0[0] = MAV_STX;
        reader
            .read_exact(message.mut_header())
            .await
            .map_err(|_| MessageReadError::Io)?;
        reader
            .read_exact(message.mut_payload_and_checksum())
            .await
            .map_err(|_| MessageReadError::Io)?;

        // retry if CRC failed after previous STX
        // (an STX byte may appear in the middle of a message)
        if message.has_valid_crc::<M>() {
            return Ok(message);
        }
    }
}

/// Read and parse a MAVLink 1 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
pub fn read_v1_msg<M: Message, R: Read>(
    r: &mut PeekReader<R>,
) -> Result<(MavHeader, M), MessageReadError> {
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

/// Asynchronously read and parse a MAVLink 1 message from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_v1_msg_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    r: &mut AsyncPeekReader<R>,
) -> Result<(MavHeader, M), MessageReadError> {
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

/// Asynchronously read and parse a MAVLink 1 message from a [`embedded_io_async::Read`]er.
///
/// NOTE: it will be add ~80KB to firmware flash size because all *_DATA::deser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn read_v1_msg_async<M: Message>(
    r: &mut impl embedded_io_async::Read,
) -> Result<(MavHeader, M), MessageReadError> {
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
/// Byte buffer containing the raw representation of a MAVLink 2 message beginning with the STX marker.
///
/// Follow protocol definition: <https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format>.
/// Maximum size is [280 bytes](MAX_FRAME_SIZE).
pub struct MAVLinkV2MessageRaw([u8; 1 + Self::HEADER_SIZE + 255 + 2 + Self::SIGNATURE_SIZE]);

impl Default for MAVLinkV2MessageRaw {
    fn default() -> Self {
        Self::new()
    }
}

impl MAVLinkV2MessageRaw {
    const HEADER_SIZE: usize = 9;
    const SIGNATURE_SIZE: usize = 13;

    /// Create a new raw MAVLink 2 message filled with zeros.
    pub const fn new() -> Self {
        Self([0; 1 + Self::HEADER_SIZE + 255 + 2 + Self::SIGNATURE_SIZE])
    }

    /// Create a new raw MAVLink 1 message from a given buffer.
    ///
    /// Note: This method does not guarantee that the constructed MAVLink message is valid.
    pub const fn from_bytes_unparsed(
        bytes: [u8; 1 + Self::HEADER_SIZE + 255 + 2 + Self::SIGNATURE_SIZE],
    ) -> Self {
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
    pub fn into_inner(self) -> [u8; 1 + Self::HEADER_SIZE + 255 + 2 + Self::SIGNATURE_SIZE] {
        self.0
    }

    /// Reference to the 9 byte header slice of the message
    #[inline]
    pub fn header(&self) -> &[u8] {
        &self.0[1..=Self::HEADER_SIZE]
    }

    /// Mutable reference to the header byte slice of the message
    #[inline]
    fn mut_header(&mut self) -> &mut [u8] {
        &mut self.0[1..=Self::HEADER_SIZE]
    }

    /// Size of the payload of the message
    #[inline]
    pub fn payload_length(&self) -> u8 {
        self.0[1]
    }

    /// [Incompatiblity flags](https://mavlink.io/en/guide/serialization.html#incompat_flags) of the message
    ///
    /// Currently the only supported incompatebility flag is `MAVLINK_IFLAG_SIGNED`.
    #[inline]
    pub fn incompatibility_flags(&self) -> u8 {
        self.0[2]
    }

    /// Mutable reference to the [incompatiblity flags](https://mavlink.io/en/guide/serialization.html#incompat_flags) of the message
    ///
    /// Currently the only supported incompatebility flag is `MAVLINK_IFLAG_SIGNED`.
    #[inline]
    pub fn incompatibility_flags_mut(&mut self) -> &mut u8 {
        &mut self.0[2]
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
        &self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + payload_length)]
    }

    /// [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) field of the message
    #[inline]
    pub fn checksum(&self) -> u16 {
        let payload_length: usize = self.payload_length().into();
        u16::from_le_bytes([
            self.0[1 + Self::HEADER_SIZE + payload_length],
            self.0[1 + Self::HEADER_SIZE + payload_length + 1],
        ])
    }

    /// Reference to the 2 checksum bytes of the message
    #[cfg(feature = "signing")]
    #[inline]
    pub fn checksum_bytes(&self) -> &[u8] {
        let checksum_offset = 1 + Self::HEADER_SIZE + self.payload_length() as usize;
        &self.0[checksum_offset..(checksum_offset + 2)]
    }

    /// Signature [Link ID](https://mavlink.io/en/guide/message_signing.html#link_ids)
    ///
    /// If the message is not signed this 0.
    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_link_id(&self) -> u8 {
        let payload_length: usize = self.payload_length().into();
        self.0[1 + Self::HEADER_SIZE + payload_length + 2]
    }

    /// Mutable reference to the signature [Link ID](https://mavlink.io/en/guide/message_signing.html#link_ids)
    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_link_id_mut(&mut self) -> &mut u8 {
        let payload_length: usize = self.payload_length().into();
        &mut self.0[1 + Self::HEADER_SIZE + payload_length + 2]
    }

    /// Message [signature timestamp](https://mavlink.io/en/guide/message_signing.html#timestamp)
    ///
    /// The timestamp is a 48 bit number with units of 10 microseconds since 1st January 2015 GMT.
    /// The offset since 1st January 1970 (the unix epoch) is 1420070400 seconds.
    /// Since all timestamps generated must be at least 1 more than the previous timestamp this timestamp may get ahead of GMT time if there is a burst of packets at a rate of more than 100000 packets per second.
    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_timestamp(&self) -> u64 {
        let mut timestamp_bytes = [0u8; 8];
        timestamp_bytes[0..6].copy_from_slice(self.signature_timestamp_bytes());
        u64::from_le_bytes(timestamp_bytes)
    }

    /// 48 bit [signature timestamp](https://mavlink.io/en/guide/message_signing.html#timestamp) byte slice
    ///
    /// If the message is not signed this contains zeros.
    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_timestamp_bytes(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let timestamp_start = 1 + Self::HEADER_SIZE + payload_length + 3;
        &self.0[timestamp_start..(timestamp_start + 6)]
    }

    /// Mutable reference to the 48 bit signature timestams byte slice
    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_timestamp_bytes_mut(&mut self) -> &mut [u8] {
        let payload_length: usize = self.payload_length().into();
        let timestamp_start = 1 + Self::HEADER_SIZE + payload_length + 3;
        &mut self.0[timestamp_start..(timestamp_start + 6)]
    }

    /// Reference to the 48 bit [message signature](https://mavlink.io/en/guide/message_signing.html#signature) byte slice
    ///
    /// If the message is not signed this contains zeros.
    #[cfg(feature = "signing")]
    #[inline]
    pub fn signature_value(&self) -> &[u8] {
        let payload_length: usize = self.payload_length().into();
        let signature_start = 1 + Self::HEADER_SIZE + payload_length + 3 + 6;
        &self.0[signature_start..(signature_start + 6)]
    }

    /// Mutable reference to the 48 bit [message signature](https://mavlink.io/en/guide/message_signing.html#signature) byte slice
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

    /// Checks wether the message's [CRC-16 checksum](https://mavlink.io/en/guide/serialization.html#checksum) calculation matches its checksum field.
    #[inline]
    pub fn has_valid_crc<M: Message>(&self) -> bool {
        let payload_length: usize = self.payload_length().into();
        self.checksum()
            == calculate_crc(
                &self.0[1..(1 + Self::HEADER_SIZE + payload_length)],
                M::extra_crc(self.message_id()),
            )
    }

    /// Calculates the messages sha256_48 signature.
    ///
    /// This calculates the [SHA-256](https://en.wikipedia.org/wiki/SHA-2) checksum of messages appended to the 32 byte secret key and copies the first 6 bytes of the result into the target buffer.
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

    /// Raw byte slice of the message
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

    /// Serialize a [Message] with a given header into this raw message buffer.
    ///
    /// This does not set any compatiblity or incompatiblity flags.
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

    /// Serialize a [Message] with a given header into this raw message buffer and sets the `MAVLINK_IFLAG_SIGNED` incompatiblity flag.
    ///
    /// This does not update the message's signature fields.
    /// This does not set any compatiblity flags.
    pub fn serialize_message_for_signing<M: Message>(&mut self, header: MavHeader, message: &M) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message.ser(MavlinkVersion::V2, payload_buf);

        let message_id = message.message_id();
        self.serialize_stx_and_header_and_crc(
            header,
            message_id,
            payload_length,
            M::extra_crc(message_id),
            MAVLINK_IFLAG_SIGNED,
        );
    }

    pub fn serialize_message_data<D: MessageData>(&mut self, header: MavHeader, message_data: &D) {
        let payload_buf = &mut self.0[(1 + Self::HEADER_SIZE)..(1 + Self::HEADER_SIZE + 255)];
        let payload_length = message_data.ser(MavlinkVersion::V2, payload_buf);

        self.serialize_stx_and_header_and_crc(header, D::ID, payload_length, D::EXTRA_CRC, 0);
    }
}

#[allow(unused_variables)]
fn try_decode_v2<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<Option<MAVLinkV2MessageRaw>, MessageReadError> {
    let mut message = MAVLinkV2MessageRaw::new();
    let whole_header_size = MAVLinkV2MessageRaw::HEADER_SIZE + 1;

    message.0[0] = MAV_STX_V2;
    let header = &reader.peek_exact(whole_header_size)?[1..whole_header_size];
    message.mut_header().copy_from_slice(header);

    if message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
        // if there are incompatibility flags set that we do not know discard the message
        reader.consume(1);
        return Ok(None);
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
        return Ok(None);
    }

    #[cfg(feature = "signing")]
    if let Some(signing_data) = signing_data {
        if !signing_data.verify_signature(&message) {
            return Ok(None);
        }
    }

    Ok(Some(message))
}

#[cfg(feature = "tokio-1")]
#[allow(unused_variables)]
// other then the blocking version the STX is read not peeked, this changed some sizes
async fn try_decode_v2_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<Option<MAVLinkV2MessageRaw>, MessageReadError> {
    let mut message = MAVLinkV2MessageRaw::new();

    message.0[0] = MAV_STX_V2;
    let header = &reader.peek_exact(MAVLinkV2MessageRaw::HEADER_SIZE).await?
        [..MAVLinkV2MessageRaw::HEADER_SIZE];
    message.mut_header().copy_from_slice(header);

    if message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
        // if there are incompatibility flags set that we do not know discard the message
        return Ok(None);
    }

    let packet_length = message.raw_bytes().len() - 1;
    let payload_and_checksum_and_sign =
        &reader.peek_exact(packet_length).await?[MAVLinkV2MessageRaw::HEADER_SIZE..packet_length];
    message
        .mut_payload_and_checksum_and_sign()
        .copy_from_slice(payload_and_checksum_and_sign);

    if message.has_valid_crc::<M>() {
        // even if the signature turn out to be invalid the valid crc shows that the received data presents a valid message as opposed to random bytes
        reader.consume(message.raw_bytes().len() - 1);
    } else {
        return Ok(None);
    }

    #[cfg(feature = "signing")]
    if let Some(signing_data) = signing_data {
        if !signing_data.verify_signature(&message) {
            return Ok(None);
        }
    }

    Ok(Some(message))
}

/// Read a raw MAVLink 2 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[inline]
pub fn read_v2_raw_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    read_v2_raw_message_inner::<M, R>(reader, None)
}

/// Read a raw MAVLink 2 message with signing support from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "signing")]
#[inline]
pub fn read_v2_raw_message_signed<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    read_v2_raw_message_inner::<M, R>(reader, signing_data)
}

#[allow(unused_variables)]
fn read_v2_raw_message_inner<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        while reader.peek_exact(1)?[0] != MAV_STX_V2 {
            reader.consume(1);
        }

        if let Some(message) = try_decode_v2::<M, _>(reader, signing_data)? {
            return Ok(message);
        }
    }
}

/// Asynchronously read a raw MAVLink 2 message from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_v2_raw_message_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    read_v2_raw_message_async_inner::<M, R>(reader, None).await
}

#[cfg(feature = "tokio-1")]
#[allow(unused_variables)]
async fn read_v2_raw_message_async_inner<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    loop {
        loop {
            // search for the magic framing value indicating start of mavlink message
            if reader.read_u8().await? == MAV_STX_V2 {
                break;
            }
        }

        if let Some(message) = try_decode_v2_async::<M, _>(reader, signing_data).await? {
            return Ok(message);
        }
    }
}

/// Asynchronously read a raw MAVLink 2 message with signing support from a [`AsyncPeekReader`]
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_v2_raw_message_async_signed<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    read_v2_raw_message_async_inner::<M, R>(reader, signing_data).await
}

/// Asynchronously read a raw MAVLink 2 message with signing support from a [`embedded_io_async::Read`]er.
///
/// # Example
///
/// See mavlink/examples/embedded-async-read full example for details.
#[cfg(feature = "embedded")]
pub async fn read_v2_raw_message_async<M: Message>(
    reader: &mut impl embedded_io_async::Read,
) -> Result<MAVLinkV2MessageRaw, MessageReadError> {
    loop {
        // search for the magic framing value indicating start of mavlink message
        let mut byte = [0u8];
        loop {
            reader
                .read_exact(&mut byte)
                .await
                .map_err(|_| MessageReadError::Io)?;
            if byte[0] == MAV_STX_V2 {
                break;
            }
        }

        let mut message = MAVLinkV2MessageRaw::new();

        message.0[0] = MAV_STX_V2;
        reader
            .read_exact(message.mut_header())
            .await
            .map_err(|_| MessageReadError::Io)?;

        if message.incompatibility_flags() & !MAVLINK_SUPPORTED_IFLAGS > 0 {
            // if there are incompatibility flags set that we do not know discard the message
            continue;
        }

        reader
            .read_exact(message.mut_payload_and_checksum_and_sign())
            .await
            .map_err(|_| MessageReadError::Io)?;

        // retry if CRC failed after previous STX
        // (an STX byte may appear in the middle of a message)
        if message.has_valid_crc::<M>() {
            return Ok(message);
        }
    }
}

/// Read and parse a MAVLink 2 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[inline]
pub fn read_v2_msg<M: Message, R: Read>(
    read: &mut PeekReader<R>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_v2_msg_inner(read, None)
}

/// Read and parse a MAVLink 2 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "signing")]
#[inline]
pub fn read_v2_msg_signed<M: Message, R: Read>(
    read: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_v2_msg_inner(read, signing_data)
}

fn read_v2_msg_inner<M: Message, R: Read>(
    read: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
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

/// Asynchronously read and parse a MAVLink 2 message from a [`AsyncPeekReader`].
///  
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_v2_msg_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    read: &mut AsyncPeekReader<R>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_v2_msg_async_inner(read, None).await
}

/// Asynchronously read and parse a MAVLink 2 message with signing support from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_v2_msg_async_signed<M: Message, R: tokio::io::AsyncRead + Unpin>(
    read: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_v2_msg_async_inner(read, signing_data).await
}

#[cfg(feature = "tokio-1")]
async fn read_v2_msg_async_inner<M: Message, R: tokio::io::AsyncRead + Unpin>(
    read: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
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

/// Asynchronously and parse read a MAVLink 2 message from a [`embedded_io_async::Read`]er.
///
/// NOTE: it will be add ~80KB to firmware flash size because all *_DATA::deser methods will be add to firmware.
/// Use `*_DATA::deser` methods manually to prevent it.
#[cfg(feature = "embedded")]
pub async fn read_v2_msg_async<M: Message, R: embedded_io_async::Read>(
    r: &mut R,
) -> Result<(MavHeader, M), MessageReadError> {
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

/// Read a raw MAVLink 1 or 2 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[inline]
pub fn read_any_raw_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    read_any_raw_message_inner::<M, R>(reader, None)
}

/// Read a raw MAVLink 1 or 2 message from a [`PeekReader`] with signing support.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "signing")]
#[inline]
pub fn read_any_raw_message_signed<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    read_any_raw_message_inner::<M, R>(reader, signing_data)
}

#[allow(unused_variables)]
fn read_any_raw_message_inner<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    loop {
        // search for the magic framing value indicating start of MAVLink message
        let version = loop {
            let byte = reader.peek_exact(1)?[0];
            if byte == MAV_STX {
                break MavlinkVersion::V1;
            }
            if byte == MAV_STX_V2 {
                break MavlinkVersion::V2;
            }
            reader.consume(1);
        };
        match version {
            MavlinkVersion::V1 => {
                if let Some(message) = try_decode_v1::<M, _>(reader)? {
                    // With signing enabled and unsigned messages not allowed do not further process V1
                    #[cfg(feature = "signing")]
                    if let Some(signing) = signing_data {
                        if signing.config.allow_unsigned {
                            return Ok(MAVLinkMessageRaw::V1(message));
                        }
                    } else {
                        return Ok(MAVLinkMessageRaw::V1(message));
                    }
                    #[cfg(not(feature = "signing"))]
                    return Ok(MAVLinkMessageRaw::V1(message));
                }

                reader.consume(1);
            }
            MavlinkVersion::V2 => {
                if let Some(message) = try_decode_v2::<M, _>(reader, signing_data)? {
                    return Ok(MAVLinkMessageRaw::V2(message));
                }
            }
        }
    }
}

/// Asynchronously read a raw MAVLink 1 or 2 message from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_any_raw_message_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    read_any_raw_message_async_inner::<M, R>(reader, None).await
}

/// Asynchronously read a raw MAVLink 1 or 2 message from a [`AsyncPeekReader`] with signing support.
///
/// This will attempt to read until encounters a valid message or an error.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(all(feature = "tokio-1", feature = "signing"))]
pub async fn read_any_raw_message_async_signed<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    read_any_raw_message_async_inner::<M, R>(reader, signing_data).await
}

#[cfg(feature = "tokio-1")]
#[allow(unused_variables)]
async fn read_any_raw_message_async_inner<M: Message, R: tokio::io::AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    loop {
        // search for the magic framing value indicating start of MAVLink 1 or 2 message
        let version = loop {
            let read = reader.read_u8().await?;
            if read == MAV_STX {
                break MavlinkVersion::V1;
            }
            if read == MAV_STX_V2 {
                break MavlinkVersion::V2;
            }
        };

        match version {
            MavlinkVersion::V1 => {
                if let Some(message) = try_decode_v1_async::<M, _>(reader).await? {
                    // With signing enabled and unsigned messages not allowed do not further process them
                    #[cfg(feature = "signing")]
                    if let Some(signing) = signing_data {
                        if signing.config.allow_unsigned {
                            return Ok(MAVLinkMessageRaw::V1(message));
                        }
                    } else {
                        return Ok(MAVLinkMessageRaw::V1(message));
                    }
                    #[cfg(not(feature = "signing"))]
                    return Ok(MAVLinkMessageRaw::V1(message));
                }
            }
            MavlinkVersion::V2 => {
                if let Some(message) = try_decode_v2_async::<M, _>(reader, signing_data).await? {
                    return Ok(MAVLinkMessageRaw::V2(message));
                }
            }
        }
    }
}

/// Read and parse a MAVLink 1 or 2 message from a [`PeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[inline]
pub fn read_any_msg<M: Message, R: Read>(
    read: &mut PeekReader<R>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_any_msg_inner(read, None)
}

/// Read and parse a MAVLink 1 or 2 message from a [`PeekReader`] with signing support.
///
/// MAVLink 1 messages a treated as unsigned.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "signing")]
#[inline]
pub fn read_any_msg_signed<M: Message, R: Read>(
    read: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_any_msg_inner(read, signing_data)
}

fn read_any_msg_inner<M: Message, R: Read>(
    read: &mut PeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    let message = read_any_raw_message_inner::<M, _>(read, signing_data)?;
    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(message.version(), message.message_id(), message.payload())?,
    ))
}

/// Asynchronously read and parse a MAVLink 1 or 2 message from a [`AsyncPeekReader`].
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(feature = "tokio-1")]
pub async fn read_any_msg_async<M: Message, R: tokio::io::AsyncRead + Unpin>(
    read: &mut AsyncPeekReader<R>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_any_msg_async_inner(read, None).await
}

/// Asynchronously read and parse a MAVLink 1 or 2 message from a [`AsyncPeekReader`] with signing support.
///
/// MAVLink 1 messages a treated as unsigned.
///
/// # Errors
///
/// See [`read_` function error documentation](crate#read-errors)
#[cfg(all(feature = "tokio-1", feature = "signing"))]
#[inline]
pub async fn read_any_msg_async_signed<M: Message, R: tokio::io::AsyncRead + Unpin>(
    read: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    read_any_msg_async_inner(read, signing_data).await
}

#[cfg(feature = "tokio-1")]
async fn read_any_msg_async_inner<M: Message, R: tokio::io::AsyncRead + Unpin>(
    read: &mut AsyncPeekReader<R>,
    signing_data: Option<&SigningData>,
) -> Result<(MavHeader, M), MessageReadError> {
    let message = read_any_raw_message_async_inner::<M, _>(read, signing_data).await?;

    Ok((
        MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        },
        M::parse(message.version(), message.message_id(), message.payload())?,
    ))
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
#[cfg(feature = "signing")]
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
#[cfg(feature = "tokio-1")]
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
#[cfg(all(feature = "tokio-1", feature = "signing"))]
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
#[cfg(feature = "embedded")]
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
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Write a MAVLink 2 message to a [`Write`]r with signing support.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "signing")]
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

/// Asynchronously write a MAVLink 2 message to a [`AsyncWrite`]r.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "tokio-1")]
pub async fn write_v2_msg_async<M: Message, W: AsyncWrite + Unpin>(
    w: &mut W,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Write a MAVLink 2 message to a [`AsyncWrite`]r with signing support.
///
/// # Errors
///
/// See [`write_` function error documentation](crate#write-errors).
#[cfg(feature = "signing")]
#[cfg(feature = "tokio-1")]
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

/// Asynchronously write a MAVLink 2 message to a [`embedded_io_async::Write`]r.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
///
/// # Errors
///
/// Returns the first error that occurs when writing to the [`embedded_io_async::Write`]r.
#[cfg(feature = "embedded")]
pub async fn write_v2_msg_async<M: Message>(
    w: &mut impl embedded_io_async::Write,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    let mut message_raw = MAVLinkV2MessageRaw::new();
    message_raw.serialize_message(header, data);

    let payload_length: usize = message_raw.payload_length().into();
    let len = 1 + MAVLinkV2MessageRaw::HEADER_SIZE + payload_length + 2;

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
    let len = 1 + MAVLinkV1MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])?;

    Ok(len)
}

/// Asynchronously write a MAVLink 1 message to a [`AsyncWrite`]r.
///
/// # Errors
///
/// Returns the first error that occurs when writing to the [`AsyncWrite`]r.
#[cfg(feature = "tokio-1")]
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
    let len = 1 + MAVLinkV1MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len]).await?;

    Ok(len)
}

/// Write a MAVLink 1 message to a [`embedded_io_async::Write`]r.
///
/// NOTE: it will be add ~70KB to firmware flash size because all *_DATA::ser methods will be add to firmware.
/// Use `*_DATA::ser` methods manually to prevent it.
#[cfg(feature = "embedded")]
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
    let len = 1 + MAVLinkV1MessageRaw::HEADER_SIZE + payload_length + 2;

    w.write_all(&message_raw.0[..len])
        .await
        .map_err(|_| MessageWriteError::Io)?;

    Ok(len)
}
