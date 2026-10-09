//! Incremental blocking MAVLink reader.

use crate::{
    MAVLinkMessageRaw, MAVLinkUnverifiedFrame, MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader,
    MavlinkVersion, Message, SigningData,
    error::MessageReadError,
    frame_decoder::{FrameDecoder, FrameRef, VersionFilter},
};

#[cfg(all(feature = "embedded", not(feature = "std")))]
use embedded_io::Read;
#[cfg(feature = "std")]
use std::io::Read;

/// Incrementally reads complete MAVLink frames from a blocking byte stream.
///
/// The reader keeps partial and read-ahead data between calls. Use the same
/// MAVLink dialect `M` for the lifetime of a stream, because CRC validation is
/// dialect-specific.
pub struct MavlinkReader<R> {
    source: R,
    decoder: FrameDecoder,
}

impl<R> MavlinkReader<R> {
    /// Creates a reader around `source` with the default read-ahead capacity.
    ///
    /// The buffer is allocated once and reused for the reader's lifetime.
    #[cfg(feature = "std")]
    pub fn new(source: R) -> Self
    where
        R: Read,
    {
        Self {
            source,
            decoder: FrameDecoder::new(),
        }
    }

    /// Creates an allocation-free reader around `source`.
    #[cfg(all(feature = "embedded", not(feature = "std")))]
    pub const fn new(source: R) -> Self
    where
        R: Read,
    {
        Self {
            source,
            decoder: FrameDecoder::new(),
        }
    }

    /// Creates a reader with at least `capacity` bytes of read-ahead space.
    ///
    /// Capacities smaller than one maximum-size MAVLink frame are raised to
    /// [`consts::MAX_FRAME_SIZE`](crate::consts::MAX_FRAME_SIZE). The buffer is
    /// allocated once and reused for the lifetime of the reader.
    #[cfg(feature = "std")]
    pub fn with_capacity(capacity: usize, source: R) -> Self
    where
        R: Read,
    {
        Self {
            source,
            decoder: FrameDecoder::with_capacity(capacity),
        }
    }

    /// Returns a shared reference to the underlying source.
    pub const fn get_ref(&self) -> &R {
        &self.source
    }

    /// Returns a mutable reference to the underlying source.
    ///
    /// Reading directly from the source can skip bytes already buffered by
    /// this reader and should therefore be avoided.
    pub const fn get_mut(&mut self) -> &mut R {
        &mut self.source
    }

    /// Returns the underlying source.
    ///
    /// Any bytes read ahead by this reader are discarded.
    pub fn into_inner(self) -> R {
        self.source
    }
}

#[cfg(any(feature = "std", all(feature = "embedded", not(feature = "std"))))]
impl<R: Read> MavlinkReader<R> {
    /// Reads the next complete frame accepted by `version` without verifying it.
    /// The returned bytes must be treated as untrusted until validated.
    pub fn read_unverified_frame(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<MAVLinkUnverifiedFrame, MessageReadError> {
        self.read_unverified_frame_inner(VersionFilter::Exact(version))
    }

    /// Reads the next complete MAVLink 1 or 2 frame without verifying it.
    pub fn read_any_unverified_frame(
        &mut self,
    ) -> Result<MAVLinkUnverifiedFrame, MessageReadError> {
        self.read_unverified_frame_inner(VersionFilter::Any)
    }

    fn read_unverified_frame_inner(
        &mut self,
        filter: VersionFilter,
    ) -> Result<MAVLinkUnverifiedFrame, MessageReadError> {
        loop {
            if let Some(frame) = try_decode_unverified(&mut self.decoder, filter) {
                return Ok(frame);
            }
            self.read_more()?;
        }
    }

    /// Reads and parses the next CRC-valid message accepted by `version`.
    pub fn read_message<M: Message>(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Exact(version), None)
    }

    /// Reads and parses the next CRC-valid MAVLink 1 or MAVLink 2 message.
    pub fn read_any_message<M: Message>(&mut self) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Any, None)
    }

    /// Reads the next CRC-valid raw message accepted by `version`.
    pub fn read_raw_message<M: Message>(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Exact(version), None)
    }

    /// Reads the next CRC-valid MAVLink 1 or MAVLink 2 raw message.
    pub fn read_any_raw_message<M: Message>(
        &mut self,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Any, None)
    }

    /// Reads, verifies, and parses the next message accepted by `version`.
    ///
    /// With signing data, MAVLink 1 and unsigned MAVLink 2 frames require
    /// `SigningConfig::allow_unsigned`; signed MAVLink 2 frames must have a valid
    /// signature. With `None`, signature verification is skipped.
    #[cfg(feature = "mav2-message-signing")]
    pub fn read_message_signed<M: Message>(
        &mut self,
        version: MavlinkVersion,
        signing_data: Option<&SigningData>,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Exact(version), signing_data)
    }

    /// Reads, verifies, and parses the next MAVLink 1 or MAVLink 2 message.
    /// With signing data, MAVLink 1 frames follow `SigningConfig::allow_unsigned`.
    /// With `None`, signature verification is skipped.
    #[cfg(feature = "mav2-message-signing")]
    pub fn read_any_message_signed<M: Message>(
        &mut self,
        signing_data: Option<&SigningData>,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Any, signing_data)
    }

    /// Reads and verifies the next raw message accepted by `version`.
    ///
    /// With signing data, MAVLink 1 and unsigned MAVLink 2 frames require
    /// `SigningConfig::allow_unsigned`; signed MAVLink 2 frames must have a valid
    /// signature. With `None`, signature verification is skipped.
    #[cfg(feature = "mav2-message-signing")]
    pub fn read_raw_message_signed<M: Message>(
        &mut self,
        version: MavlinkVersion,
        signing_data: Option<&SigningData>,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Exact(version), signing_data)
    }

    /// Reads and verifies the next MAVLink 1 or MAVLink 2 raw message.
    /// With signing data, MAVLink 1 frames follow `SigningConfig::allow_unsigned`.
    /// With `None`, signature verification is skipped.
    #[cfg(feature = "mav2-message-signing")]
    pub fn read_any_raw_message_signed<M: Message>(
        &mut self,
        signing_data: Option<&SigningData>,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Any, signing_data)
    }

    #[inline]
    pub(crate) fn read_message_inner<M: Message>(
        &mut self,
        filter: VersionFilter,
        signing_data: Option<&SigningData>,
    ) -> Result<(MavHeader, M), MessageReadError> {
        loop {
            if let Some(result) = try_decode_message::<M>(&mut self.decoder, filter, signing_data) {
                return result;
            }
            self.read_more()?;
        }
    }

    #[inline]
    pub(crate) fn read_raw_message_inner<M: Message>(
        &mut self,
        filter: VersionFilter,
        signing_data: Option<&SigningData>,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        loop {
            if let Some(message) =
                try_decode_raw_message::<M>(&mut self.decoder, filter, signing_data)
            {
                return Ok(message);
            }
            self.read_more()?;
        }
    }

    fn read_more(&mut self) -> Result<(), MessageReadError> {
        let destination = self.decoder.spare_capacity_mut();
        assert!(
            !destination.is_empty(),
            "a pending MAVLink frame must fit in the decoder buffer"
        );

        #[cfg(feature = "std")]
        let count = loop {
            match self.source.read(destination) {
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                result => break result?,
            }
        };

        #[cfg(all(feature = "embedded", not(feature = "std")))]
        let count = self
            .source
            .read(destination)
            .map_err(|_| MessageReadError::Io)?;

        if count == 0 {
            return Err(MessageReadError::eof());
        }
        self.decoder.commit(count);
        Ok(())
    }
}

#[inline]
pub(crate) fn try_decode_unverified(
    decoder: &mut FrameDecoder,
    filter: VersionFilter,
) -> Option<MAVLinkUnverifiedFrame> {
    let meta = decoder.next_unverified_frame(filter)?;
    let frame = decoder.frame(meta);
    let unverified = match frame.version() {
        MavlinkVersion::V1 => MAVLinkUnverifiedFrame::V1(MAVLinkV1MessageRaw::from(frame)),
        MavlinkVersion::V2 => MAVLinkUnverifiedFrame::V2(MAVLinkV2MessageRaw::from(frame)),
    };
    decoder.advance(meta);
    Some(unverified)
}

#[inline]
// The loop retries after a rejected signature when signing is enabled.
#[cfg_attr(not(feature = "mav2-message-signing"), allow(clippy::never_loop))]
pub(crate) fn try_decode_message<M: Message>(
    decoder: &mut FrameDecoder,
    filter: VersionFilter,
    signing_data: Option<&SigningData>,
) -> Option<Result<(MavHeader, M), MessageReadError>> {
    loop {
        let meta = decoder.next_frame::<M>(filter)?;

        #[cfg(feature = "mav2-message-signing")]
        if let Some(signing_data) = signing_data {
            if !signature_is_valid(decoder.frame(meta), signing_data) {
                decoder.advance(meta);
                continue;
            }
        }

        #[cfg(not(feature = "mav2-message-signing"))]
        let _ = signing_data;

        let (header, parsed) = {
            let frame = decoder.frame(meta);
            (
                frame_header(frame),
                M::parse(frame.version(), frame.message_id(), frame.payload()),
            )
        };
        decoder.advance(meta);
        return Some(parsed.map(|message| (header, message)).map_err(Into::into));
    }
}

#[inline]
// The loop retries after a rejected signature when signing is enabled.
#[cfg_attr(not(feature = "mav2-message-signing"), allow(clippy::never_loop))]
pub(crate) fn try_decode_raw_message<M: Message>(
    decoder: &mut FrameDecoder,
    filter: VersionFilter,
    signing_data: Option<&SigningData>,
) -> Option<MAVLinkMessageRaw> {
    loop {
        let meta = decoder.next_frame::<M>(filter)?;
        let frame = decoder.frame(meta);

        let message = match frame.version() {
            MavlinkVersion::V1 => MAVLinkMessageRaw::V1(MAVLinkV1MessageRaw::from(frame)),
            MavlinkVersion::V2 => MAVLinkMessageRaw::V2(MAVLinkV2MessageRaw::from(frame)),
        };

        #[cfg(feature = "mav2-message-signing")]
        if let Some(signing_data) = signing_data {
            if !raw_signature_is_valid(&message, signing_data) {
                decoder.advance(meta);
                continue;
            }
        }

        #[cfg(not(feature = "mav2-message-signing"))]
        let _ = signing_data;

        decoder.advance(meta);
        return Some(message);
    }
}

#[inline]
fn frame_header(frame: FrameRef<'_>) -> MavHeader {
    MavHeader {
        sequence: frame.sequence(),
        system_id: frame.system_id(),
        component_id: frame.component_id(),
    }
}

#[cfg(feature = "mav2-message-signing")]
fn signature_is_valid(frame: FrameRef<'_>, signing_data: &SigningData) -> bool {
    match frame.version() {
        MavlinkVersion::V1 => signing_data.config.allow_unsigned,
        MavlinkVersion::V2 => signing_data.verify_signature(&MAVLinkV2MessageRaw::from(frame)),
    }
}

#[cfg(feature = "mav2-message-signing")]
fn raw_signature_is_valid(message: &MAVLinkMessageRaw, signing_data: &SigningData) -> bool {
    match message {
        MAVLinkMessageRaw::V1(_) => signing_data.config.allow_unsigned,
        MAVLinkMessageRaw::V2(message) => signing_data.verify_signature(message),
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use std::io;

    #[derive(Debug, PartialEq, Eq)]
    struct TestMessage(u8);

    impl Message for TestMessage {
        fn message_id(&self) -> u32 {
            0
        }

        fn message_name(&self) -> &'static str {
            "TEST"
        }

        fn target_system_id(&self) -> Option<u8> {
            None
        }

        fn target_component_id(&self) -> Option<u8> {
            None
        }

        fn ser(&self, _: MavlinkVersion, bytes: &mut [u8]) -> usize {
            bytes[0] = self.0;
            1
        }

        fn parse(_: MavlinkVersion, _: u32, payload: &[u8]) -> Result<Self, crate::ParserError> {
            Ok(Self(payload[0]))
        }

        fn message_id_from_name(_: &str) -> Option<u32> {
            None
        }

        fn default_message_from_id(_: u32) -> Option<Self> {
            None
        }

        #[cfg(feature = "arbitrary")]
        fn random_message_from_id<R: rand::TryRng<Error = core::convert::Infallible>>(
            _: u32,
            _: &mut R,
        ) -> Option<Self> {
            None
        }

        fn extra_crc(_: u32) -> u8 {
            77
        }
    }

    struct CountingReader {
        bytes: std::vec::Vec<u8>,
        offset: usize,
        reads: usize,
    }

    impl Read for CountingReader {
        fn read(&mut self, destination: &mut [u8]) -> io::Result<usize> {
            self.reads += 1;
            let count = destination.len().min(self.bytes.len() - self.offset);
            destination[..count].copy_from_slice(&self.bytes[self.offset..self.offset + count]);
            self.offset += count;
            Ok(count)
        }
    }

    #[test]
    fn standard_reader_capacity_is_configurable_and_frame_safe() {
        let mut default_reader = MavlinkReader::new(io::empty());
        assert_eq!(
            default_reader.decoder.spare_capacity_mut().len(),
            crate::consts::DEFAULT_READ_BUFFER_CAPACITY
        );

        let mut minimum_reader = MavlinkReader::with_capacity(1, io::empty());
        assert_eq!(
            minimum_reader.decoder.spare_capacity_mut().len(),
            crate::consts::MAX_FRAME_SIZE
        );
    }

    #[test]
    fn reads_multiple_frames_per_source_read() {
        let header = MavHeader {
            sequence: 1,
            system_id: 2,
            component_id: 3,
        };
        let mut bytes = std::vec::Vec::new();
        for value in 0..16 {
            let mut raw = MAVLinkV2MessageRaw::new();
            raw.serialize_message(header, &TestMessage(value));
            bytes.extend_from_slice(raw.raw_bytes());
        }

        let mut reader = MavlinkReader::new(CountingReader {
            bytes,
            offset: 0,
            reads: 0,
        });
        for value in 0..16 {
            let (_, message) = reader.read_any_message::<TestMessage>().unwrap();
            assert_eq!(message, TestMessage(value));
        }
        assert_eq!(reader.get_ref().reads, 1);
    }
}
