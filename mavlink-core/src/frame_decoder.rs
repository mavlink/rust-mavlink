//! Incremental MAVLink frame boundary detection.

use crate::{MAV_STX, MAV_STX_V2, MavlinkVersion, Message, calculate_crc, consts};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VersionFilter {
    Exact(MavlinkVersion),
    Any,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct FrameMeta {
    pub(crate) version: MavlinkVersion,
    pub(crate) len: usize,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct FrameRef<'a> {
    bytes: &'a [u8],
    version: MavlinkVersion,
}

impl<'a> FrameRef<'a> {
    #[inline]
    pub(crate) fn bytes(self) -> &'a [u8] {
        self.bytes
    }

    #[inline]
    pub(crate) fn version(self) -> MavlinkVersion {
        self.version
    }

    #[inline]
    pub(crate) fn message_id(self) -> u32 {
        match self.version {
            MavlinkVersion::V1 => u32::from(self.bytes[5]),
            MavlinkVersion::V2 => {
                u32::from_le_bytes([self.bytes[7], self.bytes[8], self.bytes[9], 0])
            }
        }
    }

    #[inline]
    pub(crate) fn sequence(self) -> u8 {
        match self.version {
            MavlinkVersion::V1 => self.bytes[2],
            MavlinkVersion::V2 => self.bytes[4],
        }
    }

    #[inline]
    pub(crate) fn system_id(self) -> u8 {
        match self.version {
            MavlinkVersion::V1 => self.bytes[3],
            MavlinkVersion::V2 => self.bytes[5],
        }
    }

    #[inline]
    pub(crate) fn component_id(self) -> u8 {
        match self.version {
            MavlinkVersion::V1 => self.bytes[4],
            MavlinkVersion::V2 => self.bytes[6],
        }
    }

    #[inline]
    pub(crate) fn payload(self) -> &'a [u8] {
        let start = match self.version {
            MavlinkVersion::V1 => consts::STX_SIZE + consts::v1::HEADER_SIZE,
            MavlinkVersion::V2 => consts::STX_SIZE + consts::v2::HEADER_SIZE,
        };
        &self.bytes[start..start + usize::from(self.bytes[consts::PAYLOAD_LEN_OFFSET])]
    }
}

/// Decoder shared by blocking and asynchronous readers.
///
/// The buffer can hold at least one maximum-size MAVLink frame. Input may
/// arrive in arbitrary chunks; complete trailing frames remain buffered for
/// the next call. Invalid candidates are retried from their second byte so an
/// embedded framing marker cannot be lost.
pub(crate) struct FrameDecoder {
    #[cfg(feature = "std")]
    buffer: Box<[u8]>,
    #[cfg(not(feature = "std"))]
    buffer: [u8; consts::MAX_FRAME_SIZE],
    start: usize,
    end: usize,
}

impl FrameDecoder {
    #[cfg(feature = "std")]
    pub(crate) fn new() -> Self {
        Self::with_capacity(consts::DEFAULT_READ_BUFFER_CAPACITY)
    }

    #[cfg(not(feature = "std"))]
    pub(crate) const fn new() -> Self {
        Self {
            buffer: [0; consts::MAX_FRAME_SIZE],
            start: 0,
            end: 0,
        }
    }

    #[cfg(feature = "std")]
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            buffer: vec![0; capacity.max(consts::MAX_FRAME_SIZE)].into_boxed_slice(),
            start: 0,
            end: 0,
        }
    }

    /// Returns the largest contiguous destination available for read-ahead.
    #[inline]
    pub(crate) fn spare_capacity_mut(&mut self) -> &mut [u8] {
        self.compact();
        &mut self.buffer[self.end..]
    }

    #[inline]
    pub(crate) fn commit(&mut self, count: usize) {
        assert!(count <= self.buffer.len() - self.end);
        self.end += count;
    }

    #[inline]
    pub(crate) fn next_frame<M: Message>(&mut self, filter: VersionFilter) -> Option<FrameMeta> {
        loop {
            self.seek_marker(filter)?;

            let version = marker_version(self.buffer[self.start])?;
            let candidate = &self.buffer[self.start..self.end];
            let header_prefix = match version {
                MavlinkVersion::V1 => 2,
                MavlinkVersion::V2 => 3,
            };
            if candidate.len() < header_prefix {
                return None;
            }

            if version == MavlinkVersion::V2
                && candidate[consts::v2::INCOMPAT_FLAGS_OFFSET] & !consts::v2::SUPPORTED_IFLAGS != 0
            {
                self.reject_candidate();
                continue;
            }

            let payload_len = usize::from(candidate[consts::PAYLOAD_LEN_OFFSET]);
            let signature_len = if version == MavlinkVersion::V2
                && candidate[consts::v2::INCOMPAT_FLAGS_OFFSET] & consts::v2::IFLAG_SIGNED != 0
            {
                consts::v2::SIGNATURE_SIZE
            } else {
                0
            };
            let header_len = match version {
                MavlinkVersion::V1 => consts::v1::HEADER_SIZE,
                MavlinkVersion::V2 => consts::v2::HEADER_SIZE,
            };
            let frame_len =
                consts::STX_SIZE + header_len + payload_len + consts::CHECKSUM_SIZE + signature_len;
            if candidate.len() < frame_len {
                return None;
            }

            let checksum_offset = consts::STX_SIZE + header_len + payload_len;
            let expected =
                u16::from_le_bytes([candidate[checksum_offset], candidate[checksum_offset + 1]]);
            let message_id = match version {
                MavlinkVersion::V1 => u32::from(candidate[5]),
                MavlinkVersion::V2 => {
                    u32::from_le_bytes([candidate[7], candidate[8], candidate[9], 0])
                }
            };
            let actual = calculate_crc(
                &candidate[consts::STX_SIZE..checksum_offset],
                M::extra_crc(message_id),
            );

            if actual == expected {
                return Some(FrameMeta {
                    version,
                    len: frame_len,
                });
            }

            self.reject_candidate();
        }
    }

    #[inline]
    pub(crate) fn frame(&self, meta: FrameMeta) -> FrameRef<'_> {
        FrameRef {
            bytes: &self.buffer[self.start..self.start + meta.len],
            version: meta.version,
        }
    }

    #[inline]
    pub(crate) fn advance(&mut self, meta: FrameMeta) {
        self.start += meta.len;
        if self.start == self.end {
            self.start = 0;
            self.end = 0;
        }
    }

    fn seek_marker(&mut self, filter: VersionFilter) -> Option<()> {
        let offset = find_marker(&self.buffer[self.start..self.end], filter);
        match offset {
            Some(offset) => {
                self.start += offset;
                Some(())
            }
            None => {
                self.start = 0;
                self.end = 0;
                None
            }
        }
    }

    #[inline]
    fn reject_candidate(&mut self) {
        self.start += 1;
    }

    fn compact(&mut self) {
        if self.start == 0 {
            return;
        }
        self.buffer.copy_within(self.start..self.end, 0);
        self.end -= self.start;
        self.start = 0;
    }
}

#[inline]
fn find_marker(bytes: &[u8], filter: VersionFilter) -> Option<usize> {
    bytes.iter().position(|&byte| match filter {
        VersionFilter::Exact(MavlinkVersion::V1) => byte == MAV_STX,
        VersionFilter::Exact(MavlinkVersion::V2) => byte == MAV_STX_V2,
        VersionFilter::Any => byte == MAV_STX || byte == MAV_STX_V2,
    })
}

const fn marker_version(marker: u8) -> Option<MavlinkVersion> {
    match marker {
        MAV_STX => Some(MavlinkVersion::V1),
        MAV_STX_V2 => Some(MavlinkVersion::V2),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    struct TestMessage;

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
        fn ser(&self, _: crate::MavlinkVersion, _: &mut [u8]) -> usize {
            0
        }
        fn parse(
            _: crate::MavlinkVersion,
            _: u32,
            _: &[u8],
        ) -> Result<Self, crate::error::ParserError> {
            Ok(Self)
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

    fn v2_frame(payload: &[u8]) -> std::vec::Vec<u8> {
        let mut frame = std::vec![MAV_STX_V2, payload.len() as u8, 0, 0, 1, 2, 3, 0, 0, 0];
        frame.extend_from_slice(payload);
        let crc = calculate_crc(&frame[1..], TestMessage::extra_crc(0));
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    #[test]
    fn decodes_fragmented_and_adjacent_frames() {
        let first = v2_frame(&[1, 2, 3]);
        let second = v2_frame(&[4, 5]);
        let mut input = std::vec![9, 8, 7];
        input.extend_from_slice(&first);
        input.extend_from_slice(&second);

        for chunk_size in 1..=input.len() {
            let mut decoder = FrameDecoder::new();
            let mut offset = 0;
            let mut payloads = std::vec::Vec::new();
            while offset < input.len() {
                let amount = chunk_size
                    .min(input.len() - offset)
                    .min(decoder.spare_capacity_mut().len());
                decoder.spare_capacity_mut()[..amount]
                    .copy_from_slice(&input[offset..offset + amount]);
                decoder.commit(amount);
                offset += amount;

                while let Some(meta) = decoder.next_frame::<TestMessage>(VersionFilter::Any) {
                    payloads.push(decoder.frame(meta).payload().to_vec());
                    decoder.advance(meta);
                }
            }
            assert_eq!(payloads, std::vec![std::vec![1, 2, 3], std::vec![4, 5]]);
        }
    }

    #[test]
    fn recovers_from_invalid_candidate_with_embedded_frame() {
        let valid = v2_frame(&[42]);
        let mut input = v2_frame(&[0; 32]);
        input[10..10 + valid.len()].copy_from_slice(&valid);

        let mut decoder = FrameDecoder::new();
        decoder.spare_capacity_mut()[..input.len()].copy_from_slice(&input);
        decoder.commit(input.len());
        let meta = decoder
            .next_frame::<TestMessage>(VersionFilter::Any)
            .expect("embedded frame");
        assert_eq!(decoder.frame(meta).payload(), [42]);
    }

    #[test]
    fn maximum_signed_frame_always_makes_progress() {
        let mut frame = v2_frame(&[0; consts::MAX_PAYLOAD_LEN]);
        frame[consts::v2::INCOMPAT_FLAGS_OFFSET] = consts::v2::IFLAG_SIGNED;
        let checksum_offset = consts::STX_SIZE + consts::v2::HEADER_SIZE + consts::MAX_PAYLOAD_LEN;
        let crc = calculate_crc(&frame[1..checksum_offset], TestMessage::extra_crc(0));
        frame[checksum_offset..checksum_offset + consts::CHECKSUM_SIZE]
            .copy_from_slice(&crc.to_le_bytes());
        frame.extend_from_slice(&[0; consts::v2::SIGNATURE_SIZE]);
        assert_eq!(frame.len(), consts::MAX_FRAME_SIZE);

        let mut decoder = FrameDecoder::new();
        decoder.spare_capacity_mut()[..frame.len()].copy_from_slice(&frame);
        decoder.commit(frame.len());
        let meta = decoder
            .next_frame::<TestMessage>(VersionFilter::Any)
            .expect("maximum frame");
        assert_eq!(meta.len, consts::MAX_FRAME_SIZE);
    }
}
