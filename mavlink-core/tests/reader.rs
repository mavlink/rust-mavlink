//! Incremental reader behavior at the public API boundary.

#![cfg(any(feature = "std", feature = "tokio"))]

use mavlink_core::{
    MAV_STX, MAV_STX_V2, MAVLinkV1MessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion,
    Message, error::ParserError,
};

const HEADER: MavHeader = MavHeader {
    system_id: 11,
    component_id: 22,
    sequence: 33,
};

#[derive(Debug, PartialEq, Eq)]
struct TestMessage(u8);

impl Message for TestMessage {
    fn message_id(&self) -> u32 {
        42
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

    fn parse(_: MavlinkVersion, message_id: u32, payload: &[u8]) -> Result<Self, ParserError> {
        if message_id != 42 {
            return Err(ParserError::UnknownMessage { id: message_id });
        }
        let value = payload[0];
        if value == u8::MAX {
            return Err(ParserError::InvalidEnum {
                enum_type: "Test",
                value: value.into(),
            });
        }
        Ok(Self(value))
    }

    fn message_id_from_name(name: &str) -> Option<u32> {
        (name == "TEST").then_some(42)
    }

    fn default_message_from_id(message_id: u32) -> Option<Self> {
        (message_id == 42).then_some(Self(0))
    }

    #[cfg(feature = "arbitrary")]
    fn random_message_from_id<R: rand::TryRng<Error = core::convert::Infallible>>(
        message_id: u32,
        _: &mut R,
    ) -> Option<Self> {
        Self::default_message_from_id(message_id)
    }

    fn extra_crc(message_id: u32) -> u8 {
        if message_id == 42 { 91 } else { 0 }
    }
}

fn v1_frame(value: u8) -> MAVLinkV1MessageRaw {
    let mut frame = MAVLinkV1MessageRaw::new();
    frame.serialize_message(HEADER, &TestMessage(value));
    frame
}

fn v2_frame(value: u8) -> MAVLinkV2MessageRaw {
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message(HEADER, &TestMessage(value));
    frame
}

#[cfg(feature = "std")]
mod blocking {
    use std::io::{self, Read};

    use mavlink_core::{
        MavlinkReader,
        error::{MessageReadError, ParserError},
    };

    use super::{HEADER, MAV_STX, MAV_STX_V2, TestMessage, v1_frame, v2_frame};

    struct ChunkedReader {
        bytes: Vec<u8>,
        offset: usize,
        chunk_size: usize,
    }

    impl Read for ChunkedReader {
        fn read(&mut self, destination: &mut [u8]) -> io::Result<usize> {
            let count = destination
                .len()
                .min(self.chunk_size)
                .min(self.bytes.len() - self.offset);
            destination[..count].copy_from_slice(&self.bytes[self.offset..self.offset + count]);
            self.offset += count;
            Ok(count)
        }
    }

    #[test]
    fn arbitrary_chunks_and_adjacent_versions() {
        let v1 = v1_frame(7);
        let v2 = v2_frame(8);
        let mut bytes = vec![0, 1, 2];
        bytes.extend_from_slice(v1.raw_bytes());
        bytes.extend_from_slice(v2.raw_bytes());

        let mut reader = MavlinkReader::new(ChunkedReader {
            bytes,
            offset: 0,
            chunk_size: 1,
        });
        assert_eq!(
            reader.read_any_message::<TestMessage>().unwrap(),
            (HEADER, TestMessage(7))
        );
        assert_eq!(
            reader.read_any_message::<TestMessage>().unwrap(),
            (HEADER, TestMessage(8))
        );
    }

    struct IntermittentReader {
        bytes: Vec<u8>,
        offset: usize,
        would_block: bool,
    }

    impl Read for IntermittentReader {
        fn read(&mut self, destination: &mut [u8]) -> io::Result<usize> {
            if self.would_block {
                self.would_block = false;
                return Err(io::ErrorKind::WouldBlock.into());
            }
            self.would_block = true;
            let count = destination.len().min(3).min(self.bytes.len() - self.offset);
            destination[..count].copy_from_slice(&self.bytes[self.offset..self.offset + count]);
            self.offset += count;
            Ok(count)
        }
    }

    #[test]
    fn partial_frame_survives_io_errors() {
        let frame = v2_frame(9);
        let mut reader = MavlinkReader::new(IntermittentReader {
            bytes: frame.raw_bytes().to_vec(),
            offset: 0,
            would_block: false,
        });

        loop {
            match reader.read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2) {
                Ok(message) => {
                    assert_eq!(message, (HEADER, TestMessage(9)));
                    break;
                }
                Err(MessageReadError::Io(error)) if error.kind() == io::ErrorKind::WouldBlock => {}
                Err(error) => panic!("unexpected read error: {error}"),
            }
        }
    }

    #[test]
    fn rejected_frames_resynchronize_and_parse_errors_consume_one_frame() {
        let valid = v2_frame(10);
        let mut unsupported = valid.raw_bytes().to_vec();
        unsupported[2] = 0x80;
        let mut invalid_crc = valid.raw_bytes().to_vec();
        *invalid_crc.last_mut().unwrap() ^= 1;
        let invalid_value = v2_frame(u8::MAX);

        let mut bytes = unsupported;
        bytes.extend_from_slice(&invalid_crc);
        bytes.extend_from_slice(invalid_value.raw_bytes());
        bytes.extend_from_slice(valid.raw_bytes());
        let mut reader = MavlinkReader::new(bytes.as_slice());

        assert!(matches!(
            reader.read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2),
            Err(MessageReadError::Parse(ParserError::InvalidEnum { .. }))
        ));
        assert_eq!(
            reader
                .read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2)
                .unwrap(),
            (HEADER, TestMessage(10))
        );
    }

    #[test]
    fn exact_version_ignores_incomplete_other_version_candidate() {
        let v1 = v1_frame(13);
        let mut before_v1 = vec![MAV_STX_V2, u8::MAX, 0];
        before_v1.extend_from_slice(v1.raw_bytes());
        assert_eq!(
            MavlinkReader::new(before_v1.as_slice())
                .read_message::<TestMessage>(mavlink_core::MavlinkVersion::V1)
                .unwrap(),
            (HEADER, TestMessage(13))
        );

        let v2 = v2_frame(14);
        let mut before_v2 = vec![MAV_STX, u8::MAX];
        before_v2.extend_from_slice(v2.raw_bytes());
        assert_eq!(
            MavlinkReader::new(before_v2.as_slice())
                .read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2)
                .unwrap(),
            (HEADER, TestMessage(14))
        );
    }
}

#[cfg(feature = "tokio")]
mod asynchronous {
    use core::{future::Future, pin::Pin, task::Poll};
    use std::{io, task::Context};

    use mavlink_core::AsyncMavlinkReader;
    use tokio::io::{AsyncRead, ReadBuf};

    use super::{HEADER, MAV_STX, MAV_STX_V2, TestMessage, v1_frame, v2_frame};

    struct PausingReader {
        bytes: Vec<u8>,
        offset: usize,
        paused: bool,
    }

    impl AsyncRead for PausingReader {
        fn poll_read(
            mut self: Pin<&mut Self>,
            context: &mut Context<'_>,
            destination: &mut ReadBuf<'_>,
        ) -> Poll<io::Result<()>> {
            if self.offset == 5 && !self.paused {
                self.paused = true;
                context.waker().wake_by_ref();
                return Poll::Pending;
            }

            let count = if self.offset < 5 {
                5 - self.offset
            } else {
                self.bytes.len() - self.offset
            }
            .min(destination.remaining());
            destination.put_slice(&self.bytes[self.offset..self.offset + count]);
            self.offset += count;
            Poll::Ready(Ok(()))
        }
    }

    #[tokio::test]
    async fn partial_frame_survives_cancellation() {
        let frame = v2_frame(12);
        let mut reader = AsyncMavlinkReader::new(PausingReader {
            bytes: frame.raw_bytes().to_vec(),
            offset: 0,
            paused: false,
        });

        let mut first_attempt =
            Box::pin(reader.read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2));
        let mut context = Context::from_waker(std::task::Waker::noop());
        assert!(matches!(
            first_attempt.as_mut().poll(&mut context),
            Poll::Pending
        ));
        drop(first_attempt);

        assert_eq!(
            reader
                .read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2)
                .await
                .unwrap(),
            (HEADER, TestMessage(12))
        );
    }

    #[tokio::test]
    async fn exact_version_ignores_incomplete_other_version_candidate() {
        let v1 = v1_frame(15);
        let mut before_v1 = vec![MAV_STX_V2, u8::MAX, 0];
        before_v1.extend_from_slice(v1.raw_bytes());
        assert_eq!(
            AsyncMavlinkReader::new(before_v1.as_slice())
                .read_message::<TestMessage>(mavlink_core::MavlinkVersion::V1)
                .await
                .unwrap(),
            (HEADER, TestMessage(15))
        );

        let v2 = v2_frame(16);
        let mut before_v2 = vec![MAV_STX, u8::MAX];
        before_v2.extend_from_slice(v2.raw_bytes());
        assert_eq!(
            AsyncMavlinkReader::new(before_v2.as_slice())
                .read_message::<TestMessage>(mavlink_core::MavlinkVersion::V2)
                .await
                .unwrap(),
            (HEADER, TestMessage(16))
        );
    }
}
