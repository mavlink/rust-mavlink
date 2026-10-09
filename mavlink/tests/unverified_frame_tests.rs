#![cfg(feature = "dialect-common")]

use mavlink::{
    MAV_STX, MAV_STX_V2, MAVLinkUnverifiedFrame, MAVLinkV2MessageRaw, MavHeader, MavlinkReader,
    MavlinkVersion, Message, calculate_crc, dialects::common::MavMessage,
    error::FrameValidationErrorKind,
};

const UNKNOWN_V1_ID: u8 = 255;
const UNKNOWN_V2_ID: u32 = 0x00_ff_fe;
const UNKNOWN_CRC_EXTRA: u8 = 123;
const PAYLOAD: &[u8] = &[0x12, 0xfe, 0xfd, 0x34];

fn unknown_v1_frame() -> Vec<u8> {
    assert!(MavMessage::default_message_from_id(UNKNOWN_V1_ID.into()).is_none());
    let mut frame = vec![MAV_STX, PAYLOAD.len() as u8, 42, 7, 8, UNKNOWN_V1_ID];
    frame.extend_from_slice(PAYLOAD);
    let checksum = calculate_crc(&frame[1..], UNKNOWN_CRC_EXTRA);
    frame.extend_from_slice(&checksum.to_le_bytes());
    frame
}

fn unknown_v2_frame() -> Vec<u8> {
    unknown_v2_frame_with_payload(PAYLOAD)
}

fn unknown_v2_frame_with_payload(payload: &[u8]) -> Vec<u8> {
    assert!(MavMessage::default_message_from_id(UNKNOWN_V2_ID).is_none());
    let id = UNKNOWN_V2_ID.to_le_bytes();
    let mut frame = vec![
        MAV_STX_V2,
        payload.len() as u8,
        0,
        0,
        43,
        9,
        10,
        id[0],
        id[1],
        id[2],
    ];
    frame.extend_from_slice(payload);
    let checksum = calculate_crc(&frame[1..], UNKNOWN_CRC_EXTRA);
    frame.extend_from_slice(&checksum.to_le_bytes());
    frame
}

fn known_v2_frame() -> Vec<u8> {
    let message = MavMessage::default_message_from_id(0).unwrap();
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(MavHeader::default(), &message);
    raw.raw_bytes().to_vec()
}

#[test]
fn low_level_reader_returns_unknown_frames_without_verification() {
    let v1 = unknown_v1_frame();
    let v2 = unknown_v2_frame();
    let bytes = [v1.as_slice(), v2.as_slice()].concat();
    let mut reader = MavlinkReader::new(bytes.as_slice());

    let first = reader.read_any_unverified_frame().unwrap();
    let second = reader.read_any_unverified_frame().unwrap();

    assert!(matches!(first, MAVLinkUnverifiedFrame::V1(_)));
    assert_eq!(first.raw_bytes(), v1);
    assert_eq!(first.message_id(), u32::from(UNKNOWN_V1_ID));
    assert!(matches!(second, MAVLinkUnverifiedFrame::V2(_)));
    assert_eq!(second.raw_bytes(), v2);
    assert_eq!(second.message_id(), UNKNOWN_V2_ID);
}

#[test]
fn validation_error_preserves_the_complete_unverified_frame() {
    let bytes = unknown_v2_frame();
    let mut reader = MavlinkReader::new(bytes.as_slice());
    let frame = reader.read_unverified_frame(MavlinkVersion::V2).unwrap();

    let error = match frame.validate::<MavMessage>() {
        Ok(_) => panic!("unknown frame unexpectedly validated"),
        Err(error) => error,
    };

    assert_eq!(error.reason, FrameValidationErrorKind::InvalidChecksum);
    assert_eq!(frame.raw_bytes(), bytes);
    assert_eq!(frame.message_id(), UNKNOWN_V2_ID);
}

#[test]
fn existing_validated_reader_keeps_discarding_invalid_candidates() {
    let unknown = unknown_v2_frame_with_payload(&[0x12, 0x34, 0x35, 0x36]);
    let known = known_v2_frame();
    let bytes = [
        [MAV_STX_V2, 255, 0x80].as_slice(),
        unknown.as_slice(),
        known.as_slice(),
    ]
    .concat();
    let mut reader = MavlinkReader::new(bytes.as_slice());

    let received = reader
        .read_raw_message::<MavMessage>(MavlinkVersion::V2)
        .unwrap();

    match received {
        mavlink::MAVLinkMessageRaw::V2(frame) => assert_eq!(frame.raw_bytes(), known),
        _ => panic!("expected MAVLink 2 frame"),
    }
}

#[test]
fn unverified_frame_can_be_validated_against_a_known_dialect() {
    let bytes = known_v2_frame();
    let mut reader = MavlinkReader::new(bytes.as_slice());
    let frame = reader.read_unverified_frame(MavlinkVersion::V2).unwrap();

    let validated = frame.validate::<MavMessage>().unwrap();

    assert_eq!(validated.message_id(), 0);
    assert!(matches!(validated, mavlink::MAVLinkMessageRaw::V2(_)));
}

#[test]
fn mixed_stream_parses_the_known_frame_and_not_the_unknown_frame() {
    let known = known_v2_frame();
    let unknown = unknown_v2_frame();
    let bytes = [known.as_slice(), unknown.as_slice()].concat();
    let mut reader = MavlinkReader::new(bytes.as_slice());
    let mut parsed_messages = Vec::new();
    let mut unknown_frames = Vec::new();

    for _i in 0..2 {
        let frame = reader.read_any_unverified_frame().unwrap();

        match frame.validate::<MavMessage>() {
            Ok(raw) => parsed_messages
                .push(MavMessage::parse(raw.version(), raw.message_id(), raw.payload()).unwrap()),
            Err(_) => unknown_frames.push(frame),
        }
    }

    assert!(matches!(&parsed_messages[0], MavMessage::HEARTBEAT(_)));
    assert_eq!(parsed_messages.len(), 1);
    assert_eq!(unknown_frames.len(), 1);
    assert_eq!(unknown_frames[0].message_id(), UNKNOWN_V2_ID);
    assert_eq!(unknown_frames[0].raw_bytes(), unknown);
}

#[test]
fn low_level_reader_makes_no_checksum_claim() {
    let mut bytes = unknown_v1_frame();
    *bytes.last_mut().unwrap() ^= 0xff;
    let mut reader = MavlinkReader::new(bytes.as_slice());

    let frame = reader.read_unverified_frame(MavlinkVersion::V1).unwrap();

    assert_eq!(frame.raw_bytes(), bytes);
}

#[test]
fn validated_and_unverified_reads_share_buffered_input() {
    let known = known_v2_frame();
    let unknown = unknown_v2_frame();
    let bytes = [known.as_slice(), unknown.as_slice(), known.as_slice()].concat();
    let mut reader = MavlinkReader::new(bytes.as_slice());

    assert_eq!(
        reader.read_any_unverified_frame().unwrap().raw_bytes(),
        known
    );
    assert_eq!(
        reader.read_any_unverified_frame().unwrap().raw_bytes(),
        unknown
    );
    let parsed = reader.read_any_raw_message::<MavMessage>().unwrap();
    assert_eq!(parsed.message_id(), 0);
}

#[test]
fn unverified_reader_skips_unsupported_flags_before_using_length() {
    let unsupported = [MAV_STX_V2, 0, 0x80, 0, 43, 9, 10, 0, 0, 0, 0, 0];
    let expected = unknown_v2_frame();
    let bytes = [
        [MAV_STX_V2, 255, 0x80].as_slice(),
        unsupported.as_slice(),
        expected.as_slice(),
    ]
    .concat();
    let mut reader = MavlinkReader::new(bytes.as_slice());

    let frame = reader.read_any_unverified_frame().unwrap();

    assert_eq!(frame.raw_bytes(), expected);
}

#[cfg(feature = "tokio")]
#[tokio::test]
async fn async_low_level_reader_returns_unknown_frame() {
    use mavlink::AsyncMavlinkReader;

    let expected = unknown_v2_frame();
    let bytes = [[MAV_STX_V2, 255, 0x80].as_slice(), expected.as_slice()].concat();
    let mut reader = AsyncMavlinkReader::new(bytes.as_slice());

    let frame = reader.read_any_unverified_frame().await.unwrap();

    assert_eq!(frame.raw_bytes(), expected);
    assert_eq!(frame.message_id(), UNKNOWN_V2_ID);
}
