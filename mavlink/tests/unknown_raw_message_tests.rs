#![cfg(feature = "dialect-common")]

use mavlink::{
    MAV_STX, MAV_STX_V2, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavConnection, MavHeader, Message,
    calculate_crc, dialects::common::MavMessage, error::ParserError, peek_reader::PeekReader,
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
    assert!(MavMessage::default_message_from_id(UNKNOWN_V2_ID).is_none());

    let id = UNKNOWN_V2_ID.to_le_bytes();
    let mut frame = vec![
        MAV_STX_V2,
        PAYLOAD.len() as u8,
        0,
        0,
        43,
        9,
        10,
        id[0],
        id[1],
        id[2],
    ];
    frame.extend_from_slice(PAYLOAD);
    let checksum = calculate_crc(&frame[1..], UNKNOWN_CRC_EXTRA);
    frame.extend_from_slice(&checksum.to_le_bytes());
    frame
}

fn known_v2_frame() -> Vec<u8> {
    let known = MavMessage::default_message_from_id(0).unwrap();
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(MavHeader::default(), &known);
    raw.raw_bytes().to_vec()
}

#[test]
fn raw_reader_preserves_unknown_v1_frame() {
    let bytes = unknown_v1_frame();
    let mut reader = PeekReader::new(bytes.as_slice());

    let raw = mavlink::read_v1_raw_message_including_unknown::<MavMessage, _>(&mut reader).unwrap();

    assert_eq!(raw.message_id(), UNKNOWN_V1_ID);
    assert_eq!(raw.payload(), PAYLOAD);
    assert_eq!(raw.raw_bytes(), bytes);
    assert!(!raw.has_valid_crc::<MavMessage>());
}

#[test]
fn raw_reader_preserves_unknown_v2_frame() {
    let bytes = unknown_v2_frame();
    let mut reader = PeekReader::new(bytes.as_slice());

    let raw = mavlink::read_v2_raw_message_including_unknown::<MavMessage, _>(&mut reader).unwrap();

    assert_eq!(raw.message_id(), UNKNOWN_V2_ID);
    assert_eq!(raw.payload(), PAYLOAD);
    assert_eq!(raw.raw_bytes(), bytes);
    assert!(!raw.has_valid_crc::<MavMessage>());
}

#[test]
fn existing_raw_reader_does_not_return_unknown_messages() {
    let unknown = unknown_v2_frame();
    let known = known_v2_frame();
    let bytes = [unknown.as_slice(), known.as_slice()].concat();
    let mut reader = PeekReader::new(bytes.as_slice());

    let received = mavlink::read_v2_raw_message::<MavMessage, _>(&mut reader).unwrap();

    assert_eq!(received.raw_bytes(), known);
}

#[test]
fn version_agnostic_raw_reader_preserves_unknown_frames() {
    let v1 = unknown_v1_frame();
    let v2 = unknown_v2_frame();
    let bytes = [v1.as_slice(), v2.as_slice()].concat();
    let mut reader = PeekReader::new(bytes.as_slice());

    let first =
        mavlink::read_any_raw_message_including_unknown::<MavMessage, _>(&mut reader).unwrap();
    let second =
        mavlink::read_any_raw_message_including_unknown::<MavMessage, _>(&mut reader).unwrap();

    assert!(matches!(first, MAVLinkMessageRaw::V1(ref raw) if raw.raw_bytes() == v1));
    assert!(matches!(second, MAVLinkMessageRaw::V2(ref raw) if raw.raw_bytes() == v2));
}

#[test]
fn preserved_unknown_frame_cannot_be_parsed_by_the_dialect() {
    let bytes = unknown_v2_frame();
    let mut reader = PeekReader::new(bytes.as_slice());
    let raw = mavlink::read_v2_raw_message_including_unknown::<MavMessage, _>(&mut reader).unwrap();

    let error = MavMessage::parse(mavlink::MavlinkVersion::V2, raw.message_id(), raw.payload())
        .unwrap_err();

    assert!(matches!(
        error,
        ParserError::UnknownMessage { id: UNKNOWN_V2_ID }
    ));
}

#[test]
fn raw_reader_still_rejects_bad_crc_for_known_messages() {
    let known = MavMessage::default_message_from_id(0).unwrap();
    let mut raw = MAVLinkV2MessageRaw::new();
    raw.serialize_message(MavHeader::default(), &known);
    let mut corrupt_known = raw.raw_bytes().to_vec();
    *corrupt_known.last_mut().unwrap() ^= 0xff;

    let unknown = unknown_v2_frame();
    let bytes = [corrupt_known.as_slice(), unknown.as_slice()].concat();
    let mut reader = PeekReader::new(bytes.as_slice());

    let received =
        mavlink::read_v2_raw_message_including_unknown::<MavMessage, _>(&mut reader).unwrap();

    assert_eq!(received.raw_bytes(), unknown);
}

#[test]
fn connection_requires_explicit_opt_in_for_unknown_messages() {
    let path = std::env::temp_dir().join(format!(
        "rust-mavlink-unknown-message-{}.tlog",
        std::process::id()
    ));
    let unknown = unknown_v2_frame();
    std::fs::write(&path, &unknown).unwrap();

    let connection = mavlink::connect::<MavMessage>(&format!("file:{}", path.display())).unwrap();
    let received = connection.recv_raw_including_unknown().unwrap();

    assert!(matches!(received, MAVLinkMessageRaw::V2(ref raw) if raw.raw_bytes() == unknown));
    std::fs::remove_file(path).unwrap();
}

#[cfg(feature = "tokio")]
#[tokio::test]
async fn async_raw_reader_preserves_unknown_frames() {
    use mavlink::async_peek_reader::AsyncPeekReader;

    let v1 = unknown_v1_frame();
    let v2 = unknown_v2_frame();
    let bytes = [v1.as_slice(), v2.as_slice()].concat();
    let mut reader = AsyncPeekReader::new(bytes.as_slice());

    let first = mavlink::read_any_raw_message_async_including_unknown::<MavMessage, _>(&mut reader)
        .await
        .unwrap();
    let second =
        mavlink::read_any_raw_message_async_including_unknown::<MavMessage, _>(&mut reader)
            .await
            .unwrap();

    assert!(matches!(first, MAVLinkMessageRaw::V1(ref raw) if raw.raw_bytes() == v1));
    assert!(matches!(second, MAVLinkMessageRaw::V2(ref raw) if raw.raw_bytes() == v2));
}
