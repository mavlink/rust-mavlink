pub mod test_shared;

use mavlink::MAV_STX;
use mavlink::MAV_STX_V2;

// 100 randomly generted bytes with 10 extra MAV_STX/MAV_STX_V2 each inserted
const GARBAGE: [u8; 120] = [
    0xfe, 0x43, 0x2d, MAV_STX, MAV_STX, 0x26, 0x1e, 0x33, 0x85, 0x38, 0x1d, 0x20, 0x20, 0x90, 0xd9,
    0x24, 0xb6, 0xd7, 0xb1, 0x22, 0x3b, 0xaf, 0x7c, 0x2f, MAV_STX, 0x9d, 0x1a, 0x13, 0x16, 0x2b,
    0xf8, 0x6f, 0xf4, 0xdc, 0x66, 0xff, 0x2d, MAV_STX_V2, 0xe2, 0x2c, 0xb1, MAV_STX_V2, 0x4e, 0xc9,
    0xc6, 0xcb, 0x3e, 0x3e, 0xf4, MAV_STX_V2, MAV_STX_V2, 0x49, 0xbc, 0x11, 0xb7, 0xd4, 0x5e,
    MAV_STX, 0x46, 0x6a, 0xd3, 0xb9, MAV_STX, 0xe3, 0x81, 0x1d, MAV_STX_V2, 0x80, 0x47, 0xfc, 0xff,
    0x0c, 0xaa, 0xf3, MAV_STX, MAV_STX_V2, 0x87, 0x2f, 0x9a, 0x15, MAV_STX_V2, MAV_STX, 0x06, 0xc9,
    0xe1, 0xc0, 0x98, 0xf5, 0x71, 0x78, 0x1c, 0x4a, 0xe3, 0xf1, MAV_STX_V2, 0x5f, 0xdb, 0x0e, 0x3f,
    MAV_STX, 0x2e, MAV_STX_V2, 0x08, 0x39, 0x6e, 0x15, 0x3c, 0x55, 0xcb, 0x78, 0xe0, MAV_STX, 0x5a,
    0xb3, 0x1b, 0xf9, MAV_STX, 0xe0, 0xa0, MAV_STX_V2,
];

#[cfg(all(feature = "std", feature = "common"))]
mod test_agnostic_encode_decode {
    use crate::GARBAGE;
    use mavlink_core::peek_reader::PeekReader;
    use std::io::Write;

    #[test]
    pub fn test_read_heartbeats() {
        let mut buf = vec![];
        _ = buf.write(crate::test_shared::HEARTBEAT_V1);
        _ = buf.write(crate::test_shared::HEARTBEAT_V2);
        let mut r = PeekReader::new(buf.as_slice());
        // read 2 messages
        for _ in 0..2 {
            let (header, msg) = mavlink::read_any_msg(&mut r).expect("Failed to parse message");

            assert_eq!(header, crate::test_shared::COMMON_MSG_HEADER);
            let heartbeat_msg = crate::test_shared::get_heartbeat_msg();

            if let mavlink::common::MavMessage::HEARTBEAT(msg) = msg {
                assert_eq!(msg.custom_mode, heartbeat_msg.custom_mode);
                assert_eq!(msg.mavtype, heartbeat_msg.mavtype);
                assert_eq!(msg.autopilot, heartbeat_msg.autopilot);
                assert_eq!(msg.base_mode, heartbeat_msg.base_mode);
                assert_eq!(msg.system_status, heartbeat_msg.system_status);
                assert_eq!(msg.mavlink_version, heartbeat_msg.mavlink_version);
            } else {
                panic!("Decoded wrong message type")
            }
        }
    }

    #[test]
    pub fn test_read_inbetween_garbage() {
        // write some garbage bytes as well as 2 heartbeats and attempt to read them

        let mut buf = vec![];
        _ = buf.write(&GARBAGE);
        _ = buf.write(crate::test_shared::HEARTBEAT_V1);
        _ = buf.write(&GARBAGE);
        // only part of message
        _ = buf.write(&crate::test_shared::HEARTBEAT_V1[..5]);
        _ = buf.write(crate::test_shared::HEARTBEAT_V2);
        _ = buf.write(&GARBAGE);
        // only part of message
        _ = buf.write(&crate::test_shared::HEARTBEAT_V1[5..]);
        // add some zeros to prevent invalid package sizes from causing a read error
        _ = buf.write(&[0; 100]);

        let mut r = PeekReader::new(buf.as_slice());
        _ = mavlink::read_any_msg::<mavlink::common::MavMessage, _>(&mut r).unwrap();
        _ = mavlink::read_any_msg::<mavlink::common::MavMessage, _>(&mut r).unwrap();
        assert!(
            mavlink::read_any_msg::<mavlink::common::MavMessage, _>(&mut r).is_err(),
            "Parsed message from garbage data"
        )
    }
}

#[cfg(all(feature = "std", feature = "tokio-1", feature = "common"))]
mod test_agnostic_encode_decode_async {
    use crate::GARBAGE;
    use mavlink_core::async_peek_reader::AsyncPeekReader;
    use std::io::Write;

    #[tokio::test]
    pub async fn test_read_heartbeats() {
        let mut buf = vec![];
        _ = buf.write(crate::test_shared::HEARTBEAT_V1);
        _ = buf.write(crate::test_shared::HEARTBEAT_V2);
        let mut r = AsyncPeekReader::new(buf.as_slice());
        // read 2 messages
        for _ in 0..2 {
            let (header, msg) = mavlink::read_any_msg_async(&mut r)
                .await
                .expect("Failed to parse message");

            assert_eq!(header, crate::test_shared::COMMON_MSG_HEADER);
            let heartbeat_msg = crate::test_shared::get_heartbeat_msg();

            if let mavlink::common::MavMessage::HEARTBEAT(msg) = msg {
                assert_eq!(msg.custom_mode, heartbeat_msg.custom_mode);
                assert_eq!(msg.mavtype, heartbeat_msg.mavtype);
                assert_eq!(msg.autopilot, heartbeat_msg.autopilot);
                assert_eq!(msg.base_mode, heartbeat_msg.base_mode);
                assert_eq!(msg.system_status, heartbeat_msg.system_status);
                assert_eq!(msg.mavlink_version, heartbeat_msg.mavlink_version);
            } else {
                panic!("Decoded wrong message type")
            }
        }
    }

    #[tokio::test]
    pub async fn test_read_inbetween_garbage() {
        // write some garbage bytes as well as 2 heartbeats and attempt to read them

        let mut buf = vec![];
        _ = buf.write(&GARBAGE);
        _ = buf.write(crate::test_shared::HEARTBEAT_V1);
        _ = buf.write(&GARBAGE);
        // only part of message
        _ = buf.write(&crate::test_shared::HEARTBEAT_V1[..5]);
        _ = buf.write(crate::test_shared::HEARTBEAT_V2);
        _ = buf.write(&GARBAGE);
        // only part of message
        _ = buf.write(&crate::test_shared::HEARTBEAT_V1[5..]);
        // add some zeros to prevent invalid package sizes from causing a read error
        _ = buf.write(&[0; 100]);

        let mut r = AsyncPeekReader::new(buf.as_slice());
        _ = mavlink::read_any_msg_async::<mavlink::common::MavMessage, _>(&mut r)
            .await
            .unwrap();
        _ = mavlink::read_any_msg_async::<mavlink::common::MavMessage, _>(&mut r)
            .await
            .unwrap();
        assert!(
            mavlink::read_any_msg_async::<mavlink::common::MavMessage, _>(&mut r)
                .await
                .is_err(),
            "Parsed message from garbage data"
        )
    }
}
