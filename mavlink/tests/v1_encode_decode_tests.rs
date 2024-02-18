pub mod test_shared;

#[cfg(all(feature = "std", feature = "common"))]
mod test_v1_encode_decode {

    pub const HEARTBEAT_V1: &[u8] = &[
        mavlink::MAV_STX,
        0x09,
        crate::test_shared::COMMON_MSG_HEADER.sequence,
        crate::test_shared::COMMON_MSG_HEADER.system_id,
        crate::test_shared::COMMON_MSG_HEADER.component_id,
        0x00,
        0x05,
        0x00,
        0x00,
        0x00,
        0x02,
        0x03,
        0x59,
        0x03,
        0x03,
        0x1f,
        0x50,
    ];

    #[test]
    pub fn test_read_heartbeat() {
        let mut r = HEARTBEAT_V1;
        let (header, msg) = mavlink::read_v1_msg(&mut r).expect("Failed to parse message");
        //println!("{:?}, {:?}", header, msg);

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

    #[test]
    pub fn test_write_heartbeat() {
        let mut v = vec![];
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();
        mavlink::write_v1_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(heartbeat_msg),
        )
        .expect("Failed to write message");

        assert_eq!(&v[..], HEARTBEAT_V1);
    }

    #[test]
    #[cfg(not(feature = "emit-extensions"))]
    pub fn test_echo_servo_output_raw() {
        use mavlink::Message;

        let mut v = vec![];
        let send_msg = crate::test_shared::get_servo_output_raw_v1();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::SERVO_OUTPUT_RAW(send_msg),
        )
        .expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg): (mavlink::MavHeader, mavlink::common::MavMessage) =
            mavlink::read_v2_msg(&mut c).expect("Failed to read");

        assert_eq!(
            mavlink::common::MavMessage::extra_crc(recv_msg.message_id()),
            222_u8
        );

        if let mavlink::common::MavMessage::SERVO_OUTPUT_RAW(recv_msg) = recv_msg {
            assert_eq!(recv_msg.port, 123_u8);
            assert_eq!(recv_msg.servo4_raw, 1400_u16);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    pub fn test_serialize_to_raw() {
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();
        let mut raw_msg = mavlink::MAVLinkV1MessageRaw::new();

        raw_msg.serialize_message_data(crate::test_shared::COMMON_MSG_HEADER, &heartbeat_msg);

        assert_eq!(raw_msg.raw_bytes(), HEARTBEAT_V1);
        assert!(raw_msg.has_valid_crc::<mavlink::common::MavMessage>());
    }
}
