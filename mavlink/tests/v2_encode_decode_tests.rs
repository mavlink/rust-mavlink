mod test_shared;

#[cfg(all(feature = "std", feature = "common"))]
mod test_v2_encode_decode {
    use crate::test_shared::HEARTBEAT_V2;
    use mavlink_core::peek_reader::PeekReader;
    use mavlink_core::Message;

    #[test]
    pub fn test_read_v2_heartbeat() {
        let mut r = PeekReader::new(HEARTBEAT_V2);
        let (header, msg) = mavlink::read_v2_msg(&mut r).expect("Failed to parse message");

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
    pub fn test_write_v2_heartbeat() {
        let mut v = vec![];
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();
        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(heartbeat_msg),
        )
        .expect("Failed to write message");

        assert_eq!(&v[..], HEARTBEAT_V2);
    }

    /// A COMMAND_LONG message with a truncated payload (allowed for empty fields)
    pub const COMMAND_LONG_TRUNCATED_V2: &[u8] = &[
        mavlink::MAV_STX_V2,
        30,
        0,
        0,
        0,
        0,
        50, //header
        76,
        0,
        0, //msg ID
        //truncated payload:
        0,
        0,
        230,
        66,
        0,
        64,
        156,
        69,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        255,
        1,
        // crc:
        188,
        195,
    ];

    #[test]
    pub fn test_read_truncated_command_long() {
        let mut r = PeekReader::new(COMMAND_LONG_TRUNCATED_V2);
        let (_header, recv_msg) =
            mavlink::read_v2_msg(&mut r).expect("Failed to parse COMMAND_LONG_TRUNCATED_V2");

        if let mavlink::common::MavMessage::COMMAND_LONG(recv_msg) = recv_msg {
            assert_eq!(
                recv_msg.command,
                mavlink::common::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL
            );
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    #[cfg(feature = "emit-extensions")]
    pub fn test_echo_servo_output_raw() {
        use mavlink::{common, Message};

        let mut v = vec![];
        let send_msg = crate::test_shared::get_servo_output_raw_v2();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::SERVO_OUTPUT_RAW(send_msg.clone()),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(v.as_slice());
        let (_header, recv_msg): (mavlink::MavHeader, mavlink::common::MavMessage) =
            mavlink::read_v2_msg(&mut c).expect("Failed to read");

        assert_eq!(
            mavlink::common::MavMessage::extra_crc(recv_msg.message_id()),
            222 as u8
        );

        if let mavlink::common::MavMessage::SERVO_OUTPUT_RAW(recv_msg) = recv_msg {
            assert_eq!(recv_msg.port, 123 as u8);
            assert_eq!(recv_msg.servo4_raw, 1400 as u16);
            assert_eq!(recv_msg.servo14_raw, 1660 as u16);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    pub fn test_serialize_to_raw() {
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();
        let mut raw_msg = mavlink::MAVLinkV2MessageRaw::new();

        raw_msg.serialize_message_data(crate::test_shared::COMMON_MSG_HEADER, &heartbeat_msg);

        assert_eq!(raw_msg.raw_bytes(), HEARTBEAT_V2);
        assert!(raw_msg.has_valid_crc::<mavlink::common::MavMessage>());
    }

    #[test]
    pub fn test_read_error() {
        use std::io::ErrorKind;

        use mavlink_core::error::MessageReadError;

        let mut reader = PeekReader::new(crate::test_shared::BlockyReader::new(HEARTBEAT_V2));

        loop {
            match mavlink::read_v2_msg::<mavlink::common::MavMessage, _>(&mut reader) {
                Ok((header, _)) => {
                    assert_eq!(header, crate::test_shared::COMMON_MSG_HEADER);
                    break;
                }
                Err(MessageReadError::Io(err)) if err.kind() == ErrorKind::WouldBlock => {}
                Err(err) => panic!("{err}"),
            }
        }
    }

    const PARAMETER_VALUE_BAT1_R_INTERNAL: &[u8] = &[
        0xfd, 0x19, 0x00, 0x00, 0x5a, 0x01, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x80, 0xbf, 0xf5,
        0x03, 0x04, 0x00, 0x42, 0x41, 0x54, 0x31, 0x5f, 0x52, 0x5f, 0x49, 0x4e, 0x54, 0x45, 0x52,
        0x4e, 0x41, 0x4c, 0x00, 0x09, 0xd4, 0x14,
    ];

    const PARAMETER_VALUE_HASH_CHECK: &[u8] = &[
        0xfd, 0x19, 0x00, 0x00, 0xed, 0x01, 0x01, 0x16, 0x00, 0x00, 0x52, 0x53, 0x89, 0x84, 0xf5,
        0x03, 0xff, 0xff, 0x5f, 0x48, 0x41, 0x53, 0x48, 0x5f, 0x43, 0x48, 0x45, 0x43, 0x4b, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x05, 0x87, 0x87,
    ];

    #[test]
    pub fn test_decode_encode_v2_frame_parameter_value_bat1_r_internal() {
        let mut r = PeekReader::new(PARAMETER_VALUE_BAT1_R_INTERNAL);
        let (header, msg) =
            mavlink::read_v2_msg::<mavlink::common::MavMessage, _>(&mut r).expect("decode");

        let mut buffer = [0; 512];
        let mut out: &mut [u8] = &mut buffer[..];
        let len = mavlink::write_v2_msg(&mut out, header, &msg).expect("encode");
        assert_eq!(&buffer[..len], PARAMETER_VALUE_BAT1_R_INTERNAL);
    }

    #[test]
    pub fn test_decode_encode_v2_frame_parameter_value_hash_check() {
        let mut r = PeekReader::new(PARAMETER_VALUE_HASH_CHECK);
        let (header, msg) =
            mavlink::read_v2_msg::<mavlink::common::MavMessage, _>(&mut r).expect("decode");

        let param_value = match msg.clone() {
            mavlink::common::MavMessage::PARAM_VALUE(param_value) => param_value,
            _ => panic!(
                "Expected PARAMETER_VALUE message, got {:?}",
                msg.message_id()
            ),
        };

        let param_id = String::from_utf8(param_value.param_id[..11].to_vec()).unwrap();
        assert_eq!(param_id, "_HASH_CHECK");
        assert_eq!(
            param_value.param_type,
            mavlink::common::MavParamType::MAV_PARAM_TYPE_UINT32
        );

        let mut buffer = [0; 512];
        let mut out: &mut [u8] = &mut buffer[..];
        let len = mavlink::write_v2_msg(&mut out, header, &msg).expect("encode");
        assert_eq!(&buffer[..len], PARAMETER_VALUE_HASH_CHECK);
    }
}
