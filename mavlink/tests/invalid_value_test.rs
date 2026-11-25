mod test_shared;

#[cfg(feature = "dialect-common")]
mod helper_tests {
    use crate::test_shared::{COMMON_MSG_HEADER, HEARTBEAT_V2};
    use mavlink::{
        MavlinkReader, MavlinkVersion, MessageData, calculate_crc,
        dialects::common::{HEARTBEAT_DATA, MavAutopilot, MavMessage},
    };

    fn read_v2(frame: &[u8]) -> MavMessage {
        let (header, msg) = MavlinkReader::new(frame)
            .read_message::<MavMessage>(MavlinkVersion::V2)
            .expect("failed to parse message");
        assert_eq!(header, COMMON_MSG_HEADER);
        msg
    }

    fn write_v2(msg: &MavMessage) -> Vec<u8> {
        let mut buffer = [0u8; 280];
        let len = mavlink::write_v2_msg(&mut buffer.as_mut_slice(), COMMON_MSG_HEADER, msg)
            .expect("failed to serialize message");
        buffer[..len].to_vec()
    }

    #[test]
    fn test_unknown_enum_value() {
        let mut frame = HEARTBEAT_V2.to_vec();
        // set autopilot to an invalid MavAutopilot value
        frame[1 + 9 + 5] = 255;
        // update crc
        let crc = calculate_crc(&frame[1..frame.len() - 2], HEARTBEAT_DATA::EXTRA_CRC);
        let crc_start = frame.len() - 2;
        frame[crc_start..].copy_from_slice(&crc.to_le_bytes());

        let msg = read_v2(&frame);
        let MavMessage::HEARTBEAT(heartbeat) = &msg else {
            panic!("decoded wrong message type");
        };
        assert!(
            matches!(heartbeat.autopilot, MavAutopilot::Unknown(value) if value.value() == 255)
        );
        // The message encodes back to the same bytes.
        assert_eq!(write_v2(&msg), frame);
    }

    #[test]
    fn test_from_raw() {
        assert_eq!(
            MavAutopilot::from_raw(3),
            MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA
        );
        // MavAutopilot defines 0..=20.
        for raw in 0..=255 {
            let autopilot = MavAutopilot::from_raw(raw);
            assert_eq!(autopilot.to_raw(), raw);
            assert_eq!(matches!(autopilot, MavAutopilot::Unknown(_)), raw > 20);
        }
    }

    #[cfg(feature = "mav2-message-extensions")]
    #[test]
    fn test_negative_unknown_enum_value() {
        use mavlink::dialects::common::{WIFI_CONFIG_AP_DATA, WifiConfigApResponse};

        // `response` is an int8_t field; -5 has no variant.
        let response = WifiConfigApResponse::from_raw(-5i8 as u32);
        assert!(matches!(response, WifiConfigApResponse::Unknown(_)));
        let msg = MavMessage::WIFI_CONFIG_AP(WIFI_CONFIG_AP_DATA {
            response,
            ..WIFI_CONFIG_AP_DATA::DEFAULT
        });

        let frame = write_v2(&msg);
        assert_eq!(read_v2(&frame), msg);
        assert_eq!(response.to_raw() as i8, -5);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn test_unknown_enum_value_serde() {
        let unknown = MavAutopilot::from_raw(255);
        let json = serde_json::to_value(unknown).unwrap();
        assert_eq!(json, serde_json::json!({ "type": "Unknown", "value": 255 }));
        assert_eq!(
            serde_json::from_value::<MavAutopilot>(json).unwrap(),
            unknown
        );

        // A value that has a variant must use the variant's name.
        let known = serde_json::json!({ "type": "Unknown", "value": 3 });
        assert!(serde_json::from_value::<MavAutopilot>(known).is_err());
    }

    #[test]
    fn test_unknown_bitflag_bits_are_preserved() {
        use mavlink::dialects::common::{
            MavFrame, PositionTargetTypemask, SET_POSITION_TARGET_GLOBAL_INT_DATA,
        };

        let send_msg = SET_POSITION_TARGET_GLOBAL_INT_DATA {
            coordinate_frame: MavFrame::MAV_FRAME_GLOBAL,
            // Regression test for https://github.com/mavlink/rust-mavlink/issues/484
            type_mask: PositionTargetTypemask::from_bits_retain(65016),
            ..SET_POSITION_TARGET_GLOBAL_INT_DATA::DEFAULT
        };

        let mut buffer = [0u8; 280];
        let mut writer: &mut [u8] = &mut buffer;

        mavlink::write_v2_msg(
            &mut writer,
            crate::test_shared::COMMON_MSG_HEADER,
            &MavMessage::SET_POSITION_TARGET_GLOBAL_INT(send_msg),
        )
        .expect("failed to serialize SET_POSITION_TARGET_GLOBAL_INT");

        let mut reader = MavlinkReader::new(buffer.as_slice());
        let (_header, recv_msg) = reader
            .read_message::<MavMessage>(mavlink::MavlinkVersion::V2)
            .expect("failed to parse SET_POSITION_TARGET_GLOBAL_INT with unknown bitmask bits");

        let MavMessage::SET_POSITION_TARGET_GLOBAL_INT(recv_msg) = recv_msg else {
            panic!("decoded wrong message type");
        };

        assert_eq!(recv_msg.type_mask.bits(), 65016);
    }
}
