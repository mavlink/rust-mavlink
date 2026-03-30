mod test_shared;

#[cfg(feature = "dialect-common")]
mod helper_tests {
    use mavlink::{
        MessageData, calculate_crc,
        dialects::common::MavMessage,
        error::{MessageReadError, ParserError},
        peek_reader::PeekReader,
    };

    #[test]
    fn test_invalid_enum() {
        use crate::test_shared::HEARTBEAT_V2;

        let mut invalid_enum_buf = [0; HEARTBEAT_V2.len()];
        invalid_enum_buf.copy_from_slice(HEARTBEAT_V2);
        // set autopilot to an invalid MavAutopilot value
        invalid_enum_buf[1 + 9 + 5] = 255;
        // update crc
        let crc = calculate_crc(
            &invalid_enum_buf[1..HEARTBEAT_V2.len() - 2],
            mavlink::dialects::common::HEARTBEAT_DATA::EXTRA_CRC,
        );
        invalid_enum_buf[HEARTBEAT_V2.len() - 2..HEARTBEAT_V2.len()]
            .copy_from_slice(&crc.to_le_bytes());

        let result = mavlink::read_v2_msg::<MavMessage, _>(&mut PeekReader::new(
            invalid_enum_buf.as_slice(),
        ));
        assert!(matches!(
            result,
            Err(MessageReadError::Parse(ParserError::InvalidEnum {
                enum_type: "MavAutopilot",
                value: 255
            }))
        ));
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

        let mut reader = PeekReader::new(buffer.as_slice());
        let (_header, recv_msg) = mavlink::read_v2_msg::<MavMessage, _>(&mut reader)
            .expect("failed to parse SET_POSITION_TARGET_GLOBAL_INT with unknown bitmask bits");

        let MavMessage::SET_POSITION_TARGET_GLOBAL_INT(recv_msg) = recv_msg else {
            panic!("decoded wrong message type");
        };

        assert_eq!(recv_msg.type_mask.bits(), 65016);
    }
}
