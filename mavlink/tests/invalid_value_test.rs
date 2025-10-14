mod test_shared;

#[cfg(feature = "common")]
mod helper_tests {
    use mavlink::{
        calculate_crc,
        common::MavMessage,
        error::{MessageReadError, ParserError},
        peek_reader::PeekReader,
        MavlinkVersion, MessageData,
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
            mavlink::common::HEARTBEAT_DATA::EXTRA_CRC,
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
    fn test_invalid_bitflag() {
        use mavlink::common::HIL_ACTUATOR_CONTROLS_DATA;

        let msg = HIL_ACTUATOR_CONTROLS_DATA::DEFAULT;
        let mut invalid_flag_buf = [0; 1 + 9 + HIL_ACTUATOR_CONTROLS_DATA::ENCODED_LEN + 2];
        let len = msg.ser(
            MavlinkVersion::V2,
            &mut invalid_flag_buf[10..10 + HIL_ACTUATOR_CONTROLS_DATA::ENCODED_LEN],
        );
        invalid_flag_buf[0] = mavlink::MAV_STX_V2;
        invalid_flag_buf[1] = len as u8;
        invalid_flag_buf[7] = HIL_ACTUATOR_CONTROLS_DATA::ID as u8;
        // set flags to an invalid HilActuatorControlsFlags value
        invalid_flag_buf[1 + 9 + 8..1 + 9 + 16].copy_from_slice(&u64::MAX.to_le_bytes());
        // update crc
        let crc = calculate_crc(
            &invalid_flag_buf[1..1 + 9 + len],
            HIL_ACTUATOR_CONTROLS_DATA::EXTRA_CRC,
        );
        invalid_flag_buf[1 + 9 + len..1 + 9 + len + 2].copy_from_slice(&crc.to_le_bytes());

        let result = mavlink::read_v2_msg::<MavMessage, _>(&mut PeekReader::new(
            invalid_flag_buf.as_slice(),
        ));
        assert!(matches!(
            result,
            Err(MessageReadError::Parse(ParserError::InvalidFlag {
                flag_type: "HilActuatorControlsFlags",
                value: u64::MAX
            }))
        ));
    }
}
