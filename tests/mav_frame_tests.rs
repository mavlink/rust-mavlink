pub mod test_shared;

mod mav_frame_tests {
    // NOTE: No header
    pub const HEARTBEAT_V2: &[u8] = &[
        crate::test_shared::COMMON_MSG_HEADER.sequence,
        crate::test_shared::COMMON_MSG_HEADER.system_id,
        crate::test_shared::COMMON_MSG_HEADER.component_id,
        0x00, // msg ID
        0x00,
        0x00,
        0x05, // payload
        0x00,
        0x00,
        0x00,
        0x02,
        0x03,
        0x59,
        0x03,
        0x03,
        0x10, // checksum
        0xf0,
    ];

    #[test]
    pub fn test_deser_ser() {
        use mavlink::{common::MavMessage, MavFrame, MavlinkVersion};
        let frame = MavFrame::<MavMessage>::deser(MavlinkVersion::V2, HEARTBEAT_V2)
            .expect("failed to parse message");

        assert_eq!(frame.header, crate::test_shared::COMMON_MSG_HEADER);
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();

        let mut buffer = [0u8; HEARTBEAT_V2.len()];
        frame.ser(&mut buffer);
        assert_eq!(buffer[..buffer.len() - 2], HEARTBEAT_V2[..buffer.len() - 2]);

        let msg = match frame.msg {
            MavMessage::HEARTBEAT(msg) => msg,
            _ => panic!("Decoded wrong message type"),
        };
        assert_eq!(msg.custom_mode, heartbeat_msg.custom_mode);
        assert_eq!(msg.mavtype, heartbeat_msg.mavtype);
        assert_eq!(msg.autopilot, heartbeat_msg.autopilot);
        assert_eq!(msg.base_mode, heartbeat_msg.base_mode);
        assert_eq!(msg.system_status, heartbeat_msg.system_status);
        assert_eq!(msg.mavlink_version, heartbeat_msg.mavlink_version);
    }
}
