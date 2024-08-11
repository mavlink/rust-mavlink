pub mod test_shared;

mod mav_frame_tests {
    use mavlink::ardupilotmega::MavMessage;
    use mavlink::MavFrame;
    use mavlink::MavHeader;

    // NOTE: No STX, length, or flag fields in the header
    pub const HEARTBEAT_V2: &[u8] = &[
        // Currently [`MavFrame::deser`] and [`MavFrame::ser`] does not account for the first four fields.
        // 0xfd, // STX V2
        // 0x09, // len
        // 0x00, // incompat_flags
        // 0x00, // compat_flags
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

    #[test]
    pub fn test_deser_ser_message() {
        let buf: &mut [u8; 255] = &mut [0; 255];

        let mavlink_message = mavlink_message();
        let mavlink_frame = new(mavlink_message);

        let _len = mavlink_frame.ser(buf);

        let parsed_mavlink_frame =
            MavFrame::<mavlink::ardupilotmega::MavMessage>::deser(mavlink::MavlinkVersion::V2, buf)
                .unwrap();

        assert_eq!(
            format!("{mavlink_frame:?}"),
            format!("{parsed_mavlink_frame:?}")
        );
    }

    fn mavlink_message() -> mavlink::ardupilotmega::MavMessage {
        mavlink::ardupilotmega::MavMessage::LINK_NODE_STATUS(
            mavlink::ardupilotmega::LINK_NODE_STATUS_DATA {
                timestamp: 92197916,
                tx_rate: 0x11223344,
                rx_rate: 0x55667788,
                messages_sent: 0x99001122,
                messages_received: 0x33445566,
                messages_lost: 0x77889900,
                rx_parse_err: 0x1122,
                tx_overflows: 0x3355,
                rx_overflows: 0x5566,
                tx_buf: 0xff,
                rx_buf: 0x11,
            },
        )
    }

    fn new(msg: MavMessage) -> MavFrame<MavMessage> {
        MavFrame {
            header: MavHeader {
                system_id: 1,
                component_id: 2,
                sequence: 84,
            },
            msg,
            protocol_version: mavlink::MavlinkVersion::V2,
        }
    }
}
