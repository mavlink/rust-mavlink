pub mod test_shared;

#[cfg(feature = "common")]
mod test_v1_encode_decode {
    use crate::test_shared::HEARTBEAT_V1;
    use mavlink_core::peek_reader::PeekReader;

    #[test]
    pub fn test_read_heartbeat() {
        let mut r = PeekReader::new(HEARTBEAT_V1);
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
        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();
        mavlink::write_v1_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(heartbeat_msg),
        )
        .expect("Failed to write message");

        assert_eq!(&b[..HEARTBEAT_V1.len()], HEARTBEAT_V1);
    }

    #[test]
    #[cfg(not(feature = "emit-extensions"))]
    pub fn test_echo_servo_output_raw() {
        use mavlink::Message;

        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_servo_output_raw_v1();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::SERVO_OUTPUT_RAW(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
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

    #[test]
    #[cfg(feature = "std")]
    pub fn test_read_error() {
        use std::io::ErrorKind;

        use mavlink_core::error::MessageReadError;

        let mut reader = PeekReader::new(crate::test_shared::BlockyReader::new(HEARTBEAT_V1));

        loop {
            match mavlink::read_v1_msg::<mavlink::common::MavMessage, _>(&mut reader) {
                Ok((header, _)) => {
                    assert_eq!(header, crate::test_shared::COMMON_MSG_HEADER);
                    break;
                }
                Err(MessageReadError::Io(err)) if err.kind() == ErrorKind::WouldBlock => {}
                Err(err) => panic!("{err}"),
            }
        }
    }

    #[test]
    #[cfg(feature = "emit-extensions")]
    pub fn test_extensions_v1() {
        use mavlink::common::COMMAND_ACK_DATA;
        // test if "Extension fields are not sent when a message is encoded using the MAVLink 1 protocol" holds
        let ack_command = COMMAND_ACK_DATA {
            command: mavlink::common::MavCmd::MAV_CMD_NAV_WAYPOINT,
            result: mavlink::common::MavResult::MAV_RESULT_TEMPORARILY_REJECTED,
            progress: 2,
            result_param2: 3,
            target_system: 4,
            target_component: 5,
        };
        let ack_msg_data = mavlink::common::MavMessage::COMMAND_ACK(ack_command);
        let mut buf = vec![];
        mavlink::write_v1_msg(
            &mut buf,
            crate::test_shared::COMMON_MSG_HEADER,
            &ack_msg_data,
        )
        .unwrap();
        // check expected len of serialized buffer
        // expected is 1 byte STX, 5 byte header, 3 bytes for message content and 2 byte crc
        assert_eq!(buf.len(), 1 + 5 + 3 + 2);

        let mut reader = PeekReader::new(&*buf);
        let (_, read_msg) =
            mavlink::read_v1_msg::<mavlink::common::MavMessage, _>(&mut reader).unwrap();
        if let mavlink::common::MavMessage::COMMAND_ACK(read_ack_command) = read_msg {
            // chech if the deserialized message has extension fields set to 0
            assert_eq!(
                read_ack_command.command,
                mavlink::common::MavCmd::MAV_CMD_NAV_WAYPOINT
            );
            assert_eq!(
                read_ack_command.result,
                mavlink::common::MavResult::MAV_RESULT_TEMPORARILY_REJECTED
            );
            assert_eq!(read_ack_command.progress, 0);
            assert_eq!(read_ack_command.result_param2, 0);
            assert_eq!(read_ack_command.target_system, 0);
            assert_eq!(read_ack_command.target_component, 0);
        } else {
            panic!("Read invalid message")
        }
    }

    #[test]
    pub fn test_overflowing_msg_id() {
        // test behaivior for message ids that are not valid for MAVLink 1
        let msg_data = mavlink::common::MavMessage::SETUP_SIGNING(
            mavlink::common::SETUP_SIGNING_DATA::default(),
        );
        let mut buf = vec![];
        assert!(
            matches!(
                mavlink::write_v1_msg(&mut buf, crate::test_shared::COMMON_MSG_HEADER, &msg_data,),
                Err(mavlink::error::MessageWriteError::MAVLink2Only)
            ),
            "Writing a message with id 256 should return an error for MAVLink 1"
        );
        assert!(buf.is_empty(), "No bytes should be written");
    }
}
