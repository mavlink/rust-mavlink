#[macro_use]
mod test_shared;

#[cfg(feature = "common")]
mod test_encode_decode {
    use mavlink::{common, Message};
    use mavlink_core::peek_reader::PeekReader;

    #[test]
    pub fn test_echo_heartbeat() {
        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_heartbeat_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &common::MavMessage::HEARTBEAT(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
        let (_header, recv_msg): (mavlink::MavHeader, common::MavMessage) =
            mavlink::read_v2_msg(&mut c).expect("Failed to read");
        assert_eq!(recv_msg.message_id(), 0);
    }

    #[test]
    pub fn test_echo_command_int() {
        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_cmd_nav_takeoff_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::COMMAND_INT(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");

        if let common::MavMessage::COMMAND_INT(recv_msg) = recv_msg {
            assert_eq!(recv_msg.command, common::MavCmd::MAV_CMD_NAV_TAKEOFF);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    pub fn test_echo_hil_actuator_controls() {
        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_hil_actuator_controls_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HIL_ACTUATOR_CONTROLS(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");
        if let mavlink::common::MavMessage::HIL_ACTUATOR_CONTROLS(recv_msg) = recv_msg {
            assert_eq!(
                mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                recv_msg.mode & mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            );
        } else {
            panic!("Decoded wrong message type")
        }
    }

    /// This test makes sure that we can still receive messages in the common set
    /// properly when we're trying to decode APM messages.
    #[test]
    #[cfg(feature = "ardupilotmega")]
    pub fn test_echo_apm_heartbeat() {
        use mavlink::ardupilotmega;

        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_heartbeat_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");

        match &recv_msg {
            ardupilotmega::MavMessage::HEARTBEAT(_data) => {
                assert_eq!(recv_msg.message_id(), 0);
            }
            _ => panic!("Decoded wrong message type"),
        }
    }

    /// This test makes sure that messages that are not
    /// in the common set also get encoded and decoded
    /// properly.
    #[test]
    #[cfg(feature = "ardupilotmega")]
    pub fn test_echo_apm_mount_status() {
        use mavlink::ardupilotmega;

        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_apm_mount_status();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &ardupilotmega::MavMessage::MOUNT_STATUS(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");
        if let ardupilotmega::MavMessage::MOUNT_STATUS(recv_msg) = recv_msg {
            assert_eq!(4, recv_msg.pointing_b);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    #[cfg(feature = "ardupilotmega")]
    pub fn test_echo_apm_command_int() {
        use mavlink::ardupilotmega;

        let mut b = [0u8; 280];
        let mut v: &mut [u8] = &mut b;
        let send_msg = crate::test_shared::get_cmd_nav_takeoff_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &common::MavMessage::COMMAND_INT(send_msg),
        )
        .expect("Failed to write message");

        let mut c = PeekReader::new(b.as_slice());
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");

        match &recv_msg {
            ardupilotmega::MavMessage::COMMAND_INT(data) => {
                assert_eq!(data.command, ardupilotmega::MavCmd::MAV_CMD_NAV_TAKEOFF);
            }
            _ => panic!("Decoded wrong message type"),
        }
    }

    fn encode_decode_all_messages<M, F>(message_ids: &'static [u32], message_from_id: F)
    where
        M: mavlink::Message + std::fmt::Debug + std::cmp::PartialEq,
        F: Fn(u32) -> Option<M>,
    {
        let encoded_header = crate::test_shared::COMMON_MSG_HEADER;

        for id in message_ids {
            let encoded_message = message_from_id(*id).expect("Unknown ID");

            // dbg!(id, &encoded_message);

            let mut buffer = [0u8; 280];
            let mut v: &mut [u8] = &mut buffer;

            mavlink::write_v2_msg(&mut v, encoded_header, &encoded_message)
                .expect("Failed to write message");

            // dbg!(id, &v);

            let mut reader = PeekReader::new(buffer.as_slice());
            let (decoded_header, decoded_message) =
                mavlink::read_v2_msg::<M, _>(&mut reader).expect("Failed to read");

            // dbg!(id, &decoded_message);

            assert_eq!(encoded_header, decoded_header);
            assert_eq!(encoded_message, decoded_message);
        }
    }

    #[test]
    fn test_encode_decode_all_messages_from_default() {
        for_all_dialects!(
            encode_decode_all_messages,
            mavlink::Message::default_message_from_id,
        );
    }

    #[test]
    #[cfg(feature = "arbitrary")]
    fn test_encode_decode_all_messages_from_random() {
        trait RandomMessageExtension: mavlink::Message {
            fn custom_random_message_from_id(id: u32) -> Option<Self>;
        }

        impl<M: mavlink::Message> RandomMessageExtension for M {
            fn custom_random_message_from_id(id: u32) -> Option<Self> {
                use rand::{rngs::StdRng, SeedableRng};
                let mut rng = StdRng::seed_from_u64(42);

                M::random_message_from_id(id, &mut rng)
            }
        }

        for_all_dialects!(
            encode_decode_all_messages,
            RandomMessageExtension::custom_random_message_from_id
        );
    }
}
