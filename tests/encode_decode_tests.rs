extern crate mavlink;

mod test_shared;

#[cfg(test)]
mod test_encode_decode {

    #[test]
    pub fn test_echo_heartbeat() {
        let mut v = vec![];
        let send_msg = crate::test_shared::get_heartbeat_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(send_msg.clone()),
        )
        .expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");
        assert_eq!(recv_msg.message_id(), 0);
    }

    #[test]
    pub fn test_echo_command_int() {
        let mut v = vec![];
        let send_msg = crate::test_shared::get_cmd_nav_takeoff_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::COMMAND_INT(send_msg.clone()),
        )
        .expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = mavlink::read_v2_msg(&mut c).expect("Failed to read");

        if let mavlink::common::MavMessage::COMMAND_INT(recv_msg) = recv_msg {
            assert_eq!(
                recv_msg.command,
                mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF
            );
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    pub fn test_echo_hil_actuator_controls() {
        let mut v = vec![];
        let send_msg = crate::test_shared::get_hil_actuator_controls_msg();

        mavlink::write_v2_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HIL_ACTUATOR_CONTROLS(send_msg.clone()),
        )
        .expect("Failed to write message");

        let mut c = v.as_slice();
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
}
