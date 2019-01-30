
extern crate mavlink;

#[cfg(test)]
mod test_encode_decode {

    #[cfg(all(feature = "std", not(feature = "mavlink2")))]
    pub const HEARTBEAT: &'static [u8] = &[
        mavlink::MAV_STX, 0x09, 0xef, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03,
        0xf1, 0xd7,
    ];

    #[cfg(all(feature = "std", feature = "mavlink2"))]
    pub const HEARTBEAT_V2: &'static [u8] = &[
        mavlink::MAV_STX_V2, //magic
        0x09, //payload len
        0, //incompat flags
        0, //compat flags
        0xef, //seq 239
        0x01, //sys ID
        0x01, //comp ID
        0x00, 0x00, 0x00, //msg ID
        0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03, //payload
        16, 240, //checksum
    ];

    pub const COMMON_MSG_HEADER: mavlink::MavHeader = mavlink::MavHeader {
        sequence: 239,
        system_id: 1,
        component_id: 1,
    };

    fn get_heartbeat_msg() -> mavlink::common::HEARTBEAT_DATA {
        mavlink::common::HEARTBEAT_DATA {
            custom_mode: 5,
            mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: mavlink::common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | mavlink::common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | mavlink::common::MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
                | mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        }
    }


    fn get_cmd_nav_takeoff_msg() -> mavlink::common::COMMAND_INT_DATA {
        mavlink::common::COMMAND_INT_DATA {
            param1: 1.0,
            param2: 2.0,
            param3: 3.0,
            param4: 4.0,
            x: 555,
            y: 666,
            z: 777.0,
            command: mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF,
            target_system: 42,
            target_component: 84,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            current: 73,
            autocontinue: 17
        }
    }

    fn get_hil_actuator_controls_msg() -> mavlink::common::HIL_ACTUATOR_CONTROLS_DATA {
        mavlink::common::HIL_ACTUATOR_CONTROLS_DATA {
            time_usec: 1234567 as u64,
            flags: 0 as u64,
            controls: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
                10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
            mode: mavlink::common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | mavlink::common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        }
    }


    #[test]
    pub fn test_read_heartbeat() {
        #[cfg(all(feature = "std", not(feature = "mavlink2")))]
        let mut r = HEARTBEAT;
        #[cfg(all(feature = "std", feature = "mavlink2"))]
        let mut r = HEARTBEAT_V2;

        let (header, msg) = mavlink::read_msg(&mut r).expect("Failed to parse message");

        println!("{:?}, {:?}", header, msg);

        assert_eq!(header, COMMON_MSG_HEADER);
        let heartbeat_msg = get_heartbeat_msg();

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
        let heartbeat_msg = get_heartbeat_msg();
        mavlink::write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(heartbeat_msg.clone()),
        )
            .expect("Failed to write message");

        #[cfg(all(feature = "std", not(feature = "mavlink2")))] {
            assert_eq!(&v[..], HEARTBEAT);
        }

        #[cfg(all(feature = "std", feature = "mavlink2"))] {
            assert_eq!(&v[..], HEARTBEAT_V2);
        }
    }

    #[test]
    pub fn test_echo_heartbeat() {
        let mut v = vec![];
        let send_msg = get_heartbeat_msg();

        mavlink::write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = mavlink::read_msg(&mut c).expect("Failed to read");
        assert_eq!(recv_msg.message_id(), 0);
    }

    #[test]
    pub fn test_echo_command_int() {
        let mut v = vec![];
        let send_msg = get_cmd_nav_takeoff_msg();

        mavlink::write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::COMMAND_INT(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = mavlink::read_msg(&mut c).expect("Failed to read");

        if let mavlink::common::MavMessage::COMMAND_INT(recv_msg) = recv_msg {
            assert_eq!(recv_msg.command, mavlink::common::MavCmd::MAV_CMD_NAV_TAKEOFF);
        } else {
            panic!("Decoded wrong message type")
        }
    }


    #[test]
    pub fn test_echo_hil_actuator_controls() {
        let mut v = vec![];
        let send_msg = get_hil_actuator_controls_msg();

        mavlink::write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HIL_ACTUATOR_CONTROLS(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = mavlink::read_msg(&mut c).expect("Failed to read");
        if let mavlink::common::MavMessage::HIL_ACTUATOR_CONTROLS(recv_msg) = recv_msg {
            assert_eq!(mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            recv_msg.mode & mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
        } else {
            panic!("Decoded wrong message type")
        }
    }


    /// A COMMAND_LONG message with a truncated payload (allowed for empty fields)
    #[cfg(all(feature = "std", feature = "mavlink2"))]
    pub const COMMAND_LONG_TRUNCATED_V2: &'static [u8] = &[
        mavlink::MAV_STX_V2, 30, 0, 0, 0, 0, 50, //header
        76, 0, 0, //msg ID
        //truncated payload:
        0, 0, 230, 66, 0, 64, 156, 69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 1,
        // crc:
        188, 195];

    #[test]
    #[cfg(all(feature = "std", feature = "mavlink2"))]
    pub fn test_read_truncated_command_long() {
        let mut r = COMMAND_LONG_TRUNCATED_V2;
        let (_header, recv_msg) = mavlink::read_msg(&mut r).expect("Failed to parse COMMAND_LONG_TRUNCATED_V2");

        if let mavlink::common::MavMessage::COMMAND_LONG(recv_msg) = recv_msg {
            assert_eq!(recv_msg.command, mavlink::common::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    // TODO include a tlog file in this repo for testing
    //    #[test]
    //    #[cfg(all(feature = "std", not(feature="mavlink2")))]
    //    pub fn test_log_file() {
    //        use std::fs::File;
    //
    //        let path = "test.tlog";
    //        let mut f = File::open(path).unwrap();
    //
    //        loop {
    //            match self::mavlink::read_msg(&mut f) {
    //                Ok((_, msg)) => {
    //                    println!("{:#?}", msg);
    //                }
    //                Err(e) => {
    //                    println!("Error: {:?}", e);
    //                    match e.kind() {
    //                        std::io::ErrorKind::UnexpectedEof => {
    //                            break;
    //                        },
    //                        _ => {
    //                            panic!("Unexpected error");
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //    }
}