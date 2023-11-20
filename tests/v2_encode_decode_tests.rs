extern crate mavlink;

mod test_shared;

#[cfg(test)]
#[cfg(all(feature = "std", feature = "common"))]
mod test_v2_encode_decode {
    pub const HEARTBEAT_V2: &'static [u8] = &[
        mavlink::MAV_STX_V2, //magic
        0x09,                //payload len
        0,                   //incompat flags
        0,                   //compat flags
        crate::test_shared::COMMON_MSG_HEADER.sequence,
        crate::test_shared::COMMON_MSG_HEADER.system_id,
        crate::test_shared::COMMON_MSG_HEADER.component_id,
        0x00,
        0x00,
        0x00, //msg ID
        0x05,
        0x00,
        0x00,
        0x00,
        0x02,
        0x03,
        0x59,
        0x03,
        0x03, //payload
        46,
        115, //checksum
    ];

    #[test]
    pub fn test_read_v2_heartbeat() {
        let mut r = HEARTBEAT_V2;
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
            &mavlink::common::MavMessage::HEARTBEAT(heartbeat_msg.clone()),
        )
        .expect("Failed to write message");

        assert_eq!(&v[..], HEARTBEAT_V2);
    }

    /// A COMMAND_LONG message with a truncated payload (allowed for empty fields)
    pub const COMMAND_LONG_TRUNCATED_V2: &'static [u8] = &[
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
        let mut r = COMMAND_LONG_TRUNCATED_V2;
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

        let mut c = v.as_slice();
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
}
