
extern crate mavlink;

mod test_shared;


#[cfg(test)]
#[cfg(all(feature = "std"))]
mod test_v1_encode_decode {

    pub const HEARTBEAT_V1: &'static [u8] = &[
        mavlink::MAV_STX, 0x09, 0xef, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03,
        0xf1, 0xd7,
    ];

    #[test]
    pub fn test_read_heartbeat() {
        let mut r = HEARTBEAT_V1;
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
        let mut v = vec![];
        let heartbeat_msg = crate::test_shared::get_heartbeat_msg();
        mavlink::write_v1_msg(
            &mut v,
            crate::test_shared::COMMON_MSG_HEADER,
            &mavlink::common::MavMessage::HEARTBEAT(heartbeat_msg.clone()),
        )
            .expect("Failed to write message");

        assert_eq!(&v[..], HEARTBEAT_V1);
    }

}