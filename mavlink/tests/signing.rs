mod test_shared;

#[cfg(feature = "signing")]
mod signing {
    use mavlink::{
        common::HEARTBEAT_DATA, peek_reader::PeekReader, read_v2_raw_message, MAVLinkV2MessageRaw,
        MavHeader, SigningConfig, SigningData, MAV_STX_V2,
    };

    use crate::test_shared::SECRET_KEY;

    const HEARTBEAT_SIGNED: &[u8] = &[
        MAV_STX_V2,
        0x09,
        0x01, // MAVLINK_IFLAG_SIGNED
        0x00,
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
        0xc9, // checksum
        0x8b,
        0x00, // link_id
        0xff, // use max timestamp to ensure test will never fail against current time
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0x27, // signature
        0x18,
        0xb1,
        0x68,
        0xcc,
        0xf5,
    ];

    #[test]
    pub fn test_verify() {
        let signing_cfg = SigningConfig::new(SECRET_KEY, true, false);
        let signing_data = SigningData::from_config(signing_cfg);
        let mut r = PeekReader::new(HEARTBEAT_SIGNED);
        let msg = read_v2_raw_message::<mavlink::common::MavMessage, _>(&mut r).unwrap();
        assert!(
            signing_data.verify_signature(&msg),
            "Message verification failed"
        );
    }

    #[test]
    pub fn test_invalid_ts() {
        let signing_cfg = SigningConfig::new(SECRET_KEY, true, false);
        let signing_data = SigningData::from_config(signing_cfg);
        let mut r = PeekReader::new(HEARTBEAT_SIGNED);
        let mut msg = read_v2_raw_message::<mavlink::common::MavMessage, _>(&mut r).unwrap();
        msg.signature_timestamp_bytes_mut()
            .copy_from_slice(&[0, 0, 0, 0, 0, 0]); // set timestamp to min causing the timestamp test to fail
        assert!(
            !signing_data.verify_signature(&msg),
            "Invalid message verified"
        );
    }

    #[test]
    pub fn test_sign_verify() {
        use mavlink::common::MavMessage;
        let heartbeat_message = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        let mut message = MAVLinkV2MessageRaw::new();
        let header = MavHeader {
            system_id: 4,
            component_id: 3,
            sequence: 42,
        };
        message.serialize_message_for_signing(header, &heartbeat_message);

        let signing_cfg = SigningConfig::new(SECRET_KEY, true, false);
        let signing_data = SigningData::from_config(signing_cfg);
        signing_data.sign_message(&mut message);
        assert!(
            signing_data.verify_signature(&message),
            "Message verification failed"
        );
        // the same message must not be allowed to be verified again
        assert!(
            !signing_data.verify_signature(&message),
            "Invalid message verified"
        );
    }
}
