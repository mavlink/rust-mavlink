#[cfg(test)]
mod tests {
    use mavlink::ardupilotmega::MavMessage;
    use mavlink::ardupilotmega::CHANGE_OPERATOR_CONTROL_DATA;
    use mavlink::ardupilotmega::DEBUG_DATA;
    use mavlink::ardupilotmega::PING_DATA;
    use mavlink::MavlinkVersion;
    use mavlink::Message;
    #[test]
    fn test_ping_target_ids() {
        let msg = MavMessage::PING(PING_DATA {
            time_usec: 0,
            seq: 0,
            target_system: 3,
            target_component: 14,
        });
        let mut buffer: [u8; 300] = [0; 300];

        msg.ser(MavlinkVersion::V2, &mut buffer);
        let (target_system_id_offset, target_component_id_offset) =
            MavMessage::target_offsets_from_id(msg.message_id());
        assert_eq!(
            buffer[target_system_id_offset
                .expect("There should be a target_system_id_offset on this message.")],
            3
        );
        assert_eq!(
            buffer[target_component_id_offset
                .expect("There should be a target_component_id_offset on this message.")],
            14
        );
    }

    #[test]
    fn test_no_target_ids() {
        let msg = MavMessage::DEBUG(DEBUG_DATA {
            time_boot_ms: 0,
            value: 0.0,
            ind: 0,
        });
        let mut buffer: [u8; 300] = [0; 300];

        msg.ser(MavlinkVersion::V2, &mut buffer);
        let (target_system_id_offset, target_component_id_offset) =
            MavMessage::target_offsets_from_id(msg.message_id());

        assert!(target_component_id_offset.is_none());
        assert!(target_system_id_offset.is_none());
    }

    #[test]
    fn test_just_one_target_ids() {
        let msg = MavMessage::CHANGE_OPERATOR_CONTROL(CHANGE_OPERATOR_CONTROL_DATA {
            target_system: 122,
            control_request: 0,
            version: 0,
            passkey: [0; 25],
        });
        let mut buffer: [u8; 300] = [0; 300];

        msg.ser(MavlinkVersion::V2, &mut buffer);
        let (target_system_id_offset, target_component_id_offset) =
            MavMessage::target_offsets_from_id(msg.message_id());

        assert!(target_component_id_offset.is_none());
        assert_eq!(buffer[target_system_id_offset.unwrap()], 122);
    }
}
