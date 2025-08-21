mod test_shared;

#[cfg(feature = "common")]
mod target_fields_tests {
    use mavlink::{common::MavMessage, Message};

    #[test]
    fn test_target_ids_present() {
        let data = crate::test_shared::get_cmd_nav_takeoff_msg();
        let msg = MavMessage::COMMAND_INT(data);

        assert_eq!(msg.target_system_id(), Some(42));
        assert_eq!(msg.target_component_id(), Some(84));
    }

    #[test]
    fn test_target_ids_absent() {
        let data = crate::test_shared::get_heartbeat_msg();
        let msg = MavMessage::HEARTBEAT(data);

        assert_eq!(msg.target_system_id(), None);
        assert_eq!(msg.target_component_id(), None);
    }
}
