mod test_shared;

#[cfg(feature = "dialect-common")]
mod target_fields_tests {
    use mavlink::{
        Message,
        dialects::common::{MANUAL_CONTROL_DATA, MavMessage},
    };

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

    #[test]
    fn test_target_ids_mut_rewrite_in_place() {
        let data = crate::test_shared::get_cmd_nav_takeoff_msg();
        let mut msg = MavMessage::COMMAND_INT(data);

        *msg.target_system_id_mut().unwrap() = 1;
        *msg.target_component_id_mut().unwrap() = 2;

        assert_eq!(msg.target_system_id(), Some(1));
        assert_eq!(msg.target_component_id(), Some(2));
    }

    #[test]
    fn test_manual_control_target_is_system_id() {
        let data = MANUAL_CONTROL_DATA {
            target: 42,
            ..Default::default()
        };
        let mut msg = MavMessage::MANUAL_CONTROL(data);

        assert_eq!(msg.target_system_id(), Some(42));
        assert_eq!(msg.target_component_id(), None);

        *msg.target_system_id_mut().unwrap() = 1;

        assert_eq!(msg.target_system_id(), Some(1));
        assert_eq!(msg.target_component_id_mut(), None);
    }

    #[test]
    fn test_target_ids_mut_absent() {
        let data = crate::test_shared::get_heartbeat_msg();
        let mut msg = MavMessage::HEARTBEAT(data);

        assert_eq!(msg.target_system_id_mut(), None);
        assert_eq!(msg.target_component_id_mut(), None);
    }
}
