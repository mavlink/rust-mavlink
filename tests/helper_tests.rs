extern crate mavlink;

#[cfg(test)]
#[cfg(all(feature = "std", feature = "common"))]
mod helper_tests {
    use crate::mavlink::{common::MavMessage, Message};

    #[test]
    fn test_get_default_message_from_id() {
        let message_name = "PING";
        let id: std::result::Result<u32, &'static str> =
            MavMessage::message_id_from_name(message_name);
        let id = id.unwrap();
        assert!(id == 4, "Invalid id for message name: PING");
        let message = MavMessage::default_message_from_id(id);
        match message {
            Ok(MavMessage::PING(_)) => {}
            _ => unreachable!("Invalid message type."),
        }
        assert!(
            message.unwrap().message_name() == message_name,
            "Message name does not match"
        );
    }
}
