use crate::ardupilotmega::MavMessage;
use serde_json::Value;
use std::error::Error;

/// mavlink_to_json_value
/// Helper function that converts an `ardupilotmega::MavMessage` type to a `serde_json::Value` type.
pub fn mavlink_to_json_value(mavlink_message: &MavMessage) -> Result<Value, Box<dyn Error>> {
    let serialised = serde_json::to_value(&mavlink_message)?;
    return Ok(serialised);
}

/// mavlink_to_json_str
/// Helper function that converts an `ardupilotmega::MavMessage` type to a `String` type.
pub fn mavlink_to_json_str(mavlink_message: &MavMessage) -> Result<String, Box<dyn Error>> {
    let serialised = serde_json::to_string(&mavlink_message)?;
    return Ok(serialised);
}

/// json_value_to_mavlink
/// Helper function that converts an `serde_json::Value` type to a `MavMessage` type.
pub fn json_value_to_mavlink(mavlink_message: &Value) -> Result<MavMessage, Box<dyn Error>> {
    let frame = serde_json::from_value::<MavMessage>(mavlink_message.clone())?;
    return Ok(frame);
}

/// json_str_to_mavlink
/// Helper function that converts an `&str` type to a `MavMessage` type.
pub fn json_str_to_mavlink(mavlink_message: &str) -> Result<MavMessage, Box<dyn Error>> {
    let frame = serde_json::from_str::<MavMessage>(&mavlink_message)?;
    return Ok(frame);
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// Create a heartbeat message using 'ardupilotmega' dialect
    fn create_heartbeat_message() -> crate::ardupilotmega::MavMessage {
        crate::ardupilotmega::MavMessage::HEARTBEAT(crate::ardupilotmega::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: crate::ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
            autopilot: crate::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: crate::ardupilotmega::MavModeFlag::empty(),
            system_status: crate::ardupilotmega::MavState::MAV_STATE_STANDBY,
            mavlink_version: 0x3,
        })
    }

    /// Create a message requesting the parameters list
    fn create_request_parameters() -> crate::ardupilotmega::MavMessage {
        crate::ardupilotmega::MavMessage::PARAM_REQUEST_LIST(
            crate::ardupilotmega::PARAM_REQUEST_LIST_DATA {
                target_system: 0,
                target_component: 0,
            },
        )
    }

    /// Create a message enabling data streaming
    fn create_request_stream() -> crate::ardupilotmega::MavMessage {
        crate::ardupilotmega::MavMessage::REQUEST_DATA_STREAM(
            crate::ardupilotmega::REQUEST_DATA_STREAM_DATA {
                target_system: 0,
                target_component: 0,
                req_stream_id: 0,
                req_message_rate: 10,
                start_stop: 1,
            },
        )
    }

    fn create_mavlink_header() -> mavlink_core::MavHeader {
        mavlink_core::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 42,
        }
    }

    #[test]
    fn test_mavlink_to_json_value() -> Result<(), Box<dyn Error>>{
        let heartbeat_message = create_heartbeat_message();
        let a = mavlink_to_json_value(&heartbeat_message)?;
        let b = json!(
            {
                "autopilot": {
                    "type": "MAV_AUTOPILOT_ARDUPILOTMEGA"
                },
                "base_mode": {
                    "bits": 0
                },
                "custom_mode": 0,
                "mavlink_version": 3,
                "mavtype": {
                    "type": "MAV_TYPE_QUADROTOR"
                },
                "system_status": {
                    "type": "MAV_STATE_STANDBY"
                },
                "type": "HEARTBEAT"
            }
        );
        assert_eq!(a, b);
        Ok(())
    }

    #[test]
    fn test_json_value_to_mavlink() -> Result<(), Box<dyn Error>>{
        let a = json!(
            {
                "autopilot": {
                    "type": "MAV_AUTOPILOT_ARDUPILOTMEGA"
                },
                "base_mode": {
                    "bits": 0
                },
                "custom_mode": 0,
                "mavlink_version": 3,
                "mavtype": {
                    "type": "MAV_TYPE_QUADROTOR"
                },
                "system_status": {
                    "type": "MAV_STATE_STANDBY"
                },
                "type": "HEARTBEAT"
            }
        );
        let a = json_value_to_mavlink(&a)?;
        let b = create_heartbeat_message();
        assert_eq!(a, b);
        Ok(())
    }
}
