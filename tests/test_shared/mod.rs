
extern crate mavlink;


pub const COMMON_MSG_HEADER: mavlink::MavHeader = mavlink::MavHeader {
    sequence: 239,
    system_id: 1,
    component_id: 1,
};

pub fn get_heartbeat_msg() -> mavlink::common::HEARTBEAT_DATA {
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



pub fn get_cmd_nav_takeoff_msg() -> mavlink::common::COMMAND_INT_DATA {
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

pub fn get_hil_actuator_controls_msg() -> mavlink::common::HIL_ACTUATOR_CONTROLS_DATA {
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

