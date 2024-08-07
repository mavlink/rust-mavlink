#![allow(unused)]

pub const COMMON_MSG_HEADER: mavlink::MavHeader = mavlink::MavHeader {
    incompat_flags: 0,
    sequence: 239,
    system_id: 1,
    component_id: 2,
};

#[cfg(feature = "common")]
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

#[cfg(feature = "common")]
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
        autocontinue: 17,
    }
}

#[cfg(feature = "common")]
pub fn get_hil_actuator_controls_msg() -> mavlink::common::HIL_ACTUATOR_CONTROLS_DATA {
    mavlink::common::HIL_ACTUATOR_CONTROLS_DATA {
        time_usec: 1234567_u64,
        flags: 0_u64,
        controls: [
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        ],
        mode: mavlink::common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            | mavlink::common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
            | mavlink::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    }
}

#[cfg(all(feature = "common", not(feature = "emit-extensions")))]
pub fn get_servo_output_raw_v1() -> mavlink::common::SERVO_OUTPUT_RAW_DATA {
    mavlink::common::SERVO_OUTPUT_RAW_DATA {
        time_usec: 1234567_u32,
        servo1_raw: 1100_u16,
        servo2_raw: 1200_u16,
        servo3_raw: 1300_u16,
        servo4_raw: 1400_u16,
        servo5_raw: 1500_u16,
        servo6_raw: 1600_u16,
        servo7_raw: 1700_u16,
        servo8_raw: 1800_u16,
        port: 123_u8,
    }
}

#[cfg(all(feature = "common", feature = "emit-extensions"))]
pub fn get_servo_output_raw_v2() -> mavlink::common::SERVO_OUTPUT_RAW_DATA {
    mavlink::common::SERVO_OUTPUT_RAW_DATA {
        time_usec: 1234567_u32,
        servo1_raw: 1100_u16,
        servo2_raw: 1200_u16,
        servo3_raw: 1300_u16,
        servo4_raw: 1400_u16,
        servo5_raw: 1500_u16,
        servo6_raw: 1600_u16,
        servo7_raw: 1700_u16,
        servo8_raw: 1800_u16,
        port: 123_u8,
        servo9_raw: 1110_u16,
        servo10_raw: 1220_u16,
        servo11_raw: 1330_u16,
        servo12_raw: 1440_u16,
        servo13_raw: 1550_u16,
        servo14_raw: 1660_u16,
        servo15_raw: 1770_u16,
        servo16_raw: 1880_u16,
    }
}

#[cfg(all(feature = "ardupilotmega", feature = "uavionix", feature = "icarous"))]
pub fn get_apm_mount_status() -> mavlink::ardupilotmega::MOUNT_STATUS_DATA {
    mavlink::ardupilotmega::MOUNT_STATUS_DATA {
        pointing_a: 3,
        pointing_b: 4,
        pointing_c: 5,
        target_system: 2,
        target_component: 3,
    }
}
