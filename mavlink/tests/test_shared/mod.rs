#![allow(unused)]

pub const COMMON_MSG_HEADER: mavlink::MavHeader = mavlink::MavHeader {
    sequence: 239,
    system_id: 1,
    component_id: 2,
};

#[cfg(feature = "mav2-message-signing")]
pub const SECRET_KEY: [u8; 32] = [
    0x00, 0x01, 0xf2, 0xe3, 0xd4, 0xc5, 0xb6, 0xa7, 0x98, 0x00, 0x70, 0x76, 0x34, 0x32, 0x00, 0x16,
    0x22, 0x42, 0x00, 0xcc, 0xff, 0x7a, 0x00, 0x52, 0x75, 0x73, 0x74, 0x00, 0x4d, 0x41, 0x56, 0xb3,
];

pub const HEARTBEAT_V1: &[u8] = &[
    mavlink::MAV_STX,
    0x09,
    crate::test_shared::COMMON_MSG_HEADER.sequence,
    crate::test_shared::COMMON_MSG_HEADER.system_id,
    crate::test_shared::COMMON_MSG_HEADER.component_id,
    0x00, //msg ID
    0x05, //payload
    0x00,
    0x00,
    0x00,
    0x02,
    0x03,
    0x59,
    0x03,
    0x03,
    0x1f, //checksum
    0x50,
];

pub const HEARTBEAT_V2: &[u8] = &[
    mavlink::MAV_STX_V2, //magic
    0x09,                //payload len
    0,                   //incompat flags
    0,                   //compat flags
    crate::test_shared::COMMON_MSG_HEADER.sequence,
    crate::test_shared::COMMON_MSG_HEADER.system_id,
    crate::test_shared::COMMON_MSG_HEADER.component_id,
    0x00, //msg ID
    0x00,
    0x00,
    0x05, //payload
    0x00,
    0x00,
    0x00,
    0x02,
    0x03,
    0x59,
    0x03,
    0x03,
    46, //checksum
    115,
];

#[cfg(feature = "dialect-common")]
pub fn get_heartbeat_msg() -> mavlink::dialects::common::HEARTBEAT_DATA {
    mavlink::dialects::common::HEARTBEAT_DATA {
        custom_mode: 5,
        mavtype: mavlink::dialects::common::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::dialects::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            | mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
            | mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
            | mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: mavlink::dialects::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    }
}

#[cfg(feature = "dialect-common")]
pub fn get_cmd_nav_takeoff_msg() -> mavlink::dialects::common::COMMAND_INT_DATA {
    mavlink::dialects::common::COMMAND_INT_DATA {
        param1: 1.0,
        param2: 2.0,
        param3: 3.0,
        param4: 4.0,
        x: 555,
        y: 666,
        z: 777.0,
        command: mavlink::dialects::common::MavCmd::MAV_CMD_NAV_TAKEOFF,
        target_system: 42,
        target_component: 84,
        frame: mavlink::dialects::common::MavFrame::MAV_FRAME_GLOBAL,
        current: 73,
        autocontinue: 17,
    }
}

#[cfg(feature = "dialect-common")]
pub fn get_hil_actuator_controls_msg() -> mavlink::dialects::common::HIL_ACTUATOR_CONTROLS_DATA {
    mavlink::dialects::common::HIL_ACTUATOR_CONTROLS_DATA {
        time_usec: 1234567_u64,
        flags: mavlink::dialects::common::HilActuatorControlsFlags::empty(),
        controls: [
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
        ],
        mode: mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            | mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
            | mavlink::dialects::common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    }
}

#[cfg(all(feature = "dialect-common", not(feature = "mav2-message-extensions")))]
pub fn get_servo_output_raw_v1() -> mavlink::dialects::common::SERVO_OUTPUT_RAW_DATA {
    mavlink::dialects::common::SERVO_OUTPUT_RAW_DATA {
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

#[cfg(all(feature = "dialect-common", feature = "mav2-message-extensions"))]
pub fn get_servo_output_raw_v2() -> mavlink::dialects::common::SERVO_OUTPUT_RAW_DATA {
    mavlink::dialects::common::SERVO_OUTPUT_RAW_DATA {
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

#[cfg(feature = "dialect-ardupilotmega")]
pub fn get_apm_mount_status() -> mavlink::dialects::ardupilotmega::MOUNT_STATUS_DATA {
    mavlink::dialects::ardupilotmega::MOUNT_STATUS_DATA {
        pointing_a: 3,
        pointing_b: 4,
        pointing_c: 5,
        target_system: 2,
        target_component: 3,
        #[cfg(feature = "mav2-message-extensions")]
        mount_mode: mavlink::dialects::ardupilotmega::MavMountMode::MAV_MOUNT_MODE_HOME_LOCATION,
    }
}

pub struct BlockyReader<'a> {
    block_next_read: bool,
    data: &'a [u8],
    index: usize,
}

impl<'a> BlockyReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        BlockyReader {
            block_next_read: true,
            data,
            index: 0,
        }
    }
}

impl<'a> std::io::Read for BlockyReader<'a> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        use std::io::{Error, ErrorKind, Result};

        if self.block_next_read {
            self.block_next_read = false;
            Result::Err(Error::new(ErrorKind::WouldBlock, "Test Block"))
        } else {
            let read = self
                .data
                .get(self.index)
                .ok_or(Error::new(ErrorKind::UnexpectedEof, "EOF"));
            buf[0] = *read?;
            self.index += 1;
            self.block_next_read = true;
            Ok(1)
        }
    }
}

#[macro_export]
macro_rules! for_all_dialects {
    ($function:ident $(, $args:expr)* $(,)?) => {
        #[cfg(feature = "dialect-all")]
        {
            use ::mavlink::all::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-ardupilotmega")]
        {
            use ::mavlink::dialects::ardupilotmega::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-asluav")]
        {
            use ::mavlink::dialects::asluav::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-avssuas")]
        {
            use ::mavlink::dialects::avssuas::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-development")]
        {
            use ::mavlink::dialects::development::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-matrixpilot")]
        {
            use ::mavlink::dialects::matrixpilot::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-minimal")]
        {
            use ::mavlink::dialects::minimal::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-paparazzi")]
        {
            use ::mavlink::dialects::paparazzi::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-python_array_test")]
        {
            use ::mavlink::dialects::python_array_test::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-standard")]
        {
            use ::mavlink::dialects::standard::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-test")]
        {
            use ::mavlink::dialects::test::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-ualberta")]
        {
            use ::mavlink::dialects::ualberta::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-uavionix")]
        {
            use ::mavlink::dialects::uavionix::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-icarous")]
        {
            use ::mavlink::dialects::icarous::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-common")]
        {
            use ::mavlink::dialects::common::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-cubepilot")]
        {
            use ::mavlink::dialects::cubepilot::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-storm32")]
        {
            use ::mavlink::dialects::storm32::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-csairlink")]
        {
            use ::mavlink::dialects::csairlink::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-loweheiser")]
        {
            use ::mavlink::dialects::loweheiser::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-marsh")]
        {
            use ::mavlink::dialects::marsh::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "dialect-stemstudios")]
        {
            use ::mavlink::dialects::stemstudios::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }
    };
}
