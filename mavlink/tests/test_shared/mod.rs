#![allow(unused)]

pub const COMMON_MSG_HEADER: mavlink::MavHeader = mavlink::MavHeader {
    sequence: 239,
    system_id: 1,
    component_id: 2,
};

#[cfg(feature = "signing")]
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
        flags: mavlink::common::HilActuatorControlsFlags::empty(),
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

#[cfg(feature = "ardupilotmega")]
pub fn get_apm_mount_status() -> mavlink::ardupilotmega::MOUNT_STATUS_DATA {
    mavlink::ardupilotmega::MOUNT_STATUS_DATA {
        pointing_a: 3,
        pointing_b: 4,
        pointing_c: 5,
        target_system: 2,
        target_component: 3,
        #[cfg(feature = "emit-extensions")]
        mount_mode: mavlink::ardupilotmega::MavMountMode::MAV_MOUNT_MODE_HOME_LOCATION,
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
        #[cfg(feature = "ardupilotmega")]
        {
            use ::mavlink::ardupilotmega::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "asluav")]
        {
            use ::mavlink::asluav::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "avssuas")]
        {
            use ::mavlink::avssuas::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "development")]
        {
            use ::mavlink::development::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "matrixpilot")]
        {
            use ::mavlink::matrixpilot::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "minimal")]
        {
            use ::mavlink::minimal::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "paparazzi")]
        {
            use ::mavlink::paparazzi::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "python_array_test")]
        {
            use ::mavlink::python_array_test::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "standard")]
        {
            use ::mavlink::standard::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "test")]
        {
            use ::mavlink::test::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "ualberta")]
        {
            use ::mavlink::ualberta::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "uavionix")]
        {
            use ::mavlink::uavionix::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "icarous")]
        {
            use ::mavlink::icarous::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "common")]
        {
            use ::mavlink::common::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "cubepilot")]
        {
            use ::mavlink::cubepilot::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "storm32")]
        {
            use ::mavlink::storm32::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "csairlink")]
        {
            use ::mavlink::csairlink::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "loweheiser")]
        {
            use ::mavlink::loweheiser::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "marsh")]
        {
            use ::mavlink::marsh::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }

        #[cfg(feature = "stemstudios")]
        {
            use ::mavlink::stemstudios::MavMessage;

            $function::<MavMessage, _>(MavMessage::all_ids(), $($args), *);
        }
    };
}
