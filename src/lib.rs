//! The MAVLink common message set
//!
//! TODO: a parser for no_std environments
#![cfg_attr(not(feature = "std"), feature(alloc))]
#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(feature = "std"))]
extern crate alloc;



#[cfg(feature = "std")]
use std::io::{Read, Result, Write};


#[cfg(feature = "std")]
extern crate byteorder;
#[cfg(feature = "std")]
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

#[cfg(feature = "std")]
mod connection;
#[cfg(feature = "std")]
pub use self::connection::{connect, MavConnection, Tcp};
#[cfg(feature = "serial")]
pub use self::connection::{Serial};
#[cfg(feature = "udp")]
pub use self::connection::{Udp};

extern crate bytes;
use bytes::{Buf, Bytes, IntoBuf};

#[cfg(all(feature = "std", feature="mavlink2"))]
use std::mem::transmute;

#[cfg(all(not(feature = "std"), feature="mavlink2"))]
use core::mem::transmute;

extern crate num_traits;
extern crate num_derive;
extern crate bitflags;
#[macro_use]

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod common {
    include!(concat!(env!("OUT_DIR"), "/common.rs"));
}

/// Encapsulation of all possible Mavlink messages
pub use self::common::MavMessage as MavMessage;

/// Metadata from a MAVLink packet header
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct MavHeader {
    pub system_id: u8,
    pub component_id: u8,
    pub sequence: u8,
}


#[allow(dead_code)]
const MAV_STX: u8 = 0xFE;

#[allow(dead_code)]
const MAV_STX_V2: u8 = 0xFD;


impl MavHeader {
    /// Return a default GCS header, seq is replaced by the connector
    /// so it can be ignored. Set `component_id` to your desired component ID.
    pub fn get_default_header() -> MavHeader {
        MavHeader {
            system_id: 255,
            component_id: 0,
            sequence: 0,
        }
    }
}

/// Encapsulation of the Mavlink message and the header,
/// important to preserve information about the sender system
/// and component id
#[derive(Debug, Clone)]
pub struct MavFrame {
    pub header: MavHeader,
    pub msg: MavMessage,
}

impl MavFrame {
    /// Create a new frame with given message
    pub fn new(msg: MavMessage) -> MavFrame {
        MavFrame {
            header: MavHeader::get_default_header(),
            msg
        }
    }

    /// Serialize frame into a vector, so it can be send
    /// over a socket for example
    pub fn ser(&self) -> Vec<u8> {
        let mut v = vec![];

        // serialize header
        v.push(self.header.system_id);
        v.push(self.header.component_id);
        v.push(self.header.sequence);

        // message id
        #[cfg(feature="mavlink2")]
        {
            let bytes: [u8; 4] = unsafe { transmute(self.msg.message_id().to_le()) };
            v.extend_from_slice(&bytes);
        }
        #[cfg(not(feature="mavlink2"))]
        v.push(self.msg.message_id());

        // serialize message
        v.append(&mut self.msg.ser());

        v
    }

    /// Deserialize MavFrame from a slice that has been received from
    /// for example a socket.
    pub fn deser(input: &[u8]) -> Option<Self> {
        let mut buf = Bytes::from(input).into_buf();

        let system_id = buf.get_u8();
        let component_id = buf.get_u8();
        let sequence = buf.get_u8();
        let header = MavHeader{system_id,component_id,sequence};

        #[cfg(not(feature="mavlink2"))]
        let msg_id = buf.get_u8();

        #[cfg(feature="mavlink2")]
        let msg_id = buf.get_u32_le();

        if let Some(msg) = MavMessage::parse(msg_id, &buf.collect::<Vec<u8>>()) {
            Some(MavFrame {header, msg})
        } else {
            None
        }
    }

    /// Return the frame header
    pub fn header(&self) -> MavHeader {
        self.header
    }
}

/// Read a MAVLink v1  message from a Read stream.
#[cfg(all(feature = "std", not(feature="mavlink2")))]
pub fn read_msg<R: Read>(r: &mut R) -> Result<(MavHeader, MavMessage)> {
    loop {
        if r.read_u8()? != MAV_STX {
            continue;
        }
        let len = r.read_u8()? as usize;
        let seq = r.read_u8()?;
        let sysid = r.read_u8()?;
        let compid = r.read_u8()?;
        let msgid = r.read_u8()?;

        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..len];
        r.read_exact(payload)?;

        let crc = r.read_u16::<LittleEndian>()?;

        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();
        crc_calc.update(&[len as u8, seq, sysid, compid, msgid]);
        crc_calc.update(payload);
        crc_calc.update(&[MavMessage::extra_crc(msgid)]);
        let recvd_crc = crc_calc.get();
        if recvd_crc != crc {
            println!("msg id {} len {} , crc got {} expected {}", msgid, len, crc, recvd_crc );
            continue;
        }

        if let Some(msg) = MavMessage::parse(msgid, payload) {
            return Ok((
                MavHeader {
                    sequence: seq,
                    system_id: sysid,
                    component_id: compid,
                },
                msg,
            ));
        }
    }
}

#[cfg(feature="mavlink2")]
const MAVLINK_IFLAG_SIGNED: u8 = 0x01;

const  MAVLINK_CORE_HEADER_LEN: usize =  9; ///< Length of core header (of the comm. layer)
///
/// Read a MAVLink v2  message from a Read stream.
#[cfg(all(feature = "std", feature="mavlink2"))]
pub fn read_msg<R: Read>(r: &mut R) -> Result<(MavHeader, MavMessage)> {
    loop {
        if r.read_u8()? != MAV_STX_V2 {
            continue;
        }
        let mut header_buf = [0; MAVLINK_CORE_HEADER_LEN];
        let mut idx = 0;

//        println!("Got STX2");
        let payload_len = r.read_u8()? as usize;
        header_buf[idx] = payload_len as u8; idx+=1;
//        println!("Got payload_len: {}", payload_len);
        let incompat_flags = r.read_u8()?;
        header_buf[idx] = incompat_flags as u8; idx+=1;
//        println!("Got incompat flags: {}", incompat_flags);
        let compat_flags = r.read_u8()?;
        header_buf[idx] = compat_flags as u8; idx+=1;
//        println!("Got compat flags: {}", compat_flags);

        let seq = r.read_u8()?;
        header_buf[idx] = seq as u8; idx+=1;
//        println!("Got seq: {}", seq);

        let sysid = r.read_u8()?;
        header_buf[idx] = sysid as u8; idx+=1;
//        println!("Got sysid: {}", sysid);

        let compid = r.read_u8()?;
        header_buf[idx] = compid as u8; idx+=1;
//        println!("Got compid: {}", compid);

        let mut msgid_buf = [0;4];
        msgid_buf[0] = r.read_u8()?;
        msgid_buf[1] = r.read_u8()?;
        msgid_buf[2] = r.read_u8()?;
        header_buf[idx] =  msgid_buf[0]; idx+=1;
        header_buf[idx] =  msgid_buf[1]; idx+=1;
        header_buf[idx] =  msgid_buf[2]; idx+=1;
//        println!("Got msgid buf: {:?} header_len: {} ", msgid_buf,idx);
//        println!("read H: {:?} header_len: {} ",header_buf , idx );

        let msgid: u32 = unsafe { transmute(msgid_buf) };
//        println!("Got msgid: {}", msgid);

        let mut payload_buf = [0; 255];
        let payload = &mut payload_buf[..payload_len];
        r.read_exact(payload)?;

        let crc = r.read_u16::<LittleEndian>()?;
//        println!("got crc: {}", crc);

        if (incompat_flags & 0x01) == MAVLINK_IFLAG_SIGNED {
            let mut sign = [0;13];
            r.read_exact(&mut sign)?;
        }

        let mut crc_calc = crc16::State::<crc16::MCRF4XX>::new();

        crc_calc.update(&header_buf);
//        let header_crc = crc_calc.get();
        //        let value_buf = &[len as u8, seq, sysid, compid, msgid_buf[0],msgid_buf[1],msgid_buf[2]];
//        crc_calc.update(value_buf);
        crc_calc.update(payload);

//        let base_crc = crc_calc.get();
        let extra_crc = MavMessage::extra_crc(msgid);
//        println!("read header_crc: {} base_crc: {} extra_crc: {}",
//                 header_crc, base_crc, extra_crc);

        crc_calc.update(&[extra_crc]);
        let recvd_crc = crc_calc.get();
        if recvd_crc != crc {
            println!("msg id {} payload_len {} , crc got {} expected {}", msgid, payload_len, crc, recvd_crc );
            continue;
        }

        if let Some(msg) = MavMessage::parse(msgid, payload) {
            return Ok((
                MavHeader {
                    sequence: seq,
                    system_id: sysid,
                    component_id: compid,
                },
                msg,
            ));
        }
        else {
            println!("invalid MavMessage::parse");
        }
    }
}

/// Write a MAVLink v2 message to a Write stream.
#[cfg(all(feature = "std", feature="mavlink2"))]
pub fn write_msg<W: Write>(w: &mut W, header: MavHeader, data: &MavMessage) -> Result<()> {
    let msgid = data.message_id();
    let payload = data.ser();
//    println!("write payload_len : {}", payload.len());

    let header = &[
        MAV_STX_V2,
        payload.len() as u8,
        0, //incompat_flags
        0, //compat_flags
        header.sequence,
        header.system_id,
        header.component_id,
        (msgid & 0x0000FF) as u8,
        ((msgid & 0x00FF00) >> 8) as u8 ,
        ((msgid & 0xFF0000) >> 16) as u8,
    ];

//    println!("write H: {:?}",header );


    let mut crc = crc16::State::<crc16::MCRF4XX>::new();
    crc.update(&header[1..]);
//    let header_crc = crc.get();
    crc.update(&payload[..]);
//    let base_crc = crc.get();
    let extra_crc = MavMessage::extra_crc(msgid);
//    println!("write header_crc: {} base_crc: {} extra_crc: {}",
//             header_crc, base_crc, extra_crc);
    crc.update(&[extra_crc]);

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_u16::<LittleEndian>(crc.get())?;

    Ok(())
}

/// Write a MAVLink v1 message to a Write stream.
#[cfg(all(feature = "std", not(feature="mavlink2")))]
pub fn write_msg<W: Write>(w: &mut W, header: MavHeader, data: &MavMessage) -> Result<()> {
    let msgid = data.message_id();
    let payload = data.ser();

    let header = &[
        MAV_STX,
        payload.len() as u8,
        header.sequence,
        header.system_id,
        header.component_id,
        msgid,
    ];

    let mut crc = crc16::State::<crc16::MCRF4XX>::new();
    crc.update(&header[1..]);
    crc.update(&payload[..]);
    crc.update(&[MavMessage::extra_crc(msgid)]);

    w.write_all(header)?;
    w.write_all(&payload[..])?;
    w.write_u16::<LittleEndian>(crc.get())?;

    Ok(())
}

#[cfg(test)]
mod test_message {
    use super::*;

    #[cfg(all(feature = "std", not(feature="mavlink2")))]
    pub const HEARTBEAT: &'static [u8] = &[
        MAV_STX, 0x09, 0xef, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03,
        0xf1, 0xd7,
    ];

    #[cfg(all(feature = "std", feature="mavlink2"))]
    pub const HEARTBEAT_V2: &'static [u8] = &[
        MAV_STX_V2,  //magic
        0x09, //payload len
        0, //incompat flags
        0, //compat flags
        0xef, //seq 239
        0x01,  //sys ID
        0x01, //comp ID
        0x00, 0x00, 0x00, //msg ID
        0x05, 0x00, 0x00, 0x00, 0x02, 0x03, 0x59, 0x03, 0x03, //payload
        16, 240,//checksum
    ];

    pub const COMMON_MSG_HEADER: MavHeader = MavHeader {
        sequence: 239,
        system_id: 1,
        component_id: 1,
    };

    fn get_heartbeat_msg() -> common::HEARTBEAT_DATA {
        common::HEARTBEAT_DATA {
            custom_mode: 5,
            mavtype: common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        }
    }


    fn get_cmd_nav_takeoff_msg() -> common::COMMAND_INT_DATA {
        common::COMMAND_INT_DATA {
            param1: 1.0,
            param2: 2.0,
            param3: 3.0,
            param4: 4.0,
            x: 555,
            y: 666,
            z: 777.0,
            command: common::MavCmd::MAV_CMD_NAV_TAKEOFF,
            target_system: 42,
            target_component: 84,
            frame: common::MavFrame::MAV_FRAME_GLOBAL,
            current: 73,
            autocontinue: 17
        }
    }

    fn get_cmd_long_msg() -> common::COMMAND_LONG_DATA {
        common::COMMAND_LONG_DATA {
            param1: 115 as f32, //MAVLINK_MSG_ID_HIL_STATE_QUATERNION
            param2: 5e3,
            param3: 3.0,
            param4: 4.0,
            param5: 5.0,
            param6: 6.0,
            param7: 7.0,
            command: common::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            target_system: 42,
            target_component: 84,
            confirmation: 6,
        }
    }

    fn get_hil_actuator_controls_msg() -> common::HIL_ACTUATOR_CONTROLS_DATA {
        common::HIL_ACTUATOR_CONTROLS_DATA {
            time_usec: 1234567 as u64,
            flags: 0 as u64,
            controls: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
                10.0, 11.0, 12.0, 13.0, 14.0, 15.0] ,
            mode: common::MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        }
    }



    #[test]
    pub fn test_read_heartbeat() {
        #[cfg(all(feature = "std", not(feature="mavlink2")))]
        let mut r = HEARTBEAT;
        #[cfg(all(feature = "std", feature="mavlink2"))]
        let mut r =  HEARTBEAT_V2;

        let (header, msg) = read_msg(&mut r).expect("Failed to parse message");

        println!("{:?}, {:?}", header, msg);

        assert_eq!(header, COMMON_MSG_HEADER);
        let heartbeat_msg = get_heartbeat_msg();

        if let common::MavMessage::HEARTBEAT(msg) = msg {
            assert_eq!(msg.custom_mode, heartbeat_msg.custom_mode);
            assert_eq!(msg.mavtype, heartbeat_msg.mavtype);
            assert_eq!(msg.autopilot, heartbeat_msg.autopilot);
            assert_eq!(msg.base_mode, heartbeat_msg.base_mode);
            assert_eq!(msg.system_status, heartbeat_msg.system_status);
            assert_eq!(msg.mavlink_version, heartbeat_msg.mavlink_version);
        } else {
            panic!("Decoded wrong message type")
        }
    }

    #[test]
    pub fn test_write_heartbeat() {
        let mut v = vec![];
        let heartbeat_msg = get_heartbeat_msg();
        write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &common::MavMessage::HEARTBEAT(heartbeat_msg.clone()),
        )
        .expect("Failed to write message");

        #[cfg(all(feature = "std", not(feature="mavlink2")))] {
            assert_eq!(&v[..], HEARTBEAT);
        }

        #[cfg(all(feature = "std", feature="mavlink2"))] {
            assert_eq!(&v[..], HEARTBEAT_V2);
        }
    }

    #[test]
    pub fn test_echo_heartbeat() {
        let mut v = vec![];
        let send_msg = get_heartbeat_msg();
        write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &common::MavMessage::HEARTBEAT(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = read_msg(&mut c).expect("Failed to read");
        assert_eq!(recv_msg.message_id(), 0);
    }

    #[test]
    pub fn test_echo_command_int() {
        let mut v = vec![];
        let send_msg = get_cmd_nav_takeoff_msg();

        write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &common::MavMessage::COMMAND_INT(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = read_msg(&mut c).expect("Failed to read");

        if let common::MavMessage::COMMAND_INT(recv_msg) = recv_msg {
            assert_eq!(recv_msg.command, common::MavCmd::MAV_CMD_NAV_TAKEOFF);
        } else {
            panic!("Decoded wrong message type")
        }
    }


    #[test]
    pub fn test_echo_command_long() {
        let mut v = vec![];
        let send_msg = get_cmd_long_msg(); //MAV_CMD_DO_SET_MODE

        write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &common::MavMessage::COMMAND_LONG(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = read_msg(&mut c).expect("Failed to read");
        if let common::MavMessage::COMMAND_LONG(recv_msg) = recv_msg {
            assert_eq!(recv_msg.command, common::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
        } else {
            panic!("Decoded wrong message type")
        }

    }

    #[test]
    pub fn test_echo_hil_actuator_controls() {
        let mut v = vec![];
        let send_msg = get_hil_actuator_controls_msg();

        write_msg(
            &mut v,
            COMMON_MSG_HEADER,
            &common::MavMessage::HIL_ACTUATOR_CONTROLS(send_msg.clone()),
        ).expect("Failed to write message");

        let mut c = v.as_slice();
        let (_header, recv_msg) = read_msg(&mut c).expect("Failed to read");
        if let common::MavMessage::HIL_ACTUATOR_CONTROLS(recv_msg) = recv_msg {
            assert_eq!(common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            recv_msg.mode & common::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
        } else {
            panic!("Decoded wrong message type")
        }

    }

//    #[test]
//    #[cfg(all(feature = "std", not(feature="mavlink2")))]
//    pub fn test_log_file() {
//        use std::fs::File;
//
//        let path = "test.tlog";
//        let mut f = File::open(path).unwrap();
//
//        loop {
//            match self::read_msg(&mut f) {
//                Ok((_, msg)) => {
//                    println!("{:#?}", msg);
//                }
//                Err(e) => {
//                    println!("Error: {:?}", e);
//                    match e.kind() {
//                        std::io::ErrorKind::UnexpectedEof => {
//                            break;
//                        },
//                        _ => {
//                            panic!("Unexpected error");
//                        }
//                    }
//                }
//            }
//        }
//    }

}
