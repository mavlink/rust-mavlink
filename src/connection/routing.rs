//! Implements primitives to allow a fast routing of Mavlink messages.
//!
//!

use crate::connection::direct_serial::SerialConnection;
use crate::connection::udp::UdpConnection;
use crate::read_raw_message;
use crate::read_v1_raw_message;
use crate::read_v2_raw_message;
use crate::CommonMessageRaw;
use crate::MAVLinkV1MessageRaw;
use crate::MAVLinkV2MessageRaw;
use crate::Message;
use log::debug;
use serialport::SerialPort;
use std::io;

pub enum MAVLinkMessageRaw {
    V1(MAVLinkV1MessageRaw),
    V2(MAVLinkV2MessageRaw),
}

/// TODO(gbin): There must be a more elegant way to do this
impl CommonMessageRaw for MAVLinkMessageRaw {
    #[inline]
    fn message_id(&self) -> u32 {
        match self {
            Self::V1(m) => m.message_id(),
            Self::V2(m) => m.message_id(),
        }
    }

    #[inline]
    fn system_id(&self) -> u8 {
        match self {
            Self::V1(m) => m.system_id(),
            Self::V2(m) => m.system_id(),
        }
    }

    #[inline]
    fn component_id(&self) -> u8 {
        match self {
            Self::V1(m) => m.component_id(),
            Self::V2(m) => m.component_id(),
        }
    }

    #[inline]
    fn len(&self) -> usize {
        match self {
            Self::V1(m) => m.len(),
            Self::V2(m) => m.len(),
        }
    }

    #[inline]
    fn full(&self) -> &[u8] {
        match self {
            Self::V1(m) => m.full(),
            Self::V2(m) => m.full(),
        }
    }

    fn payload_length(&self) -> usize {
        match self {
            Self::V1(m) => m.payload_length(),
            Self::V2(m) => m.payload_length(),
        }
    }

    fn payload(&self) -> &[u8] {
        match self {
            Self::V1(m) => m.payload(),
            Self::V2(m) => m.payload(),
        }
    }
}

/// A RawConnection is a contract for a MavConnection with a
/// couple of functions that allow a bypass of the creation/parsing of messages for fast routing
/// between mavlink connections
/// The Message generic is necessary as we check the message validity from its CRC which is Mavlink
/// version dependent.
pub trait RawConnection<M: Message> {
    fn raw_write(&self, raw_msg: &mut dyn CommonMessageRaw) -> io::Result<usize>;
    fn raw_read(&self) -> io::Result<MAVLinkMessageRaw>;
    fn connection_id(&self) -> String;
}

impl<M: Message> RawConnection<M> for UdpConnection {
    fn raw_write(&self, msg: &mut dyn CommonMessageRaw) -> io::Result<usize> {
        let mut guard = self.writer.lock().unwrap();
        let state = &mut *guard;
        let bf = std::time::Instant::now();
        state.sequence = state.sequence.wrapping_add(1);

        let len = if let Some(addr) = state.dest {
            state.socket.send_to(msg.full(), addr)?
        } else {
            0
        };
        if bf.elapsed().as_millis() > 100 {
            debug!("Took too long to write UDP: {}ms", bf.elapsed().as_millis());
        }

        Ok(len)
    }

    fn raw_read(&self) -> io::Result<MAVLinkMessageRaw> {
        let mut guard = self.reader.lock().unwrap();
        let state = &mut *guard;
        loop {
            if state.recv_buf.len() == 0 {
                let (len, src) = match state.socket.recv_from(state.recv_buf.reset()) {
                    Ok((len, src)) => (len, src),
                    Err(error) => {
                        return Err(error);
                    }
                };
                state.recv_buf.set_len(len);

                if self.server {
                    self.writer.lock().unwrap().dest = Some(src);
                }
            }
            if state.recv_buf.slice()[0] == crate::MAV_STX {
                let Ok(msg) = read_v1_raw_message(&mut state.recv_buf) else {
                 println!("Error reading message from UDP"); // TODO(gbin): log
                 continue;
             };
                return Ok(MAVLinkMessageRaw::V1(msg));
            } else {
                if state.recv_buf.slice()[0] != crate::MAV_STX_V2 {
                    println!("Invalid MAVLink magic"); // TODO(gbin): log
                    continue;
                }
                let Ok(msg) = read_v2_raw_message(&mut state.recv_buf) else {
                 println!("Error reading message from UDP");// TODO(gbin): log
                 continue;
             };
                if !msg.has_valid_crc::<M>() {
                    println!("Invalid CRC"); // TODO(gbin): log
                    continue;
                }
                return Ok(MAVLinkMessageRaw::V2(msg));
            }
        }
    }

    fn connection_id(&self) -> String {
        self.id.clone()
    }
}

impl<M: Message> RawConnection<M> for SerialConnection {
    fn raw_write(&self, msg: &mut dyn CommonMessageRaw) -> io::Result<usize> {
        let mut port = self.writer.lock().unwrap();
        let bf = std::time::Instant::now();
        let res = match (*port).write_all(msg.full()) {
            Ok(_) => Ok(msg.len()),
            Err(e) => Err(e),
        };

        if bf.elapsed().as_millis() > 100 {
            debug!(
                "Took too long to write serial: {}ms",
                bf.elapsed().as_millis()
            );
        }
        res
    }

    fn raw_read(&self) -> io::Result<MAVLinkMessageRaw> {
        let mut port = self.reader.lock().unwrap();

        loop {
            let Ok(msg) = read_raw_message::<Box<dyn SerialPort>,M>(&mut *port) else {
                continue;
            };
            match msg {
                // This is hard to generalize through a trait because of M
                MAVLinkMessageRaw::V1(v1) => {
                    if v1.has_valid_crc::<M>() {
                        return Ok(msg);
                    }
                }
                MAVLinkMessageRaw::V2(v2) => {
                    if v2.has_valid_crc::<M>() {
                        return Ok(msg);
                    }
                }
            };
        }
    }

    fn connection_id(&self) -> String {
        self.id.clone() // FIXME(gbin)
    }
}
