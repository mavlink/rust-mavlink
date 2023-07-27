//! Implements primitives to allow a fast routing of Mavlink messages.
//!
//!

use crate::connection::direct_serial::SerialConnection;
use crate::connection::udp::UdpConnection;
use crate::read_v2_raw_message;
use crate::MAVLinkV2MessageRaw;
use crate::Message;
use std::io;
use std::io::Write;

/// A RawMavConnection is a contract for a MavConnection with a
/// couple of functions that allow a bypass of the creation/parsing of messages for fast routing
/// between mavlink connections
/// The Message generic is necessary as we check the message validity from its CRC which is Mavlink
/// version dependent.
pub trait RawMavV2Connection<M: Message> {
    fn raw_write(&self, raw_msg: &mut MAVLinkV2MessageRaw) -> io::Result<usize>;
    fn raw_read(&self) -> io::Result<MAVLinkV2MessageRaw>;
}

impl<M: Message> RawMavV2Connection<M> for UdpConnection {
    fn raw_write(&self, raw_msg: &mut MAVLinkV2MessageRaw) -> io::Result<usize> {
        let mut guard = self.writer.lock().unwrap();
        let state = &mut *guard;
        #[cfg(feature = "routing")]
        raw_msg.patch_sequence::<M>(state.sequence);
        state.sequence = state.sequence.wrapping_add(1);

        let len = if let Some(addr) = state.dest {
            state.socket.send_to(&raw_msg.0, addr)?
        } else {
            0
        };
        Ok(len)
    }

    fn raw_read(&self) -> io::Result<MAVLinkV2MessageRaw> {
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
            let Ok(raw_message) = read_v2_raw_message(&mut state.recv_buf) else {
                continue;
            };
            if raw_message.has_valid_crc::<M>() {
                return Ok(raw_message);
            }
        }
    }
}

impl<M: Message> RawMavV2Connection<M> for SerialConnection {
    fn raw_write(&self, raw_msg: &mut MAVLinkV2MessageRaw) -> io::Result<usize> {
        let mut port = self.port.lock().unwrap();
        let mut sequence = self.sequence.lock().unwrap();
        raw_msg.patch_sequence::<M>(*sequence);
        *sequence = sequence.wrapping_add(1);

        let l = raw_msg.len();
        match (&mut *port).write_all(&raw_msg.0[..l]) {
            Ok(_) => Ok(l),
            Err(e) => Err(e),
        }
    }

    fn raw_read(&self) -> io::Result<MAVLinkV2MessageRaw> {
        let mut port = self.port.lock().unwrap();

        loop {
            let Ok(raw_message) = read_v2_raw_message(&mut *port) else {continue;};
            if raw_message.has_valid_crc::<M>() {
                return Ok(raw_message);
            }
        }
    }
}
