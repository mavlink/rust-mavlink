//! UDP MAVLink connection

use crate::Connectable;
use crate::MAVLinkMessageRaw;
use crate::connection::get_socket_addr;
use crate::connection::{Connection, MavConnection};
use crate::connection_shared::{
    ConnectionState, next_send_header, read_message, read_raw_message, write_message,
    write_raw_message,
};
use crate::peek_reader::PeekReader;
use crate::{MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::sync::Mutex;

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

use super::config::{UdpConfig, UdpMode};

struct UdpRead {
    socket: UdpSocket,
    buffer: VecDeque<u8>,
    last_recv_address: Option<SocketAddr>,
}

const MTU_SIZE: usize = 1500;
impl Read for UdpRead {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if !self.buffer.is_empty() {
            self.buffer.read(buf)
        } else {
            let mut read_buffer = [0u8; MTU_SIZE];
            let (n_buffer, address) = self.socket.recv_from(&mut read_buffer)?;
            let n = (&read_buffer[0..n_buffer]).read(buf)?;
            self.buffer.extend(&read_buffer[n..n_buffer]);

            self.last_recv_address = Some(address);
            Ok(n)
        }
    }
}

struct UdpWrite {
    socket: UdpSocket,
    dest: Option<SocketAddr>,
    sequence: u8,
}

impl Write for UdpWrite {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let addr = self.dest.expect("`dest` is checked before write");
        self.socket.send_to(buf, addr)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        if self.write(buf)? != buf.len() {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "failed to send complete UDP datagram",
            ));
        }

        Ok(())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

pub struct UdpConnection {
    reader: Mutex<PeekReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
    state: ConnectionState,
    server: bool,
}

impl UdpConnection {
    fn new(socket: UdpSocket, server: bool, dest: Option<SocketAddr>) -> io::Result<Self> {
        Ok(Self {
            server,
            reader: Mutex::new(PeekReader::new(UdpRead {
                socket: socket.try_clone()?,
                buffer: VecDeque::new(),
                last_recv_address: None,
            })),
            writer: Mutex::new(UdpWrite {
                socket,
                dest,
                sequence: 0,
            }),
            state: ConnectionState::new(),
        })
    }

    fn update_reply_destination(&self, reader: &PeekReader<UdpRead>) {
        if self.server {
            if let addr @ Some(_) = reader.reader_ref().last_recv_address {
                self.writer.lock().unwrap().dest = addr;
            }
        }
    }
}

impl<M: Message> MavConnection<M> for UdpConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();

        let result = read_message::<M, _>(reader.deref_mut(), &self.state);
        self.update_reply_destination(&reader);
        result
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();

        let result = read_raw_message::<M, _>(reader.deref_mut(), &self.state);
        self.update_reply_destination(&reader);
        result
    }

    fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();
        reader.reader_mut().socket.set_nonblocking(true)?;

        let result = read_message::<M, _>(reader.deref_mut(), &self.state);
        self.update_reply_destination(&reader);

        reader.reader_mut().socket.set_nonblocking(false)?;

        result
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let mut guard = self.writer.lock().unwrap();
        let writer = &mut *guard;

        let header = next_send_header(&mut writer.sequence, header);

        let len = if writer.dest.is_some() {
            write_message(writer, &self.state, header, data)?
        } else {
            0
        };

        Ok(len)
    }

    fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, crate::error::MessageWriteError> {
        let mut guard = self.writer.lock().unwrap();
        let writer = &mut *guard;

        let len = if writer.dest.is_some() {
            write_raw_message(writer, data)?
        } else {
            0
        };

        Ok(len)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.state.set_protocol_version(version);
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.state.protocol_version()
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.state.set_allow_recv_any_version(allow);
    }

    fn allow_recv_any_version(&self) -> bool {
        self.state.allow_recv_any_version()
    }

    #[cfg(feature = "mav2-message-signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.state.setup_signing(signing_data);
    }
}

impl Connectable for UdpConfig {
    fn connect<M: Message>(&self) -> io::Result<Connection<M>> {
        let (addr, server, dest): (&str, _, _) = match self.mode {
            UdpMode::Udpin => (&self.address, true, None),
            _ => ("0.0.0.0:0", false, Some(get_socket_addr(&self.address)?)),
        };
        let socket = UdpSocket::bind(addr)?;
        if let Some(timeout) = self.read_timeout {
            socket.set_read_timeout(Some(timeout))?;
        }
        if matches!(self.mode, UdpMode::UdpBroadcast) {
            socket.set_broadcast(true)?;
        }
        Ok(UdpConnection::new(socket, server, dest)?.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_datagram_buffering() {
        let receiver_socket = UdpSocket::bind("127.0.0.1:5000").unwrap();
        let mut udp_reader = UdpRead {
            socket: receiver_socket.try_clone().unwrap(),
            buffer: VecDeque::new(),
            last_recv_address: None,
        };
        let sender_socket = UdpSocket::bind("0.0.0.0:0").unwrap();
        sender_socket.connect("127.0.0.1:5000").unwrap();

        let datagram: Vec<u8> = (0..50).collect::<Vec<_>>();

        let mut n_sent = sender_socket.send(&datagram).unwrap();
        assert_eq!(n_sent, datagram.len());
        n_sent = sender_socket.send(&datagram).unwrap();
        assert_eq!(n_sent, datagram.len());

        let mut buf = [0u8; 30];

        let mut n_read = udp_reader.read(&mut buf).unwrap();
        assert_eq!(n_read, 30);
        assert_eq!(&buf[0..n_read], (0..30).collect::<Vec<_>>().as_slice());

        n_read = udp_reader.read(&mut buf).unwrap();
        assert_eq!(n_read, 20);
        assert_eq!(&buf[0..n_read], (30..50).collect::<Vec<_>>().as_slice());

        n_read = udp_reader.read(&mut buf).unwrap();
        assert_eq!(n_read, 30);
        assert_eq!(&buf[0..n_read], (0..30).collect::<Vec<_>>().as_slice());

        n_read = udp_reader.read(&mut buf).unwrap();
        assert_eq!(n_read, 20);
        assert_eq!(&buf[0..n_read], (30..50).collect::<Vec<_>>().as_slice());
    }
}
