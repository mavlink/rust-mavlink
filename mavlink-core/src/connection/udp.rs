//! UDP MAVLink connection

use std::collections::VecDeque;

use crate::connection::MavConnection;
use crate::peek_reader::PeekReader;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::io::{self, Read};
use std::net::ToSocketAddrs;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Mutex;

use super::get_socket_addr;

#[cfg(feature = "signing")]
use super::signing::{SigningConfig, SigningData};

pub fn select_protocol<M: Message>(
    address: &str,
) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
    let connection = if let Some(address) = address.strip_prefix("udpin:") {
        udpin(address)
    } else if let Some(address) = address.strip_prefix("udpout:") {
        udpout(address)
    } else if let Some(address) = address.strip_prefix("udpbcast:") {
        udpbcast(address)
    } else {
        Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Protocol unsupported",
        ))
    };

    Ok(Box::new(connection?))
}

pub fn udpbcast<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = get_socket_addr(address)?;
    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
    socket
        .set_broadcast(true)
        .expect("Couldn't bind to broadcast address.");
    UdpConnection::new(socket, false, Some(addr))
}

pub fn udpout<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = get_socket_addr(address)?;
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    UdpConnection::new(socket, false, Some(addr))
}

pub fn udpin<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Invalid address");
    let socket = UdpSocket::bind(addr)?;
    UdpConnection::new(socket, true, None)
}

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

pub struct UdpConnection {
    reader: Mutex<PeekReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
    protocol_version: MavlinkVersion,
    server: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
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
            protocol_version: MavlinkVersion::V2,
            #[cfg(feature = "signing")]
            signing_data: None,
        })
    }
}

impl<M: Message> MavConnection<M> for UdpConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();

        loop {
            let result = read_versioned_msg(reader.deref_mut(), self.protocol_version);
            if self.server {
                if let addr @ Some(_) = reader.reader_ref().last_recv_address {
                    self.writer.lock().unwrap().dest = addr;
                }
            }
            if let ok @ Ok(..) = result {
                return ok;
            }
        }
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let mut guard = self.writer.lock().unwrap();
        let state = &mut *guard;

        let header = MavHeader {
            incompat_flags: 0,
            sequence: state.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        state.sequence = state.sequence.wrapping_add(1);

        let len = if let Some(addr) = state.dest {
            let mut buf = Vec::new();
            write_versioned_msg(&mut buf, self.protocol_version, header, data)?;
            state.socket.send_to(&buf, addr)?
        } else {
            0
        };

        Ok(len)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(|cfg| SigningData::from_config(cfg))
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
