use crate::connection::MavConnection;
use crate::peek_reader::PeekReader;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::io::{self, Read};
use std::net::ToSocketAddrs;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Mutex;

use super::get_socket_addr;

/// UDP MAVLink connection

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
    last_recv_address: Option<SocketAddr>,
}

impl Read for UdpRead {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.socket.recv_from(buf).map(|(n, addr)| {
            self.last_recv_address = Some(addr);
            n
        })
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
}

impl UdpConnection {
    fn new(socket: UdpSocket, server: bool, dest: Option<SocketAddr>) -> io::Result<Self> {
        Ok(Self {
            server,
            reader: Mutex::new(PeekReader::new(UdpRead {
                socket: socket.try_clone()?,
                last_recv_address: None,
            })),
            writer: Mutex::new(UdpWrite {
                socket,
                dest,
                sequence: 0,
            }),
            protocol_version: MavlinkVersion::V2,
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

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }
}
