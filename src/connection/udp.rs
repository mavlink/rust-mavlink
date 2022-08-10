use crate::connection::MavConnection;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use std::io::Read;
use std::io::{self};
use std::net::ToSocketAddrs;
use std::net::{SocketAddr, UdpSocket};
use std::str::FromStr;
use std::sync::Mutex;

/// UDP MAVLink connection

pub fn select_protocol<M: Message>(
    address: &str,
) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
    if address.starts_with("udpin:") {
        Ok(Box::new(udpin(&address["udpin:".len()..])?))
    } else if address.starts_with("udpout:") {
        Ok(Box::new(udpout(&address["udpout:".len()..])?))
    } else if address.starts_with("udpbcast:") {
        Ok(Box::new(udpbcast(&address["udpbcast:".len()..])?))
    } else {
        Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Protocol unsupported",
        ))
    }
}

pub fn udpbcast<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Invalid address");
    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
    socket
        .set_broadcast(true)
        .expect("Couldn't bind to broadcast address.");
    UdpConnection::new(socket, false, Some(addr))
}

pub fn udpout<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Invalid address");
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    UdpConnection::new(socket, false, Some(addr))
}

pub fn udpin<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Invalid address");
    let socket = UdpSocket::bind(&addr)?;
    UdpConnection::new(socket, true, None)
}

struct UdpWrite {
    socket: UdpSocket,
    dest: Option<SocketAddr>,
    sequence: u8,
}

struct PacketBuf {
    buf: Vec<u8>,
    start: usize,
    end: usize,
}

impl PacketBuf {
    pub fn new() -> PacketBuf {
        let mut v = Vec::new();
        v.resize(65536, 0);
        PacketBuf {
            buf: v,
            start: 0,
            end: 0,
        }
    }

    pub fn reset(&mut self) -> &mut [u8] {
        self.start = 0;
        self.end = 0;
        &mut self.buf
    }

    pub fn set_len(&mut self, size: usize) {
        self.end = size;
    }

    pub fn slice(&self) -> &[u8] {
        &self.buf[self.start..self.end]
    }

    pub fn len(&self) -> usize {
        self.slice().len()
    }
}

impl Read for PacketBuf {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = Read::read(&mut self.slice(), buf)?;
        self.start += n;
        Ok(n)
    }
}

struct UdpRead {
    socket: UdpSocket,
    recv_buf: PacketBuf,
}

pub struct UdpConnection {
    reader: Mutex<UdpRead>,
    writer: Mutex<UdpWrite>,
    protocol_version: MavlinkVersion,
    server: bool,
}

impl UdpConnection {
    fn new(socket: UdpSocket, server: bool, dest: Option<SocketAddr>) -> io::Result<UdpConnection> {
        Ok(UdpConnection {
            server: server,
            reader: Mutex::new(UdpRead {
                socket: socket.try_clone()?,
                recv_buf: PacketBuf::new(),
            }),
            writer: Mutex::new(UdpWrite {
                socket: socket,
                dest: dest,
                sequence: 0,
            }),
            protocol_version: MavlinkVersion::V2,
        })
    }
}

impl<M: Message> MavConnection<M> for UdpConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut guard = self.reader.lock().unwrap();
        let state = &mut *guard;
        loop {
            if state.recv_buf.len() == 0 {
                let (len, src) = state.socket.recv_from(state.recv_buf.reset())?;
                state.recv_buf.set_len(len);

                if self.server {
                    self.writer.lock().unwrap().dest = Some(src);
                }
            }

            match read_versioned_msg(&mut state.recv_buf, self.protocol_version) {
                ok @ Ok(..) => return ok,
                _ => (),
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
