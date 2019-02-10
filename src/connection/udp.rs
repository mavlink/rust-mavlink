

use std::net::{SocketAddr, UdpSocket};
use std::io::Read;
use std::str::FromStr;
use std::sync::Mutex;
use std::net::{ToSocketAddrs};
use std::io::{self};
use connection::MavConnection;
use crate::common::MavMessage;
use crate::{read_msg, write_msg, MavHeader};

/// UDP MAVLink connection


pub fn select_protocol(address: &str) -> io::Result<Box<MavConnection + Sync + Send>> {
    if address.starts_with("udpin:") {
        Ok(Box::new(udpin(&address["udpin:".len()..])?))
    } else if address.starts_with("udpout:") {
        Ok(Box::new(udpout(&address["udpout:".len()..])?))
    }
    else {
        Err(io::Error::new(io::ErrorKind::AddrNotAvailable,"Protocol unsupported"))
    }
}

pub fn udpout<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = address.to_socket_addrs().unwrap().next().expect("Invalid address");
    let socket = UdpSocket::bind(&SocketAddr::from_str("0.0.0.0:0").unwrap())?;
    UdpConnection::new(socket, false, Some(addr))
}

pub fn udpin<T: ToSocketAddrs>(address: T) -> io::Result<UdpConnection> {
    let addr = address.to_socket_addrs().unwrap().next().expect("Invalid address");
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
        })
    }


}

impl MavConnection for UdpConnection {
    fn recv(&self) -> io::Result<(MavHeader, MavMessage)> {
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

            if let Ok((h, m)) = read_msg(&mut state.recv_buf) {
                return Ok((h,m));
            }
        }
    }

    fn send(&self, header: &MavHeader, data: &MavMessage) -> io::Result<()> {
        let mut guard = self.writer.lock().unwrap();
        let state = &mut *guard;

        let header = MavHeader {
            sequence: state.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        state.sequence = state.sequence.wrapping_add(1);

        if let Some(addr) = state.dest {
            let mut buf = Vec::new();
            write_msg(&mut buf, header, data)?;
            state.socket.send_to(&buf, addr)?;
        }

        Ok(())
    }
}