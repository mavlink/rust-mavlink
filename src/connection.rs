use common::MavMessage;
use {Header, read, write};
use crc16;

use std::net::{TcpStream, UdpSocket, ToSocketAddrs, SocketAddr};
use std::io::{self, Cursor};

use std::str::FromStr;

pub fn parse_mavlink_string(buf: &[u8]) -> String {
    buf.iter()
       .take_while(|a| **a != 0)
       .map(|x| *x as char)
       .collect::<String>()
}

pub enum MavSocket {
    Tcp(TcpStream),
    Udp { 
        socket: UdpSocket,
        recv_buf: Cursor<Vec<u8>>,
        dest: Option<SocketAddr>,
        server: bool
    },
}

pub fn socket_tcp<T: ToSocketAddrs>(address: T) -> io::Result<MavSocket> {
    let addr = address.to_socket_addrs().unwrap().next().unwrap();
    let socket = try!(TcpStream::connect(&addr));
    Ok(MavSocket::Tcp(socket))
}

pub fn socket_udpin<T: ToSocketAddrs>(address: T) -> io::Result<MavSocket> {
    let addr = address.to_socket_addrs().unwrap().next().unwrap();
    let socket = try!(UdpSocket::bind(&addr));
    Ok(MavSocket::Udp { socket: socket, recv_buf: Cursor::new(Vec::new()), dest: None, server: true })
}

pub fn socket_udpout<T: ToSocketAddrs>(address: T) -> io::Result<MavSocket> {
    let addr = address.to_socket_addrs().unwrap().next().unwrap();
    let socket = try!(UdpSocket::bind(&SocketAddr::from_str("0.0.0.0:0").unwrap()));
    Ok(MavSocket::Udp{ socket: socket, recv_buf: Cursor::new(Vec::new()), dest: Some(addr), server: false })
}

pub fn connect(address: &str) -> io::Result<Connection> {
    connect_socket(if address.starts_with("tcp:") {
        try!(socket_tcp(&address["tcp:".len()..]))
    } else if address.starts_with("udpin:") {
        try!(socket_udpin(&address["udpin:".len()..]))
    } else if address.starts_with("udpout:") {
        try!(socket_udpout(&address["udpout:".len()..]))
    } else {
        return Err(io::Error::new(io::ErrorKind::AddrNotAvailable, "Prefix must be one of udpin, udpout, or tcp"));
    })
}

pub struct Connection {
    socket: MavSocket,
    sequence: u8,
}

pub fn connect_socket(socket: MavSocket) -> io::Result<Connection> {
    Ok(Connection {
        socket: socket,
        sequence: 0,
    })
}

impl Connection {        
    pub fn recv(&mut self) -> io::Result<MavMessage> {
        match self.socket {
            MavSocket::Tcp(ref mut socket) => {
                read(socket).map(|(_, pkt)| pkt)
            }
            MavSocket::Udp { ref mut socket, ref mut recv_buf, ref mut dest, server } => {
                loop {
                    if recv_buf.position() as usize >= recv_buf.get_ref().len() {
                        recv_buf.set_position(0);
                        let mut buf = recv_buf.get_mut();
                        buf.resize(64*1024, 0);
                        trace!("Waiting for UDP packet");
                        let (len, src) = try!(socket.recv_from(&mut buf));
                        buf.truncate(len);
                        trace!("UDP received {} bytes", len);
                        
                        if server {
                            *dest = Some(src);
                        }
                    }
                    
                    if let Ok((_, m)) = read(recv_buf) {
                        return Ok(m);
                    } else {
                        // Drop the remainder of the packet
                        recv_buf.set_position(0);
                        recv_buf.get_mut().truncate(0);
                    }
                }
            }
        }
    }

    pub fn send(&mut self, data: &MavMessage) -> io::Result<()> {
        let header = Header {
            sequence: self.sequence,
            system_id: 255,
            component_id: 0,
        };
        
        self.sequence = self.sequence.wrapping_add(1);
        
        match self.socket {
            MavSocket::Tcp(ref mut socket) => {
                try!(write(socket, header, data));
            }
            MavSocket::Udp { ref mut socket, dest: Some(ref addr), .. } => {
                let mut buf = Vec::new();
                try!(write(&mut buf, header, data));
                try!(socket.send_to(&buf, addr));
            }
            MavSocket::Udp { dest: None, .. } => {}
        }
        
        Ok(())
    }
}
