use combined::MavMessage;
use {Header, read, write};

use std::sync::Mutex;
use std::net::{TcpStream, UdpSocket, ToSocketAddrs, SocketAddr};
use std::io::{self, Read};

use std::str::FromStr;

/// A MAVLink connection
pub trait MavConnection {
    /// Receive a mavlink message.
    ///
    /// Blocks until a valid frame is received, ignoring invalid messages.
    fn recv(&self) -> io::Result<MavMessage>;


    /// Send a mavlink message
    fn send(&self, data: &MavMessage) -> io::Result<()>;
}

/// Connect to a MAVLink node by address string.
///
/// The address must be in one of the following formats:
///
///  * `tcp:<addr>:<port>`
///  * `udpin:<addr>:<port>`
///  * `udpout:<addr>:<port>`
///
/// The type of the connection is determined at runtime based on the address type, so the
/// connection is returned as a trait object.
pub fn connect(address: &str) -> io::Result<Box<MavConnection + Sync + Send>> {
    if address.starts_with("tcp:") {
        Ok(Box::new(try!(Tcp::tcp(&address["tcp:".len()..]))))
    } else if address.starts_with("udpin:") {
        Ok(Box::new(try!(Udp::udpin(&address["udpin:".len()..]))))
    } else if address.starts_with("udpout:") {
        Ok(Box::new(try!(Udp::udpout(&address["udpout:".len()..]))))
    } else {
        Err(io::Error::new(io::ErrorKind::AddrNotAvailable, "Prefix must be one of udpin, udpout, or tcp"))
    }
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
        PacketBuf { buf: v, start: 0, end: 0 }
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
        let n = try!(Read::read(&mut self.slice(), buf));
        self.start += n;
        Ok(n)
    }
}

struct UdpRead {
    socket: UdpSocket,
    recv_buf: PacketBuf,
}

/// UDP MAVLink connection
pub struct Udp {
    read: Mutex<UdpRead>,
    write: Mutex<UdpWrite>,
    server: bool,
}

impl Udp {
    fn new(socket: UdpSocket, server: bool, dest: Option<SocketAddr>) -> io::Result<Udp> {
        Ok(Udp {
            server: server,
            read: Mutex::new(UdpRead {
                socket: try!(socket.try_clone()),
                recv_buf: PacketBuf::new(),
            }),
            write: Mutex::new(UdpWrite {
                socket: socket,
                dest: dest,
                sequence: 0,
            }),
        })
    }

    pub fn udpin<T: ToSocketAddrs>(address: T) -> io::Result<Udp> {
        let addr = address.to_socket_addrs().unwrap().next().unwrap();
        let socket = try!(UdpSocket::bind(&addr));
        Udp::new(socket, true, None)
    }

    pub fn udpout<T: ToSocketAddrs>(address: T) -> io::Result<Udp> {
        let addr = address.to_socket_addrs().unwrap().next().unwrap();
        let socket = try!(UdpSocket::bind(&SocketAddr::from_str("0.0.0.0:0").unwrap()));
        Udp::new(socket, false, Some(addr))
    }
}

impl MavConnection for Udp {
    fn recv(&self) -> io::Result<MavMessage> {
        let mut guard = self.read.lock().unwrap();
        let state = &mut *guard;
        loop {
            if state.recv_buf.len() == 0 {
                let (len, src) = try!(state.socket.recv_from(state.recv_buf.reset()));
                state.recv_buf.set_len(len);

                if self.server {
                    self.write.lock().unwrap().dest = Some(src);
                }
            }

            if let Ok((_, m)) = read(&mut state.recv_buf) {
                return Ok(m);
            }
        }
    }

    fn send(&self, data: &MavMessage) -> io::Result<()> {
        let mut guard = self.write.lock().unwrap();
        let state = &mut *guard;

        let header = Header {
            sequence: state.sequence,
            system_id: 255,
            component_id: 0,
        };

        state.sequence = state.sequence.wrapping_add(1);

        if let Some(addr) = state.dest {
            let mut buf = Vec::new();
            try!(write(&mut buf, header, data));
            try!(state.socket.send_to(&buf, addr));
        }

        Ok(())
    }
}

/// TCP MAVLink connection
pub struct Tcp {
    read: Mutex<TcpStream>,
    write: Mutex<TcpWrite>,
}

struct TcpWrite {
    socket: TcpStream,
    sequence: u8,
}

impl Tcp {
    pub fn tcp<T: ToSocketAddrs>(address: T) -> io::Result<Tcp> {
        let addr = address.to_socket_addrs().unwrap().next().unwrap();
        let socket = try!(TcpStream::connect(&addr));
        Ok(Tcp {
            read: Mutex::new(try!(socket.try_clone())),
            write: Mutex::new(TcpWrite { socket: socket, sequence: 0 }),
        })
    }
}

impl MavConnection for Tcp {
    fn recv(&self) -> io::Result<MavMessage> {
        let mut lock = self.read.lock().unwrap();
        read(&mut *lock).map(|(_, pkt)| pkt)
    }

    fn send(&self, data: &MavMessage) -> io::Result<()> {
        let mut lock = self.write.lock().unwrap();

        let header = Header {
            sequence: lock.sequence,
            system_id: 255,
            component_id: 0,
        };

        lock.sequence = lock.sequence.wrapping_add(1);

        try!(write(&mut lock.socket, header, data));

        Ok(())
    }
}
