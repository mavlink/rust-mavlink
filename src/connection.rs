use crate::common::MavMessage;
use crate::{read_msg, write_msg, MavHeader};
use crate::MavFrame;
use std::sync::Mutex;

#[cfg(feature = "udp")]
use std::net::{SocketAddr, UdpSocket};

use std::net::{TcpStream, ToSocketAddrs};
use std::io::{self};

#[cfg(feature = "udp")]
use std::io::Read;


#[cfg(feature = "udp")]
use std::str::FromStr;

use std::time::Duration;


/// A MAVLink connection
pub trait MavConnection {
    /// Receive a mavlink message.
    ///
    /// Blocks until a valid frame is received, ignoring invalid messages.
    fn recv(&self) -> io::Result<(MavHeader,MavMessage)>;

    /// Send a mavlink message
    fn send(&self, header: &MavHeader, data: &MavMessage) -> io::Result<()>;

    /// Write whole frame
    fn send_frame(&self, frame: &MavFrame) -> io::Result<()> {
        self.send(&frame.header, &frame.msg)
    }

    /// Read whole frame
    fn recv_frame(&self) -> io::Result<MavFrame> {
        let (header,msg) = self.recv()?;
        Ok(MavFrame{header,msg})
    }

    /// Send a message with default header
    fn send_default(&self, data: &MavMessage) -> io::Result<()> {
        let header = MavHeader::get_default_header();
        self.send(&header, data)
    }
}

/// Connect to a MAVLink node by address string.
///
/// The address must be in one of the following formats:
///
///  * `tcp:<addr>:<port>`
///  * `udpin:<addr>:<port>`
///  * `udpout:<addr>:<port>`
///  * `serial:<port>:<baudrate>`
///
/// The type of the connection is determined at runtime based on the address type, so the
/// connection is returned as a trait object.
pub fn connect(address: &str) -> io::Result<Box<MavConnection + Sync + Send>> {

    let protocol_err = Err(io::Error::new(
        io::ErrorKind::AddrNotAvailable,
        "Protocol unsupported",
    ));


    if address.starts_with("tcp:") {
        Ok(Box::new(Tcp::tcp(&address["tcp:".len()..])?))
    } else if address.starts_with("udpin:") {
        #[cfg(not(feature = "udp"))] {
            protocol_err
        }
        #[cfg(feature = "udp")] {
            Ok(Box::new(Udp::udpin(&address["udpin:".len()..])?))
        }
    } else if address.starts_with("udpout:") {
        #[cfg(not(feature = "udp"))] {
            protocol_err
        }
        #[cfg(feature = "udp")] {
            Ok(Box::new(Udp::udpout(&address["udpout:".len()..])?))
        }
    } else if address.starts_with("serial:") {
        #[cfg(not(feature = "serial"))] {
            protocol_err
        }
        #[cfg(feature = "serial")] {
            Ok(Box::new(Serial::open(&address["serial:".len()..])?))
        }
    } else {
        protocol_err
    }
}

#[cfg(feature = "udp")]
struct UdpWrite {
    socket: UdpSocket,
    dest: Option<SocketAddr>,
    sequence: u8,
}

#[cfg(feature = "udp")]
struct PacketBuf {
    buf: Vec<u8>,
    start: usize,
    end: usize,
}

#[cfg(feature = "udp")]
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

#[cfg(feature = "udp")]
impl Read for PacketBuf {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = Read::read(&mut self.slice(), buf)?;
        self.start += n;
        Ok(n)
    }
}

#[cfg(feature = "udp")]
struct UdpRead {
    socket: UdpSocket,
    recv_buf: PacketBuf,
}

/// UDP MAVLink connection
#[cfg(feature = "udp")]
pub struct Udp {
    reader: Mutex<UdpRead>,
    writer: Mutex<UdpWrite>,
    server: bool,
}

#[cfg(feature = "udp")]
impl Udp {
    fn new(socket: UdpSocket, server: bool, dest: Option<SocketAddr>) -> io::Result<Udp> {
        Ok(Udp {
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

    pub fn udpin<T: ToSocketAddrs>(address: T) -> io::Result<Udp> {
        let addr = address.to_socket_addrs().unwrap().next().unwrap();
        let socket = UdpSocket::bind(&addr)?;
        Udp::new(socket, true, None)
    }

    pub fn udpout<T: ToSocketAddrs>(address: T) -> io::Result<Udp> {
        let addr = address.to_socket_addrs().unwrap().next().unwrap();
        let socket = UdpSocket::bind(&SocketAddr::from_str("0.0.0.0:0").unwrap())?;
        Udp::new(socket, false, Some(addr))
    }
}

#[cfg(feature = "udp")]
impl MavConnection for Udp {
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


/// TCP MAVLink connection
pub struct Tcp {
    reader: Mutex<TcpStream>,
    writer: Mutex<TcpWrite>,
}

struct TcpWrite {
    socket: TcpStream,
    sequence: u8,
}

impl Tcp {
    pub fn tcp<T: ToSocketAddrs>(address: T) -> io::Result<Tcp> {
        let addr = address.to_socket_addrs().unwrap().next().expect("Host address lookup failed.");
        let socket = TcpStream::connect(&addr)?;
        let sock_clone = socket.try_clone()?;
        sock_clone.set_read_timeout(Some(Duration::from_millis(100)))?;

        Ok(Tcp {
            reader: Mutex::new(socket.try_clone()?),
            writer: Mutex::new(TcpWrite {
                socket: socket,
                sequence: 0,
            }),
        })
    }
}

impl MavConnection for Tcp {
    fn recv(&self) -> io::Result<(MavHeader, MavMessage)> {

        loop {
            let mut lock = self.reader.lock().expect("tcp read failure");
            match read_msg(&mut *lock) {
                Ok( (header, msg) ) => {
                    println!("recvd {:?}", header);
                    return Ok((header, msg) );
                },
                Err(e) => {
                    match e.kind() {
                        io::ErrorKind::WouldBlock => {
                            //println!("would have blocked");
                            continue;
                        },
                        _ => panic!("Got an error: {}", e),
                    }
                },
            }
        }

    }

    fn send(&self, header: &MavHeader, data: &MavMessage) -> io::Result<()> {
        let mut lock = self.writer.lock().unwrap();

        let header = MavHeader {
            sequence: lock.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        lock.sequence = lock.sequence.wrapping_add(1);

        write_msg(&mut lock.socket, header, data)?;

        Ok(())
    }
}

/// Serial MAVLINK connection

#[cfg(feature = "serial" )]
pub struct Serial {
    port: Mutex<::serial::SystemPort>,
    sequence: Mutex<u8>,
}

#[cfg(feature = "serial" )]
impl Serial {
    pub fn open(settings: &str) -> io::Result<Serial> {
        let settings: Vec<&str> = settings.split(":").collect();
        let port = settings[0];
        let baud = settings[1].parse::<usize>().unwrap();
        let mut port = ::serial::open(port)?;

        let baud = ::serial::core::BaudRate::from_speed(baud);
        let settings = ::serial::core::PortSettings {
            baud_rate: baud,
            char_size: ::serial::Bits8,
            parity: ::serial::ParityNone,
            stop_bits: ::serial::Stop1,
            flow_control: ::serial::FlowNone,
        };

        port.configure(&settings)?;

        Ok(Serial {
            port: Mutex::new(port),
            sequence: Mutex::new(0),
        })
    }
}

#[cfg(feature = "serial" )]
impl MavConnection for Serial {
    fn recv(&self) -> io::Result<(MavHeader, MavMessage)> {
        let mut port = self.port.lock().unwrap();

        loop {
            if let Ok((h, m)) = read_msg(&mut *port) {
                return Ok((h,m));
            }
        }
    }

    fn send(&self, header: &MavHeader, data: &MavMessage) -> io::Result<()> {
        let mut port = self.port.lock().unwrap();
        let mut sequence = self.sequence.lock().unwrap();

        let header = MavHeader {
            sequence: *sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        *sequence = sequence.wrapping_add(1);

        write_msg(&mut *port, header, data)?;
        Ok(())
    }
}
