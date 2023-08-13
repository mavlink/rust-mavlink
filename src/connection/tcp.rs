use crate::connection::MavConnection;
use crate::{read_versioned_msg, write_versioned_msg, MavHeader, MavlinkVersion, Message};
use std::io::{self};
use std::net::ToSocketAddrs;
use std::net::{TcpListener, TcpStream};
use std::sync::Mutex;
use std::time::Duration;

/// TCP MAVLink connection

pub fn select_protocol<M: Message>(
    address: &str,
) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
    let connection = if let Some(address) = address.strip_prefix("tcpout:") {
        tcpout(address)
    } else if let Some(address) = address.strip_prefix("tcpin:") {
        tcpin(address)
    } else {
        Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Protocol unsupported",
        ))
    };

    Ok(Box::new(connection?))
}

pub fn tcpout<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Host address lookup failed.");
    let socket = TcpStream::connect(addr)?;
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;

    Ok(TcpConnection {
        reader: Mutex::new(socket.try_clone()?),
        writer: Mutex::new(TcpWrite {
            socket,
            sequence: 0,
        }),
        protocol_version: MavlinkVersion::V2,
        id: addr.to_string(),
    })
}

pub fn tcpin<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Invalid address");
    let listener = TcpListener::bind(addr)?;

    //For now we only accept one incoming stream: this blocks until we get one
    for incoming in listener.incoming() {
        match incoming {
            Ok(socket) => {
                return Ok(TcpConnection {
                    reader: Mutex::new(socket.try_clone()?),
                    writer: Mutex::new(TcpWrite {
                        socket,
                        sequence: 0,
                    }),
                    protocol_version: MavlinkVersion::V2,
                    id: addr.to_string(),
                })
            }
            Err(e) => {
                //TODO don't println in lib
                println!("listener err: {e}");
            }
        }
    }
    Err(io::Error::new(
        io::ErrorKind::NotConnected,
        "No incoming connections!",
    ))
}

pub struct TcpConnection {
    pub(crate) reader: Mutex<TcpStream>,
    pub(crate) writer: Mutex<TcpWrite>,
    protocol_version: MavlinkVersion,
    pub(super) id: String,
}

pub struct TcpWrite {
    pub(crate) socket: TcpStream,
    pub(crate) sequence: u8,
}

impl<M: Message> MavConnection<M> for TcpConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut lock = self.reader.lock().expect("tcp read failure");
        read_versioned_msg(&mut *lock, self.protocol_version)
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let mut lock = self.writer.lock().unwrap();

        let header = MavHeader {
            sequence: lock.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        lock.sequence = lock.sequence.wrapping_add(1);
        write_versioned_msg(&mut lock.socket, self.protocol_version, header, data)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }
}
