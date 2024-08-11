//! TCP MAVLink connection

use crate::connection::MavConnection;
use crate::peek_reader::PeekReader;
use crate::{MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::io;
use std::net::ToSocketAddrs;
use std::net::{TcpListener, TcpStream};
use std::sync::Mutex;
use std::time::Duration;

use super::get_socket_addr;

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg, write_versioned_msg};

#[cfg(feature = "signing")]
use crate::{read_versioned_msg_signed, write_versioned_msg_signed, SigningConfig, SigningData};

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
    let addr = get_socket_addr(address)?;

    let socket = TcpStream::connect(addr)?;
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;

    Ok(TcpConnection {
        reader: Mutex::new(PeekReader::new(socket.try_clone()?)),
        writer: Mutex::new(TcpWrite {
            socket,
            sequence: 0,
        }),
        protocol_version: MavlinkVersion::V2,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub fn tcpin<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = get_socket_addr(address)?;
    let listener = TcpListener::bind(addr)?;

    //For now we only accept one incoming stream: this blocks until we get one
    for incoming in listener.incoming() {
        match incoming {
            Ok(socket) => {
                return Ok(TcpConnection {
                    reader: Mutex::new(PeekReader::new(socket.try_clone()?)),
                    writer: Mutex::new(TcpWrite {
                        socket,
                        sequence: 0,
                    }),
                    protocol_version: MavlinkVersion::V2,
                    #[cfg(feature = "signing")]
                    signing_data: None,
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
    reader: Mutex<PeekReader<TcpStream>>,
    writer: Mutex<TcpWrite>,
    protocol_version: MavlinkVersion,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

struct TcpWrite {
    socket: TcpStream,
    sequence: u8,
}

impl<M: Message> MavConnection<M> for TcpConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();
        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg(reader.deref_mut(), self.protocol_version);
        #[cfg(feature = "signing")]
        let result = read_versioned_msg_signed(
            reader.deref_mut(),
            self.protocol_version,
            self.signing_data.as_ref(),
        );
        result
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let mut lock = self.writer.lock().unwrap();

        let header = MavHeader {
            sequence: lock.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        lock.sequence = lock.sequence.wrapping_add(1);
        #[cfg(not(feature = "signing"))]
        let result = write_versioned_msg(&mut lock.socket, self.protocol_version, header, data);
        #[cfg(feature = "signing")]
        let result = write_versioned_msg_signed(
            &mut lock.socket,
            self.protocol_version,
            header,
            data,
            self.signing_data.as_ref(),
        );
        result
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
