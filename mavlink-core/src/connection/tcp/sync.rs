//! TCP MAVLink connection

use crate::Connectable;
use crate::MAVLinkMessageRaw;
use crate::connection::get_socket_addr;
use crate::connection::{Connection, MavConnection};
use crate::connection_shared::{
    ConnectionState, next_send_header, read_message, read_raw_message, write_message,
};
use crate::peek_reader::PeekReader;
use crate::{MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::io;
use std::net::ToSocketAddrs;
use std::net::{TcpListener, TcpStream};
use std::sync::Mutex;
use std::time::Duration;

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

use super::config::{TcpConfig, TcpMode};

pub fn tcpout<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = get_socket_addr(&address)?;

    let socket = TcpStream::connect(addr)?;
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;

    Ok(TcpConnection {
        reader: Mutex::new(PeekReader::new(socket.try_clone()?)),
        writer: Mutex::new(TcpWrite {
            socket,
            sequence: 0,
        }),
        state: ConnectionState::new(),
    })
}

pub fn tcpin<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let addr = get_socket_addr(&address)?;
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
                    state: ConnectionState::new(),
                });
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
    state: ConnectionState,
}

struct TcpWrite {
    socket: TcpStream,
    sequence: u8,
}

impl<M: Message> MavConnection<M> for TcpConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();
        read_message::<M, _>(reader.deref_mut(), &self.state)
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();
        read_raw_message::<M, _>(reader.deref_mut(), &self.state)
    }

    fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().unwrap();
        reader.reader_mut().set_nonblocking(true)?;

        let result = read_message::<M, _>(reader.deref_mut(), &self.state);

        reader.reader_mut().set_nonblocking(false)?;

        result
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let mut lock = self.writer.lock().unwrap();

        let header = next_send_header(&mut lock.sequence, header);
        write_message(&mut lock.socket, &self.state, header, data)
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

impl Connectable for TcpConfig {
    fn connect<M: Message>(&self) -> io::Result<Connection<M>> {
        let conn = match self.mode {
            TcpMode::TcpIn => tcpin(&self.address),
            TcpMode::TcpOut => tcpout(&self.address),
        };

        Ok(conn?.into())
    }
}
