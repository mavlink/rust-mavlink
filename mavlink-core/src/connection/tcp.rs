//! TCP MAVLink connection

use crate::connection::get_socket_addr;
use crate::connection::MavConnection;
use crate::peek_reader::PeekReader;
use crate::Connectable;
use crate::{MavHeader, MavlinkVersion, Message, ReadVersion};
use core::ops::DerefMut;
use std::io;
use std::net::ToSocketAddrs;
use std::net::{TcpListener, TcpStream};
use std::sync::Mutex;
use std::time::Duration;

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg, write_versioned_msg};

#[cfg(feature = "signing")]
use crate::{read_versioned_msg_signed, write_versioned_msg_signed, SigningConfig, SigningData};

pub mod config;

use config::{TcpConfig, TcpMode};

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
        protocol_version: MavlinkVersion::V2,
        recv_any_version: false,
        #[cfg(feature = "signing")]
        signing_data: None,
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
                    protocol_version: MavlinkVersion::V2,
                    recv_any_version: false,
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
    recv_any_version: bool,
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
        let version = ReadVersion::from_conn_cfg::<_, M>(self);
        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg(reader.deref_mut(), version);
        #[cfg(feature = "signing")]
        let result =
            read_versioned_msg_signed(reader.deref_mut(), version, self.signing_data.as_ref());
        result
    }

    fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        self.recv()
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

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.recv_any_version = allow
    }

    fn allow_recv_any_version(&self) -> bool {
        self.recv_any_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config)
    }
}

impl Connectable for TcpConfig {
    fn connect<M: Message>(&self) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
        let conn = match self.mode {
            TcpMode::TcpIn => tcpin(&self.address),
            TcpMode::TcpOut => tcpout(&self.address),
        };

        Ok(Box::new(conn?))
    }
}
