//! TCP MAVLink connection

use crate::Dialect;
use crate::MavlinkReader;
use crate::connection::get_socket_addr;
use crate::connection::sync::{Connectable, DialectConnection, SyncTransport, Transport};
use crate::error::MessageReadError;
use std::io;
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::sync::Mutex;
use std::time::Duration;

use super::config::{TcpConfig, TcpMode};

pub fn tcpout<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let socket = TcpStream::connect(get_socket_addr(&address)?)?;
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;
    connection_from_stream(socket)
}

fn connection_from_stream(socket: TcpStream) -> io::Result<TcpConnection> {
    Ok(TcpConnection {
        reader: Mutex::new(MavlinkReader::new(socket.try_clone()?)),
        writer: Mutex::new(socket),
    })
}

pub fn tcpin<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let listener = TcpListener::bind(get_socket_addr(&address)?)?;
    accept(listener)
}

fn accept(listener: TcpListener) -> io::Result<TcpConnection> {
    for incoming in listener.incoming() {
        match incoming {
            Ok(socket) => return connection_from_stream(socket),
            Err(e) => println!("listener err: {e}"),
        }
    }
    Err(io::Error::new(
        io::ErrorKind::NotConnected,
        "No incoming connections!",
    ))
}

pub struct TcpConnection {
    reader: Mutex<MavlinkReader<TcpStream>>,
    writer: Mutex<TcpStream>,
}

impl SyncTransport for TcpConnection {
    type Reader = TcpStream;
    type Writer = TcpStream;

    fn reader(&self) -> std::sync::MutexGuard<'_, MavlinkReader<Self::Reader>> {
        self.reader.lock().unwrap()
    }

    fn lock_writer(&self) -> Option<std::sync::MutexGuard<'_, Self::Writer>> {
        Some(self.writer.lock().unwrap())
    }

    fn try_read<T>(
        &self,
        read: impl FnOnce(&mut MavlinkReader<Self::Reader>) -> Result<T, MessageReadError>,
    ) -> Result<T, MessageReadError> {
        let mut reader = self.reader();
        // The reader and writer are clones of one socket and share its nonblocking
        // flag, so it is only switched through the writer, while no send can run.
        // Otherwise a send could fail with `WouldBlock` partway through a frame.
        let writer = self.writer.lock().unwrap();
        writer.set_nonblocking(true)?;
        let result = read(&mut reader);
        writer.set_nonblocking(false)?;
        result
    }
}

impl TcpConfig {
    pub(crate) fn open(&self) -> io::Result<TcpConnection> {
        match self.mode {
            TcpMode::TcpIn => match self.take_listener()? {
                Some(listener) => accept(listener),
                None => tcpin(&self.address),
            },
            TcpMode::TcpOut => match self.take_stream()? {
                Some(stream) => connection_from_stream(stream),
                None => tcpout(&self.address),
            },
        }
    }
}

impl Connectable for TcpConfig {
    fn connect_with_dialect<D: Dialect + Send + Sync>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        Ok(DialectConnection::new(
            Transport::Tcp(self.open()?),
            dialect,
        ))
    }
}
