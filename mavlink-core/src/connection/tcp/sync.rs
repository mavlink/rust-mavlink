//! TCP MAVLink connection

use crate::MavlinkReader;
use crate::connection::get_socket_addr;
use crate::connection::sync::{
    Connectable, Connection, ConnectionCore, DialectConnection, SyncTransport,
};
use crate::connection_shared::next_send_header;
use crate::{Dialect, MavHeader};
use std::io::{self, Write};
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
        writer: Mutex::new(TcpWrite {
            socket,
            sequence: 0,
        }),
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
    writer: Mutex<TcpWrite>,
}

pub(crate) struct TcpWrite {
    socket: TcpStream,
    sequence: u8,
}

impl Write for TcpWrite {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.socket.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.socket.flush()
    }
}

impl SyncTransport for TcpConnection {
    type Reader = TcpStream;
    type Writer = TcpWrite;

    fn reader(&self) -> std::sync::MutexGuard<'_, MavlinkReader<Self::Reader>> {
        self.reader.lock().unwrap()
    }

    fn writer(&self) -> Option<std::sync::MutexGuard<'_, Self::Writer>> {
        Some(self.writer.lock().unwrap())
    }

    fn set_nonblocking(
        &self,
        reader: &mut MavlinkReader<Self::Reader>,
        enabled: bool,
    ) -> io::Result<()> {
        reader.get_mut().set_nonblocking(enabled)
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        let header = next_send_header(&mut writer.sequence, header);
        header
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
    fn connect<M: crate::Message>(&self) -> io::Result<Connection<M>> {
        Ok(Box::new(ConnectionCore::new_static(self.open()?)))
    }

    fn connect_with_dialect<D: Dialect + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        Ok(Box::new(ConnectionCore::new(self.open()?, dialect)))
    }
}
