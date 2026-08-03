//! TCP MAVLink connection

use crate::connection::get_socket_addr;
use crate::connection::sync::{
    Connectable, Connection, ConnectionCore, DialectConnection, SyncTransport,
};
use crate::connection_shared::next_send_header;
use crate::peek_reader::PeekReader;
use crate::{Dialect, MavHeader};
use std::io::{self, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::sync::Mutex;
use std::time::Duration;

use super::config::{TcpConfig, TcpMode};

pub fn tcpout<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let socket = TcpStream::connect(get_socket_addr(&address)?)?;
    socket.set_read_timeout(Some(Duration::from_millis(100)))?;
    Ok(TcpConnection {
        reader: Mutex::new(PeekReader::new(socket.try_clone()?)),
        writer: Mutex::new(TcpWrite {
            socket,
            sequence: 0,
        }),
    })
}

pub fn tcpin<T: ToSocketAddrs>(address: T) -> io::Result<TcpConnection> {
    let listener = TcpListener::bind(get_socket_addr(&address)?)?;
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
                });
            }
            Err(error) => println!("listener err: {error}"),
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

    fn reader(&self) -> std::sync::MutexGuard<'_, PeekReader<Self::Reader>> {
        self.reader.lock().unwrap()
    }

    fn writer(&self) -> Option<std::sync::MutexGuard<'_, Self::Writer>> {
        Some(self.writer.lock().unwrap())
    }

    fn set_nonblocking(
        &self,
        reader: &mut PeekReader<Self::Reader>,
        enabled: bool,
    ) -> io::Result<()> {
        reader.reader_mut().set_nonblocking(enabled)
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        next_send_header(&mut writer.sequence, header)
    }
}

impl TcpConfig {
    pub(crate) fn open(&self) -> io::Result<TcpConnection> {
        match self.mode {
            TcpMode::TcpIn => tcpin(&self.address),
            TcpMode::TcpOut => tcpout(&self.address),
        }
    }
}

impl Connectable for TcpConfig {
    fn connect<M: crate::Message>(&self) -> io::Result<Connection<M>> {
        Ok(Box::new(ConnectionCore::new_static(self.open()?)))
    }

    fn connect_with_dialect<D: Dialect + Send + Sync + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        Ok(Box::new(ConnectionCore::new(self.open()?, dialect)))
    }
}
