//! UDP MAVLink connection

use super::config::{UdpConfig, UdpMode};

use crate::MavlinkReader;
use crate::connection::get_socket_addr;
use crate::connection::sync::{
    Connectable, Connection, ConnectionCore, DialectConnection, SyncTransport,
};
use crate::connection_shared::next_send_header;
use crate::{Dialect, MavHeader};
use std::io::{self, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};

pub(crate) struct UdpRead {
    socket: UdpSocket,
    reply_destination: Option<Arc<Mutex<Option<SocketAddr>>>>,
}

impl Read for UdpRead {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let (len, address) = self.socket.recv_from(buf)?;
        if let Some(reply_destination) = &self.reply_destination {
            *reply_destination.lock().unwrap() = Some(address);
        }
        Ok(len)
    }
}

pub(crate) struct UdpWrite {
    socket: UdpSocket,
    destination: Arc<Mutex<Option<SocketAddr>>>,
    connected: bool,
    sequence: u8,
}

impl Write for UdpWrite {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.connected {
            return self.socket.send(buf);
        }
        self.socket.send_to(
            buf,
            self.destination
                .lock()
                .unwrap()
                .expect("destination checked before write"),
        )
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        if self.write(buf)? == buf.len() {
            Ok(())
        } else {
            Err(io::ErrorKind::WriteZero.into())
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

pub struct UdpConnection {
    reader: Mutex<MavlinkReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
}

impl UdpConnection {
    fn new(
        socket: UdpSocket,
        server: bool,
        dest: Option<SocketAddr>,
        connected: bool,
    ) -> io::Result<Self> {
        let destination = Arc::new(Mutex::new(dest));
        Ok(Self {
            reader: Mutex::new(MavlinkReader::new(UdpRead {
                socket: socket.try_clone()?,
                reply_destination: server.then(|| destination.clone()),
            })),
            writer: Mutex::new(UdpWrite {
                socket,
                destination,
                connected,
                sequence: 0,
            }),
        })
    }
}

impl SyncTransport for UdpConnection {
    type Reader = UdpRead;
    type Writer = UdpWrite;

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
        reader.get_mut().socket.set_nonblocking(enabled)
    }

    fn can_send(&self, writer: &Self::Writer) -> bool {
        writer.destination.lock().unwrap().is_some()
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        next_send_header(&mut writer.sequence, header)
    }
}

impl UdpConfig {
    pub(crate) fn open(&self) -> io::Result<UdpConnection> {
        let (address, server, dest): (&str, _, _) = match self.mode {
            UdpMode::Udpin => (&self.address, true, None),
            _ => ("0.0.0.0:0", false, Some(get_socket_addr(&self.address)?)),
        };
        let (socket, connected) = match self.take_socket()? {
            Some(socket) => (socket, !server),
            None => (UdpSocket::bind(address)?, false),
        };
        if let Some(timeout) = self.read_timeout {
            socket.set_read_timeout(Some(timeout))?;
        }
        if matches!(self.mode, UdpMode::UdpBroadcast) {
            socket.set_broadcast(true)?;
        }
        UdpConnection::new(socket, server, dest, connected)
    }
}

impl Connectable for UdpConfig {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_datagram_reading() {
        let receiver_socket = UdpSocket::bind("127.0.0.1:5000").unwrap();
        let mut udp_reader = UdpRead {
            socket: receiver_socket.try_clone().unwrap(),
            reply_destination: None,
        };
        let sender_socket = UdpSocket::bind("0.0.0.0:0").unwrap();
        sender_socket.connect("127.0.0.1:5000").unwrap();

        let datagram: Vec<u8> = (0..50).collect::<Vec<_>>();

        let mut n_sent = sender_socket.send(&datagram).unwrap();
        assert_eq!(n_sent, datagram.len());
        n_sent = sender_socket.send(&datagram).unwrap();
        assert_eq!(n_sent, datagram.len());

        let mut buf = [0u8; 50];

        let mut n_read = udp_reader.read(&mut buf).unwrap();
        assert_eq!(n_read, 50);
        assert_eq!(&buf[0..n_read], datagram.as_slice());

        n_read = udp_reader.read(&mut buf).unwrap();
        assert_eq!(n_read, 50);
        assert_eq!(&buf[0..n_read], datagram.as_slice());
    }
}
