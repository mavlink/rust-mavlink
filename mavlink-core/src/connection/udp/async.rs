//! Async UDP MAVLink connection

use core::task::Poll;
use std::io;
use std::sync::{Arc, Mutex as StdMutex};

use futures::lock::{Mutex, MutexGuard};
use tokio::{
    io::{AsyncRead, AsyncWrite, ReadBuf},
    net::UdpSocket,
};

use crate::connection::r#async::{
    AsyncConnectable, AsyncDialectConnection, AsyncTransport, Transport,
};
use crate::connection::get_socket_addr;
use crate::connection::udp::config::{UdpConfig, UdpMode};
use crate::{AsyncMavlinkReader, Dialect};

pub(crate) struct UdpRead {
    socket: Arc<UdpSocket>,
    reply_destination: Option<Arc<StdMutex<Option<std::net::SocketAddr>>>>,
}

impl AsyncRead for UdpRead {
    fn poll_read(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        match self.socket.poll_recv_from(cx, buf) {
            Poll::Ready(Ok(address)) => {
                if let Some(reply_destination) = &self.reply_destination {
                    *reply_destination.lock().unwrap() = Some(address);
                }
                Poll::Ready(Ok(()))
            }
            Poll::Ready(Err(err)) => Poll::Ready(Err(err)),
            Poll::Pending => Poll::Pending,
        }
    }
}

pub(crate) struct UdpWrite {
    socket: Arc<UdpSocket>,
    destination: Arc<StdMutex<Option<std::net::SocketAddr>>>,
    connected: bool,
}

impl AsyncWrite for UdpWrite {
    fn poll_write(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        let this = self.get_mut();
        let addr = this
            .destination
            .lock()
            .unwrap()
            .expect("`dest` is checked before write");

        let result = if this.connected {
            this.socket.poll_send(cx, buf)
        } else {
            this.socket.poll_send_to(cx, buf, addr)
        };
        match result {
            Poll::Ready(Ok(written)) if written == buf.len() => Poll::Ready(Ok(written)),
            Poll::Ready(Ok(_)) => Poll::Ready(Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "failed to send complete UDP datagram",
            ))),
            Poll::Ready(Err(err)) => Poll::Ready(Err(err)),
            Poll::Pending => Poll::Pending,
        }
    }

    fn poll_flush(
        self: core::pin::Pin<&mut Self>,
        _cx: &mut core::task::Context<'_>,
    ) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(
        self: core::pin::Pin<&mut Self>,
        _cx: &mut core::task::Context<'_>,
    ) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }
}

pub struct AsyncUdpConnection {
    reader: Mutex<AsyncMavlinkReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
}

impl AsyncUdpConnection {
    fn new(
        socket: UdpSocket,
        server: bool,
        dest: Option<std::net::SocketAddr>,
        connected: bool,
    ) -> io::Result<Self> {
        let socket = Arc::new(socket);
        let destination = Arc::new(StdMutex::new(dest));
        Ok(Self {
            reader: Mutex::new(AsyncMavlinkReader::new(UdpRead {
                socket: socket.clone(),
                reply_destination: server.then(|| destination.clone()),
            })),
            writer: Mutex::new(UdpWrite {
                socket,
                destination,
                connected,
            }),
        })
    }
}

impl AsyncTransport for AsyncUdpConnection {
    type Reader = UdpRead;
    type Writer = UdpWrite;

    fn reader(&self) -> &Mutex<AsyncMavlinkReader<Self::Reader>> {
        &self.reader
    }

    async fn lock_writer(&self) -> Option<MutexGuard<'_, Self::Writer>> {
        let writer = self.writer.lock().await;
        let has_destination = writer.destination.lock().unwrap().is_some();
        has_destination.then_some(writer)
    }

    fn retry_receive(&self, _: &crate::error::MessageReadError) -> bool {
        true
    }
}

impl UdpConfig {
    pub(crate) async fn open_async(&self) -> io::Result<AsyncUdpConnection> {
        let (addr, server, dest): (&str, _, _) = match self.mode {
            UdpMode::Udpin => (&self.address, true, None),
            _ => ("0.0.0.0:0", false, Some(get_socket_addr(&self.address)?)),
        };
        let (socket, connected) = match self.take_socket()? {
            Some(socket) => {
                socket.set_nonblocking(true)?;
                (UdpSocket::from_std(socket)?, !server)
            }
            None => (UdpSocket::bind(addr).await?, false),
        };
        if matches!(self.mode, UdpMode::UdpBroadcast) {
            socket.set_broadcast(true)?;
        }
        AsyncUdpConnection::new(socket, server, dest, connected)
    }
}

impl AsyncConnectable for UdpConfig {
    async fn connect_async_with_dialect<D>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D: Dialect + Send + Sync,
        D::Message: Send + Sync,
    {
        Ok(AsyncDialectConnection::new(
            Transport::Udp(self.open_async().await?),
            dialect,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::AsyncReadExt;

    #[tokio::test]
    async fn test_datagram_reading() {
        let receiver_socket = Arc::new(UdpSocket::bind("127.0.0.1:5001").await.unwrap());
        let mut udp_reader = UdpRead {
            socket: receiver_socket.clone(),
            reply_destination: None,
        };
        let sender_socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
        sender_socket.connect("127.0.0.1:5001").await.unwrap();

        let datagram: Vec<u8> = (0..50).collect::<Vec<_>>();

        let mut n_sent = sender_socket.send(&datagram).await.unwrap();
        assert_eq!(n_sent, datagram.len());
        n_sent = sender_socket.send(&datagram).await.unwrap();
        assert_eq!(n_sent, datagram.len());

        let mut buf = [0u8; 50];

        let mut n_read = udp_reader.read(&mut buf).await.unwrap();
        assert_eq!(n_read, 50);
        assert_eq!(&buf[0..n_read], datagram.as_slice());

        n_read = udp_reader.read(&mut buf).await.unwrap();
        assert_eq!(n_read, 50);
        assert_eq!(&buf[0..n_read], datagram.as_slice());
    }
}
