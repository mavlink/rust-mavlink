//! Async UDP MAVLink connection

use core::task::Poll;
use std::io;
use std::{
    collections::VecDeque,
    io::Read,
    sync::{Arc, Mutex as StdMutex},
};

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio::{
    io::{AsyncRead, AsyncWrite, ReadBuf},
    net::UdpSocket,
};

use crate::connection::r#async::{
    AsyncConnectable, AsyncConnectionCore, AsyncDialectConnectable, AsyncDialectConnection,
    AsyncMavConnection, AsyncTransport,
};
use crate::connection::get_socket_addr;
use crate::connection::udp::config::{UdpConfig, UdpMode};
use crate::connection_shared::next_send_header;
use crate::{Dialect, MavHeader, async_peek_reader::AsyncPeekReader};

pub(crate) struct UdpRead {
    socket: Arc<UdpSocket>,
    buffer: VecDeque<u8>,
    reply_destination: Option<Arc<StdMutex<Option<std::net::SocketAddr>>>>,
}

const MTU_SIZE: usize = 1500;

impl AsyncRead for UdpRead {
    fn poll_read(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        if self.buffer.is_empty() {
            let mut read_buffer = [0u8; MTU_SIZE];
            let mut read_buffer = ReadBuf::new(&mut read_buffer);

            match self.socket.poll_recv_from(cx, &mut read_buffer) {
                Poll::Ready(Ok(address)) => {
                    let n_buffer = read_buffer.filled().len();

                    let n = (&read_buffer.filled()[0..n_buffer]).read(buf.initialize_unfilled())?;
                    buf.advance(n);

                    self.buffer.extend(&read_buffer.filled()[n..n_buffer]);
                    if let Some(reply_destination) = &self.reply_destination {
                        *reply_destination.lock().unwrap() = Some(address);
                    }
                    Poll::Ready(Ok(()))
                }
                Poll::Ready(Err(err)) => Poll::Ready(Err(err)),
                Poll::Pending => Poll::Pending,
            }
        } else {
            let read_result = self.buffer.read(buf.initialize_unfilled());
            let result = match read_result {
                Ok(n) => {
                    buf.advance(n);
                    Ok(())
                }
                Err(err) => Err(err),
            };
            Poll::Ready(result)
        }
    }
}

pub(crate) struct UdpWrite {
    socket: Arc<UdpSocket>,
    destination: Arc<StdMutex<Option<std::net::SocketAddr>>>,
    sequence: u8,
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

        match this.socket.poll_send_to(cx, buf, addr) {
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
    reader: Mutex<AsyncPeekReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
}

impl AsyncUdpConnection {
    fn new(
        socket: UdpSocket,
        server: bool,
        dest: Option<std::net::SocketAddr>,
    ) -> io::Result<Self> {
        let socket = Arc::new(socket);
        let destination = Arc::new(StdMutex::new(dest));
        Ok(Self {
            reader: Mutex::new(AsyncPeekReader::new(UdpRead {
                socket: socket.clone(),
                buffer: VecDeque::new(),
                reply_destination: server.then(|| destination.clone()),
            })),
            writer: Mutex::new(UdpWrite {
                socket,
                destination,
                sequence: 0,
            }),
        })
    }
}

impl AsyncTransport for AsyncUdpConnection {
    type Reader = UdpRead;
    type Writer = UdpWrite;

    fn reader(&self) -> &Mutex<AsyncPeekReader<Self::Reader>> {
        &self.reader
    }

    fn writer(&self) -> Option<&Mutex<Self::Writer>> {
        Some(&self.writer)
    }

    fn retry_receive(&self, _: &crate::error::MessageReadError) -> bool {
        true
    }

    fn can_send(&self, writer: &Self::Writer) -> bool {
        writer.destination.lock().unwrap().is_some()
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        next_send_header(&mut writer.sequence, header)
    }
}

impl UdpConfig {
    pub(crate) async fn open_async(&self) -> io::Result<AsyncUdpConnection> {
        let (addr, server, dest): (&str, _, _) = match self.mode {
            UdpMode::Udpin => (&self.address, true, None),
            _ => ("0.0.0.0:0", false, Some(get_socket_addr(&self.address)?)),
        };
        let socket = UdpSocket::bind(addr).await?;
        if matches!(self.mode, UdpMode::UdpBroadcast) {
            socket.set_broadcast(true)?;
        }
        AsyncUdpConnection::new(socket, server, dest)
    }
}

#[async_trait]
impl AsyncConnectable for UdpConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: crate::Message + Sync + Send,
    {
        Ok(Box::new(AsyncConnectionCore::new_static(
            self.open_async().await?,
        )))
    }
}

#[async_trait]
impl AsyncDialectConnectable for UdpConfig {
    async fn connect_async_with_dialect<D>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D: Dialect + Send + Sync + 'static,
        D::Message: Send + Sync,
    {
        Ok(Box::new(AsyncConnectionCore::new(
            self.open_async().await?,
            dialect,
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::AsyncReadExt;

    #[tokio::test]
    async fn test_datagram_buffering() {
        let receiver_socket = Arc::new(UdpSocket::bind("127.0.0.1:5001").await.unwrap());
        let mut udp_reader = UdpRead {
            socket: receiver_socket.clone(),
            buffer: VecDeque::new(),
            reply_destination: None,
        };
        let sender_socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
        sender_socket.connect("127.0.0.1:5001").await.unwrap();

        let datagram: Vec<u8> = (0..50).collect::<Vec<_>>();

        let mut n_sent = sender_socket.send(&datagram).await.unwrap();
        assert_eq!(n_sent, datagram.len());
        n_sent = sender_socket.send(&datagram).await.unwrap();
        assert_eq!(n_sent, datagram.len());

        let mut buf = [0u8; 30];

        let mut n_read = udp_reader.read(&mut buf).await.unwrap();
        assert_eq!(n_read, 30);
        assert_eq!(&buf[0..n_read], (0..30).collect::<Vec<_>>().as_slice());

        n_read = udp_reader.read(&mut buf).await.unwrap();
        assert_eq!(n_read, 20);
        assert_eq!(&buf[0..n_read], (30..50).collect::<Vec<_>>().as_slice());

        n_read = udp_reader.read(&mut buf).await.unwrap();
        assert_eq!(n_read, 30);
        assert_eq!(&buf[0..n_read], (0..30).collect::<Vec<_>>().as_slice());

        n_read = udp_reader.read(&mut buf).await.unwrap();
        assert_eq!(n_read, 20);
        assert_eq!(&buf[0..n_read], (30..50).collect::<Vec<_>>().as_slice());
    }
}
