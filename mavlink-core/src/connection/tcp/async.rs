//! Async TCP MAVLink connection

use core::pin::Pin;
use core::task::{Context, Poll};
use std::io;

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio::io::AsyncWrite;
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpListener, TcpStream};

use crate::async_peek_reader::AsyncPeekReader;
use crate::connection::r#async::{
    AsyncConnectable, AsyncConnectionCore, AsyncDialectConnectable, AsyncDialectConnection,
    AsyncMavConnection, AsyncTransport,
};
use crate::connection::get_socket_addr;
use crate::connection::tcp::config::{TcpConfig, TcpMode};
use crate::connection_shared::next_send_header;
use crate::{Dialect, MavHeader};

pub async fn tcpout<T: std::net::ToSocketAddrs>(address: T) -> io::Result<AsyncTcpConnection> {
    let socket = TcpStream::connect(get_socket_addr(&address)?).await?;
    Ok(AsyncTcpConnection::from_stream(socket))
}

pub async fn tcpin<T: std::net::ToSocketAddrs>(address: T) -> io::Result<AsyncTcpConnection> {
    let listener = TcpListener::bind(get_socket_addr(&address)?).await?;
    //For now we only accept one incoming stream: this yields until we get one
    let (socket, _) = listener.accept().await?;
    Ok(AsyncTcpConnection::from_stream(socket))
}

pub struct AsyncTcpConnection {
    reader: Mutex<AsyncPeekReader<OwnedReadHalf>>,
    writer: Mutex<TcpWrite>,
}

impl AsyncTcpConnection {
    fn from_stream(socket: TcpStream) -> Self {
        let (reader, writer) = socket.into_split();
        Self {
            reader: Mutex::new(AsyncPeekReader::new(reader)),
            writer: Mutex::new(TcpWrite {
                socket: writer,
                sequence: 0,
            }),
        }
    }
}

pub(crate) struct TcpWrite {
    socket: OwnedWriteHalf,
    sequence: u8,
}

impl AsyncWrite for TcpWrite {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        Pin::new(&mut self.socket).poll_write(cx, buf)
    }

    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Pin::new(&mut self.socket).poll_flush(cx)
    }

    fn poll_shutdown(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Pin::new(&mut self.socket).poll_shutdown(cx)
    }
}

impl AsyncTransport for AsyncTcpConnection {
    type Reader = OwnedReadHalf;
    type Writer = TcpWrite;

    fn reader(&self) -> &Mutex<AsyncPeekReader<Self::Reader>> {
        &self.reader
    }

    fn writer(&self) -> Option<&Mutex<Self::Writer>> {
        Some(&self.writer)
    }

    fn try_recv_is_nonblocking(&self) -> bool {
        true
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        next_send_header(&mut writer.sequence, header)
    }
}

impl TcpConfig {
    pub(crate) async fn open_async(&self) -> io::Result<AsyncTcpConnection> {
        match self.mode {
            TcpMode::TcpIn => tcpin(&self.address).await,
            TcpMode::TcpOut => tcpout(&self.address).await,
        }
    }
}

#[async_trait]
impl AsyncConnectable for TcpConfig {
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
impl AsyncDialectConnectable for TcpConfig {
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
