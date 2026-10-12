//! Async TCP MAVLink connection

use std::io;

use async_trait::async_trait;
use futures::lock::{Mutex, MutexGuard};
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpListener, TcpStream};

use crate::AsyncMavlinkReader;
use crate::Dialect;
use crate::connection::r#async::{
    AsyncConnectable, AsyncConnectionCore, AsyncDialectConnectable, AsyncDialectConnection,
    AsyncMavConnection, AsyncTransport,
};
use crate::connection::get_socket_addr;
use crate::connection::tcp::config::{TcpConfig, TcpMode};

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
    reader: Mutex<AsyncMavlinkReader<OwnedReadHalf>>,
    writer: Mutex<OwnedWriteHalf>,
}

impl AsyncTcpConnection {
    fn from_stream(socket: TcpStream) -> Self {
        let (reader, writer) = socket.into_split();
        Self {
            reader: Mutex::new(AsyncMavlinkReader::new(reader)),
            writer: Mutex::new(writer),
        }
    }
}

impl AsyncTransport for AsyncTcpConnection {
    type Reader = OwnedReadHalf;
    type Writer = OwnedWriteHalf;

    fn reader(&self) -> &Mutex<AsyncMavlinkReader<Self::Reader>> {
        &self.reader
    }

    async fn lock_writer(&self) -> Option<MutexGuard<'_, Self::Writer>> {
        Some(self.writer.lock().await)
    }
}

impl TcpConfig {
    pub(crate) async fn open_async(&self) -> io::Result<AsyncTcpConnection> {
        match self.mode {
            TcpMode::TcpIn => match self.take_listener()? {
                Some(listener) => {
                    listener.set_nonblocking(true)?;
                    let (socket, _) = TcpListener::from_std(listener)?.accept().await?;
                    Ok(AsyncTcpConnection::from_stream(socket))
                }
                None => tcpin(&self.address).await,
            },
            TcpMode::TcpOut => match self.take_stream()? {
                Some(stream) => {
                    stream.set_nonblocking(true)?;
                    Ok(AsyncTcpConnection::from_stream(TcpStream::from_std(
                        stream,
                    )?))
                }
                None => tcpout(&self.address).await,
            },
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
