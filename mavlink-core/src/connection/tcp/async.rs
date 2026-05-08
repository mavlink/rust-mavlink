//! Async TCP MAVLink connection

use std::io;

use crate::async_peek_reader::AsyncPeekReader;
use crate::connection::tcp::config::{TcpConfig, TcpMode};
use crate::connection::{AsyncConnectable, AsyncMavConnection, get_socket_addr};
use crate::connection_shared::{
    ConnectionState, next_send_header, read_message_async, read_raw_message_async,
    write_message_async,
};
use crate::{MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message};

use async_trait::async_trait;
use core::ops::DerefMut;
use futures::{FutureExt, lock::Mutex};
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpListener, TcpStream};

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

pub async fn tcpout<T: std::net::ToSocketAddrs>(address: T) -> io::Result<AsyncTcpConnection> {
    let addr = get_socket_addr(&address)?;

    let socket = TcpStream::connect(addr).await?;

    let (reader, writer) = socket.into_split();

    Ok(AsyncTcpConnection {
        reader: Mutex::new(AsyncPeekReader::new(reader)),
        writer: Mutex::new(TcpWrite {
            socket: writer,
            sequence: 0,
        }),
        state: ConnectionState::new(),
    })
}

pub async fn tcpin<T: std::net::ToSocketAddrs>(address: T) -> io::Result<AsyncTcpConnection> {
    let addr = get_socket_addr(&address)?;
    let listener = TcpListener::bind(addr).await?;

    //For now we only accept one incoming stream: this yields until we get one
    match listener.accept().await {
        Ok((socket, _)) => {
            let (reader, writer) = socket.into_split();
            return Ok(AsyncTcpConnection {
                reader: Mutex::new(AsyncPeekReader::new(reader)),
                writer: Mutex::new(TcpWrite {
                    socket: writer,
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
    Err(io::Error::new(
        io::ErrorKind::NotConnected,
        "No incoming connections!",
    ))
}

pub struct AsyncTcpConnection {
    reader: Mutex<AsyncPeekReader<OwnedReadHalf>>,
    writer: Mutex<TcpWrite>,
    state: ConnectionState,
}

struct TcpWrite {
    socket: OwnedWriteHalf,
    sequence: u8,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncTcpConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        read_message_async::<M, _>(reader.deref_mut(), &self.state).await
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        read_raw_message_async::<M, _>(reader.deref_mut(), &self.state).await
    }

    async fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        match self.recv().now_or_never() {
            Some(result) => result,
            None => Err(crate::error::MessageReadError::Io(
                io::ErrorKind::WouldBlock.into(),
            )),
        }
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> Result<usize, crate::error::MessageWriteError> {
        let mut lock = self.writer.lock().await;

        let header = next_send_header(&mut lock.sequence, header);
        write_message_async(&mut lock.socket, &self.state, header, data).await
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

#[async_trait]
impl AsyncConnectable for TcpConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        let conn = match self.mode {
            TcpMode::TcpIn => tcpin(&self.address).await,
            TcpMode::TcpOut => tcpout(&self.address).await,
        };

        Ok(Box::new(conn?))
    }
}
