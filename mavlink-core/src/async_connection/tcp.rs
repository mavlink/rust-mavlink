//! Async TCP MAVLink connection

use std::io;

use super::{get_socket_addr, AsyncConnectable, AsyncMavConnection};
use crate::async_peek_reader::AsyncPeekReader;
use crate::connectable::TcpConnectable;
use crate::{MavHeader, MavlinkVersion, Message, ReadVersion};

use async_trait::async_trait;
use core::ops::DerefMut;
use futures::lock::Mutex;
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpListener, TcpStream};

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "signing")]
use crate::{
    read_versioned_msg_async_signed, write_versioned_msg_async_signed, SigningConfig, SigningData,
};

pub async fn tcpout<T: std::net::ToSocketAddrs>(address: T) -> io::Result<AsyncTcpConnection> {
    let addr = get_socket_addr(address)?;

    let socket = TcpStream::connect(addr).await?;

    let (reader, writer) = socket.into_split();

    Ok(AsyncTcpConnection {
        reader: Mutex::new(AsyncPeekReader::new(reader)),
        writer: Mutex::new(TcpWrite {
            socket: writer,
            sequence: 0,
        }),
        protocol_version: MavlinkVersion::V2,
        recv_any_version: false,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub async fn tcpin<T: std::net::ToSocketAddrs>(address: T) -> io::Result<AsyncTcpConnection> {
    let addr = get_socket_addr(address)?;
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
                protocol_version: MavlinkVersion::V2,
                recv_any_version: false,
                #[cfg(feature = "signing")]
                signing_data: None,
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
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

struct TcpWrite {
    socket: OwnedWriteHalf,
    sequence: u8,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncTcpConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        let version = ReadVersion::from_async_conn_cfg::<_, M>(self);
        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg_async(reader.deref_mut(), version).await;
        #[cfg(feature = "signing")]
        let result = read_versioned_msg_async_signed(
            reader.deref_mut(),
            version,
            self.signing_data.as_ref(),
        )
        .await;
        result
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> Result<usize, crate::error::MessageWriteError> {
        let mut lock = self.writer.lock().await;

        let header = MavHeader {
            sequence: lock.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        lock.sequence = lock.sequence.wrapping_add(1);
        #[cfg(not(feature = "signing"))]
        let result =
            write_versioned_msg_async(&mut lock.socket, self.protocol_version, header, data).await;
        #[cfg(feature = "signing")]
        let result = write_versioned_msg_async_signed(
            &mut lock.socket,
            self.protocol_version,
            header,
            data,
            self.signing_data.as_ref(),
        )
        .await;
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

#[async_trait]
impl AsyncConnectable for TcpConnectable {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        let conn = if self.is_out {
            tcpout(&self.address).await
        } else {
            tcpin(&self.address).await
        };
        Ok(Box::new(conn?))
    }
}
