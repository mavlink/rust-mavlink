//! Async TCP MAVLink connection

use super::{get_socket_addr, AsyncMavConnection};
use crate::async_peek_reader::AsyncPeekReader;
use crate::{
    read_v1_raw_message, read_v1_raw_message_async, read_v2_raw_message_async,
    read_v2_raw_message_async_signed, MAVLinkRawMessage, MAVLinkV2MessageRaw, MavHeader,
    MavlinkVersion, Message,
};

use core::ops::DerefMut;
use tokio::io;
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::Mutex;

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "signing")]
use crate::{
    read_versioned_msg_async_signed, write_versioned_msg_async_signed, SigningConfig, SigningData,
};

pub async fn select_protocol<M: Message + Sync + Send>(
    address: &str,
) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>> {
    let connection = if let Some(address) = address.strip_prefix("tcpout:") {
        tcpout(address).await
    } else if let Some(address) = address.strip_prefix("tcpin:") {
        tcpin(address).await
    } else {
        Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Protocol unsupported",
        ))
    };

    Ok(Box::new(connection?))
}

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
        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg_async(reader.deref_mut(), self.protocol_version).await;
        #[cfg(feature = "signing")]
        let result = read_versioned_msg_async_signed(
            reader.deref_mut(),
            self.protocol_version,
            self.signing_data.as_ref(),
        )
        .await;
        result
    }

    async fn recv_raw(&self) -> Result<MAVLinkRawMessage, crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        #[cfg(not(feature = "signing"))]
        let result = match self.protocol_version {
            MavlinkVersion::V1 => {
                MAVLinkRawMessage::V1(read_v1_raw_message_async::<M, _>(reader.deref_mut()).await?)
            }
            MavlinkVersion::V2 => {
                MAVLinkRawMessage::V2(read_v2_raw_message_async::<M, _>(reader.deref_mut()).await?)
            }
        };
        #[cfg(feature = "signing")]
        let result = match self.protocol_version {
            MavlinkVersion::V1 => {
                MAVLinkRawMessage::V1(read_v1_raw_message_async::<M, _>(reader.deref_mut()).await?)
            }
            MavlinkVersion::V2 => MAVLinkRawMessage::V2(
                read_v2_raw_message_async_signed::<M, _>(
                    reader.deref_mut(),
                    self.signing_data.as_ref(),
                )
                .await?,
            ),
        };

        Ok(result)
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

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config)
    }
}
