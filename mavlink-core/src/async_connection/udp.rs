//! Async UDP MAVLink connection

use core::{ops::DerefMut, task::Poll};
use std::io;
use std::{collections::VecDeque, io::Read, sync::Arc};

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio::{
    io::{AsyncRead, ReadBuf},
    net::UdpSocket,
};

use crate::connection::udp::config::{UdpConfig, UdpMode};
use crate::MAVLinkMessageRaw;
use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message, ReadVersion};

use super::{get_socket_addr, AsyncConnectable, AsyncMavConnection};

#[cfg(not(feature = "signing"))]
use crate::{read_raw_versioned_msg_async, read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "signing")]
use crate::{
    read_raw_versioned_msg_async_signed, read_versioned_msg_async_signed,
    write_versioned_msg_signed, SigningConfig, SigningData,
};

struct UdpRead {
    socket: Arc<UdpSocket>,
    buffer: VecDeque<u8>,
    last_recv_address: Option<std::net::SocketAddr>,
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
                    self.last_recv_address = Some(address);
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

struct UdpWrite {
    socket: Arc<UdpSocket>,
    dest: Option<std::net::SocketAddr>,
    sequence: u8,
}

pub struct AsyncUdpConnection {
    reader: Mutex<AsyncPeekReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    server: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

impl AsyncUdpConnection {
    fn new(
        socket: UdpSocket,
        server: bool,
        dest: Option<std::net::SocketAddr>,
    ) -> io::Result<Self> {
        let socket = Arc::new(socket);
        Ok(Self {
            server,
            reader: Mutex::new(AsyncPeekReader::new(UdpRead {
                socket: socket.clone(),
                buffer: VecDeque::new(),
                last_recv_address: None,
            })),
            writer: Mutex::new(UdpWrite {
                socket,
                dest,
                sequence: 0,
            }),
            protocol_version: MavlinkVersion::V2,
            recv_any_version: false,
            #[cfg(feature = "signing")]
            signing_data: None,
        })
    }
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncUdpConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        let version = ReadVersion::from_async_conn_cfg::<_, M>(self);
        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg_async(reader.deref_mut(), version).await;
            #[cfg(feature = "signing")]
            let result = read_versioned_msg_async_signed(
                reader.deref_mut(),
                version,
                self.signing_data.as_ref(),
            )
            .await;
            if self.server {
                if let addr @ Some(_) = reader.reader_ref().last_recv_address {
                    self.writer.lock().await.dest = addr;
                }
            }
            if let ok @ Ok(..) = result {
                return ok;
            }
        }
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;
        let version = ReadVersion::from_async_conn_cfg::<_, M>(self);
        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_raw_versioned_msg_async::<M, _>(reader.deref_mut(), version).await;
            #[cfg(feature = "signing")]
            let result = read_raw_versioned_msg_async_signed::<M, _>(
                reader.deref_mut(),
                version,
                self.signing_data.as_ref(),
            )
            .await;
            if self.server {
                if let addr @ Some(_) = reader.reader_ref().last_recv_address {
                    self.writer.lock().await.dest = addr;
                }
            }
            if let ok @ Ok(..) = result {
                return ok;
            }
        }
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> Result<usize, crate::error::MessageWriteError> {
        let mut guard = self.writer.lock().await;
        let state = &mut *guard;

        let header = MavHeader {
            sequence: state.sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        state.sequence = state.sequence.wrapping_add(1);

        let len = if let Some(addr) = state.dest {
            let mut buf = Vec::new();
            #[cfg(not(feature = "signing"))]
            write_versioned_msg_async(
                &mut buf,
                self.protocol_version,
                header,
                data,
                #[cfg(feature = "signing")]
                self.signing_data.as_ref(),
            )
            .await?;
            #[cfg(feature = "signing")]
            write_versioned_msg_signed(
                &mut buf,
                self.protocol_version,
                header,
                data,
                #[cfg(feature = "signing")]
                self.signing_data.as_ref(),
            )?;
            state.socket.send_to(&buf, addr).await?
        } else {
            0
        };

        Ok(len)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.recv_any_version = allow;
    }

    fn allow_recv_any_version(&self) -> bool {
        self.recv_any_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config);
    }
}

#[async_trait]
impl AsyncConnectable for UdpConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        let (addr, server, dest): (&str, _, _) = match self.mode {
            UdpMode::Udpin => (&self.address, true, None),
            _ => ("0.0.0.0:0", false, Some(get_socket_addr(&self.address)?)),
        };
        let socket = UdpSocket::bind(addr).await?;
        if matches!(self.mode, UdpMode::Udpcast) {
            socket.set_broadcast(true)?;
        }
        Ok(Box::new(AsyncUdpConnection::new(socket, server, dest)?))
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
            last_recv_address: None,
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
