//! Async UDP MAVLink connection

use core::{ops::DerefMut, task::Poll};
use std::{collections::VecDeque, io::Read, sync::Arc};

use tokio::{
    io::{self, AsyncRead, ReadBuf},
    net::UdpSocket,
    sync::Mutex,
};

use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message};

use super::{get_socket_addr, AsyncMavConnection};

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "signing")]
use crate::{
    read_versioned_msg_async_signed, write_versioned_msg_signed, SigningConfig, SigningData,
};

pub async fn select_protocol<M: Message + Sync + Send>(
    address: &str,
) -> tokio::io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>> {
    let connection = if let Some(address) = address.strip_prefix("udpin:") {
        udpin(address).await
    } else if let Some(address) = address.strip_prefix("udpout:") {
        udpout(address).await
    } else if let Some(address) = address.strip_prefix("udpbcast:") {
        udpbcast(address).await
    } else {
        Err(tokio::io::Error::new(
            tokio::io::ErrorKind::AddrNotAvailable,
            "Protocol unsupported",
        ))
    };

    Ok(Box::new(connection?))
}

pub async fn udpbcast<T: std::net::ToSocketAddrs>(address: T) -> tokio::io::Result<UdpConnection> {
    let addr = get_socket_addr(address)?;
    let socket = UdpSocket::bind("0.0.0.0:0").await?;
    socket
        .set_broadcast(true)
        .expect("Couldn't bind to broadcast address.");
    UdpConnection::new(socket, false, Some(addr))
}

pub async fn udpout<T: std::net::ToSocketAddrs>(address: T) -> tokio::io::Result<UdpConnection> {
    let addr = get_socket_addr(address)?;
    let socket = UdpSocket::bind("0.0.0.0:0").await?;
    UdpConnection::new(socket, false, Some(addr))
}

pub async fn udpin<T: std::net::ToSocketAddrs>(address: T) -> tokio::io::Result<UdpConnection> {
    let addr = address
        .to_socket_addrs()
        .unwrap()
        .next()
        .expect("Invalid address");
    let socket = UdpSocket::bind(addr).await?;
    UdpConnection::new(socket, true, None)
}

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
        buf: &mut tokio::io::ReadBuf<'_>,
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

pub struct UdpConnection {
    reader: Mutex<AsyncPeekReader<UdpRead>>,
    writer: Mutex<UdpWrite>,
    protocol_version: MavlinkVersion,
    server: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

impl UdpConnection {
    fn new(
        socket: UdpSocket,
        server: bool,
        dest: Option<std::net::SocketAddr>,
    ) -> tokio::io::Result<Self> {
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
            #[cfg(feature = "signing")]
            signing_data: None,
        })
    }
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for UdpConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut reader = self.reader.lock().await;

        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg_async(reader.deref_mut(), self.protocol_version).await;
            #[cfg(feature = "signing")]
            let result = read_versioned_msg_async_signed(
                reader.deref_mut(),
                self.protocol_version,
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

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config)
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
