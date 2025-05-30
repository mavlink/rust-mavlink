use async_trait::async_trait;
use tokio::io;

use crate::{connectable::ConnectionAddress, MavFrame, MavHeader, MavlinkVersion, Message};

#[cfg(feature = "tcp")]
mod tcp;

#[cfg(feature = "udp")]
mod udp;

#[cfg(feature = "direct-serial")]
mod direct_serial;

mod file;

#[cfg(feature = "signing")]
use crate::SigningConfig;

/// An async MAVLink connection
#[async_trait::async_trait]
pub trait AsyncMavConnection<M: Message + Sync + Send> {
    /// Receive a mavlink message.
    ///
    /// Yield until a valid frame is received, ignoring invalid messages.
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError>;

    /// Send a mavlink message
    async fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> Result<usize, crate::error::MessageWriteError>;

    /// Sets the MAVLink version to use for receiving (when `allow_recv_any_version()` is `false`) and sending messages.
    fn set_protocol_version(&mut self, version: MavlinkVersion);
    /// Gets the currently used MAVLink version
    fn protocol_version(&self) -> MavlinkVersion;

    /// Set wether MAVLink messages of either version may be received.
    ///
    /// If set to false only messages of the version configured with `set_protocol_version()` are received.
    fn set_allow_recv_any_version(&mut self, allow: bool);
    /// Wether messages of any MAVLink version may be received
    fn allow_recv_any_version(&self) -> bool;

    /// Write whole frame
    async fn send_frame(
        &self,
        frame: &MavFrame<M>,
    ) -> Result<usize, crate::error::MessageWriteError> {
        self.send(&frame.header, &frame.msg).await
    }

    /// Read whole frame
    async fn recv_frame(&self) -> Result<MavFrame<M>, crate::error::MessageReadError> {
        let (header, msg) = self.recv().await?;
        let protocol_version = self.protocol_version();
        Ok(MavFrame {
            header,
            msg,
            protocol_version,
        })
    }

    /// Send a message with default header
    async fn send_default(&self, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let header = MavHeader::default();
        self.send(&header, data).await
    }

    /// Setup secret key used for message signing, or disable message signing
    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>);
}

/// Connect asynchronously to a MAVLink node by address string.
///
/// The address must be in one of the following formats:
///
///  * `tcpin:<addr>:<port>` to create a TCP server, listening for an incoming connection
///  * `tcpout:<addr>:<port>` to create a TCP client
///  * `udpin:<addr>:<port>` to create a UDP server, listening for incoming packets
///  * `udpout:<addr>:<port>` to create a UDP client
///  * `udpbcast:<addr>:<port>` to create a UDP broadcast
///  * `serial:<port>:<baudrate>` to create a serial connection
///  * `file:<path>` to extract file data, writing to such a connection does nothing
///
/// The type of the connection is determined at runtime based on the address type, so the
/// connection is returned as a trait object.
pub async fn connect_async<M: Message + Sync + Send>(
    address: &str,
) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>> {
    ConnectionAddress::parse_address(address)?
        .connect_async::<M>()
        .await
}

/// Returns the socket address for the given address.
#[cfg(any(feature = "tcp", feature = "udp"))]
pub(crate) fn get_socket_addr<T: std::net::ToSocketAddrs>(
    address: T,
) -> Result<std::net::SocketAddr, io::Error> {
    let addr = match address.to_socket_addrs()?.next() {
        Some(addr) => addr,
        None => {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "Host address lookup failed",
            ));
        }
    };
    Ok(addr)
}

/// A MAVLink connection address that can be connected to, establishing an [`AsyncMavConnection`]
///
/// This is the `async` version of `Connectable`.
#[async_trait]
pub trait AsyncConnectable {
    /// Attempt to establish an asynchronous MAVLink connection
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send;
}

#[async_trait]
impl AsyncConnectable for ConnectionAddress {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        match self {
            #[cfg(feature = "tcp")]
            Self::Tcp(connectable) => connectable.connect_async::<M>().await,
            #[cfg(feature = "udp")]
            Self::Udp(connectable) => connectable.connect_async::<M>().await,
            #[cfg(feature = "direct-serial")]
            Self::Serial(connectable) => connectable.connect_async::<M>().await,
            Self::File(connectable) => connectable.connect_async::<M>().await,
        }
    }
}
