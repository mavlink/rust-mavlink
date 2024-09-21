use tokio::io;

use crate::{MavFrame, MavHeader, MavlinkVersion, Message};

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

    fn set_protocol_version(&mut self, version: MavlinkVersion);
    fn get_protocol_version(&self) -> MavlinkVersion;

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
        let protocol_version = self.get_protocol_version();
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
///  * `tcpin:<addr>:<port>` to create a TCP server, listening for incoming connections
///  * `tcpout:<addr>:<port>` to create a TCP client
///  * `udpin:<addr>:<port>` to create a UDP server, listening for incoming packets
///  * `udpout:<addr>:<port>` to create a UDP client
///  * `udpbcast:<addr>:<port>` to create a UDP broadcast
///  * `serial:<port>:<baudrate>` to create a serial connection
///  * `file:<path>` to extract file data
///
/// The type of the connection is determined at runtime based on the address type, so the
/// connection is returned as a trait object.
pub async fn connect_async<M: Message + Sync + Send>(
    address: &str,
) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>> {
    let protocol_err = Err(io::Error::new(
        io::ErrorKind::AddrNotAvailable,
        "Protocol unsupported",
    ));

    if cfg!(feature = "tcp") && address.starts_with("tcp") {
        #[cfg(feature = "tcp")]
        {
            tcp::select_protocol(address).await
        }
        #[cfg(not(feature = "tcp"))]
        {
            protocol_err
        }
    } else if cfg!(feature = "udp") && address.starts_with("udp") {
        #[cfg(feature = "udp")]
        {
            udp::select_protocol(address).await
        }
        #[cfg(not(feature = "udp"))]
        {
            protocol_err
        }
    } else if cfg!(feature = "direct-serial") && address.starts_with("serial") {
        #[cfg(feature = "direct-serial")]
        {
            Ok(Box::new(direct_serial::open(&address["serial:".len()..])?))
        }
        #[cfg(not(feature = "direct-serial"))]
        {
            protocol_err
        }
    } else if address.starts_with("file") {
        Ok(Box::new(file::open(&address["file:".len()..]).await?))
    } else {
        protocol_err
    }
}

/// Returns the socket address for the given address.
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
