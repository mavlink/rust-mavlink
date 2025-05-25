use crate::{connectable::ConnectionAddress, MavFrame, MavHeader, MavlinkVersion, Message};

use core::fmt::Display;
use std::io::{self};

#[cfg(feature = "tcp")]
mod tcp;

#[cfg(feature = "udp")]
mod udp;

#[cfg(feature = "direct-serial")]
mod direct_serial;

#[cfg(feature = "signing")]
use crate::SigningConfig;

mod file;

/// A MAVLink connection
pub trait MavConnection<M: Message> {
    /// Receive a MAVLink message.
    ///
    /// Blocks until a valid frame is received, ignoring invalid messages.
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError>;

    /// Send a MAVLink message
    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, crate::error::MessageWriteError>;

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
    fn send_frame(&self, frame: &MavFrame<M>) -> Result<usize, crate::error::MessageWriteError> {
        self.send(&frame.header, &frame.msg)
    }

    /// Read whole frame
    fn recv_frame(&self) -> Result<MavFrame<M>, crate::error::MessageReadError> {
        let (header, msg) = self.recv()?;
        let protocol_version = self.protocol_version();
        Ok(MavFrame {
            header,
            msg,
            protocol_version,
        })
    }

    /// Send a message with default header
    fn send_default(&self, data: &M) -> Result<usize, crate::error::MessageWriteError> {
        let header = MavHeader::default();
        self.send(&header, data)
    }

    /// Setup secret key used for message signing, or disable message signing
    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>);
}

/// Connect to a MAVLink node by address string.
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
pub fn connect<M: Message + Sync + Send>(
    address: &str,
) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
    ConnectionAddress::parse_address(address)?.connect::<M>()
}

/// Returns the socket address for the given address.
#[cfg(any(feature = "tcp", feature = "udp"))]
pub(crate) fn get_socket_addr<T: std::net::ToSocketAddrs>(
    address: &T,
) -> Result<std::net::SocketAddr, io::Error> {
    address.to_socket_addrs()?.next().ok_or(io::Error::new(
        io::ErrorKind::Other,
        "Host address lookup failed",
    ))
}

pub trait Connectable: Display {
    fn connect<M: Message>(&self) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>>;
}

impl Connectable for ConnectionAddress {
    fn connect<M>(&self) -> std::io::Result<Box<dyn crate::MavConnection<M> + Sync + Send>>
    where
        M: Message,
    {
        match self {
            #[cfg(feature = "tcp")]
            Self::Tcp(connectable) => connectable.connect::<M>(),
            #[cfg(feature = "udp")]
            Self::Udp(connectable) => connectable.connect::<M>(),
            #[cfg(feature = "direct-serial")]
            Self::Serial(connectable) => connectable.connect::<M>(),
            Self::File(connectable) => connectable.connect::<M>(),
        }
    }
}
