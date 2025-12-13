#[cfg(feature = "tcp")]
pub mod tcp;

#[cfg(feature = "udp")]
pub mod udp;

#[cfg(feature = "direct-serial")]
pub mod direct_serial;

pub mod file;

use core::fmt::Display;
use core::marker::PhantomData;
use std::io::{self};

#[cfg(feature = "tcp")]
use self::tcp::TcpConnection;

#[cfg(feature = "udp")]
use self::udp::UdpConnection;

#[cfg(feature = "direct-serial")]
use self::direct_serial::SerialConnection;

use self::file::FileConnection;

#[cfg(feature = "signing")]
use crate::SigningConfig;

use crate::error::MessageReadError;
use crate::error::MessageWriteError;
use crate::{
    connectable::ConnectionAddress, MAVLinkMessageRaw, MavFrame, MavHeader, MavlinkVersion, Message,
};

/// A MAVLink connection
pub trait MavConnection<M: Message> {
    /// Receive a MAVLink message.
    ///
    /// May blocks until a valid frame is received, ignoring invalid messages.
    ///
    /// # Errors
    ///
    /// If the connection type blocks until a valid message is received this can not
    /// return any errors, otherwise return any errors that occured while receiving.
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError>;

    /// Receive a raw, unparsed MAVLink message.
    ///
    /// Blocks until a valid frame is received, ignoring invalid messages.
    ///
    /// # Errors
    ///
    /// If the connection type blocks until a valid message is received this can not
    /// return any errors, otherwise return any errors that occured while receiving.
    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError>;

    /// Try to receive a MAVLink message.
    ///
    /// Non-blocking variant of `recv()`, returns immediately with a `MessageReadError`
    /// if there is an error or no message is available.
    ///
    /// # Errors
    ///
    /// Returns any eror encounter while receiving or deserializing a message
    fn try_recv(&self) -> Result<(MavHeader, M), MessageReadError>;

    /// Send a MAVLink message
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError>;

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
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send_frame(&self, frame: &MavFrame<M>) -> Result<usize, MessageWriteError> {
        self.send(&frame.header, &frame.msg)
    }

    /// Read whole frame
    ///
    /// # Errors
    ///
    /// Returns any eror encounter while receiving or deserializing a message
    fn recv_frame(&self) -> Result<MavFrame<M>, MessageReadError> {
        let (header, msg) = self.recv()?;
        let protocol_version = self.protocol_version();
        Ok(MavFrame {
            header,
            msg,
            protocol_version,
        })
    }

    /// Send a message with default header
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send_default(&self, data: &M) -> Result<usize, MessageWriteError> {
        let header = MavHeader::default();
        self.send(&header, data)
    }

    /// Setup secret key used for message signing, or disable message signing
    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>);
}

/// Concrete MAVLink connection returned by [`connect`].
pub struct Connection<M: Message> {
    inner: ConnectionInner,
    _p: PhantomData<M>,
}

enum ConnectionInner {
    #[cfg(feature = "tcp")]
    Tcp(TcpConnection),
    #[cfg(feature = "udp")]
    Udp(UdpConnection),
    #[cfg(feature = "direct-serial")]
    Serial(SerialConnection),
    File(FileConnection),
}

impl<M: Message> Connection<M> {
    fn new(inner: ConnectionInner) -> Self {
        Self {
            inner,
            _p: PhantomData,
        }
    }
}

#[cfg(feature = "tcp")]
impl<M: Message> From<TcpConnection> for Connection<M> {
    fn from(value: TcpConnection) -> Self {
        Self::new(ConnectionInner::Tcp(value))
    }
}

#[cfg(feature = "udp")]
impl<M: Message> From<UdpConnection> for Connection<M> {
    fn from(value: UdpConnection) -> Self {
        Self::new(ConnectionInner::Udp(value))
    }
}

#[cfg(feature = "direct-serial")]
impl<M: Message> From<SerialConnection> for Connection<M> {
    fn from(value: SerialConnection) -> Self {
        Self::new(ConnectionInner::Serial(value))
    }
}

impl<M: Message> From<FileConnection> for Connection<M> {
    fn from(value: FileConnection) -> Self {
        Self::new(ConnectionInner::File(value))
    }
}

impl<M: Message> MavConnection<M> for Connection<M> {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        match &self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => <TcpConnection as MavConnection<M>>::recv(conn),
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => <UdpConnection as MavConnection<M>>::recv(conn),
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => <SerialConnection as MavConnection<M>>::recv(conn),
            ConnectionInner::File(conn) => <FileConnection as MavConnection<M>>::recv(conn),
        }
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        match &self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => <TcpConnection as MavConnection<M>>::recv_raw(conn),
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => <UdpConnection as MavConnection<M>>::recv_raw(conn),
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => <SerialConnection as MavConnection<M>>::recv_raw(conn),
            ConnectionInner::File(conn) => <FileConnection as MavConnection<M>>::recv_raw(conn),
        }
    }

    fn try_recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        match &self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => <TcpConnection as MavConnection<M>>::try_recv(conn),
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => <UdpConnection as MavConnection<M>>::try_recv(conn),
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => <SerialConnection as MavConnection<M>>::try_recv(conn),
            ConnectionInner::File(conn) => <FileConnection as MavConnection<M>>::try_recv(conn),
        }
    }

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError> {
        match &self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => {
                <TcpConnection as MavConnection<M>>::send(conn, header, data)
            }
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => {
                <UdpConnection as MavConnection<M>>::send(conn, header, data)
            }
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => {
                <SerialConnection as MavConnection<M>>::send(conn, header, data)
            }
            ConnectionInner::File(conn) => {
                <FileConnection as MavConnection<M>>::send(conn, header, data)
            }
        }
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        match &mut self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => {
                <TcpConnection as MavConnection<M>>::set_protocol_version(conn, version);
            }
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => {
                <UdpConnection as MavConnection<M>>::set_protocol_version(conn, version);
            }
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => {
                <SerialConnection as MavConnection<M>>::set_protocol_version(conn, version);
            }
            ConnectionInner::File(conn) => {
                <FileConnection as MavConnection<M>>::set_protocol_version(conn, version);
            }
        }
    }

    fn protocol_version(&self) -> MavlinkVersion {
        match &self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => {
                <TcpConnection as MavConnection<M>>::protocol_version(conn)
            }
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => {
                <UdpConnection as MavConnection<M>>::protocol_version(conn)
            }
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => {
                <SerialConnection as MavConnection<M>>::protocol_version(conn)
            }
            ConnectionInner::File(conn) => {
                <FileConnection as MavConnection<M>>::protocol_version(conn)
            }
        }
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        match &mut self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => {
                <TcpConnection as MavConnection<M>>::set_allow_recv_any_version(conn, allow);
            }
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => {
                <UdpConnection as MavConnection<M>>::set_allow_recv_any_version(conn, allow);
            }
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => {
                <SerialConnection as MavConnection<M>>::set_allow_recv_any_version(conn, allow);
            }
            ConnectionInner::File(conn) => {
                <FileConnection as MavConnection<M>>::set_allow_recv_any_version(conn, allow);
            }
        }
    }

    fn allow_recv_any_version(&self) -> bool {
        match &self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => {
                <TcpConnection as MavConnection<M>>::allow_recv_any_version(conn)
            }
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => {
                <UdpConnection as MavConnection<M>>::allow_recv_any_version(conn)
            }
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => {
                <SerialConnection as MavConnection<M>>::allow_recv_any_version(conn)
            }
            ConnectionInner::File(conn) => {
                <FileConnection as MavConnection<M>>::allow_recv_any_version(conn)
            }
        }
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        let mut signing_data = signing_data;
        match &mut self.inner {
            #[cfg(feature = "tcp")]
            ConnectionInner::Tcp(conn) => {
                <TcpConnection as MavConnection<M>>::setup_signing(conn, signing_data.take());
            }
            #[cfg(feature = "udp")]
            ConnectionInner::Udp(conn) => {
                <UdpConnection as MavConnection<M>>::setup_signing(conn, signing_data.take());
            }
            #[cfg(feature = "direct-serial")]
            ConnectionInner::Serial(conn) => {
                <SerialConnection as MavConnection<M>>::setup_signing(conn, signing_data.take());
            }
            ConnectionInner::File(conn) => {
                <FileConnection as MavConnection<M>>::setup_signing(conn, signing_data.take());
            }
        }
    }
}

/// Connect to a MAVLink node by address string.
///
/// The address must be in one of the following formats:
///
///  * `tcpin:<addr>:<port>` to create a TCP server, listening an incoming connection
///  * `tcpout:<addr>:<port>` to create a TCP client
///  * `udpin:<addr>:<port>` to create a UDP server, listening for incoming packets
///  * `udpout:<addr>:<port>` to create a UDP client
///  * `udpbcast:<addr>:<port>` to create a UDP broadcast
///  * `serial:<port>:<baudrate>` to create a serial connection
///  * `file:<path>` to extract file data, writing to such a connection does nothing
///
/// The type of the connection is determined at runtime based on the address type
/// and the resulting [`Connection`] enum stores the concrete transport.
///
/// # Errors
///
/// - [`AddrNotAvailable`] if the address string could not be parsed as a valid MAVLink address
/// - When the connection could not be established a corresponding [`io::Error`] is returned
///
/// [`AddrNotAvailable`]: io::ErrorKind::AddrNotAvailable
pub fn connect<M: Message + Sync + Send>(address: &str) -> io::Result<Connection<M>> {
    ConnectionAddress::parse_address(address)?.connect::<M>()
}

/// Returns the socket address for the given address.
#[cfg(any(feature = "tcp", feature = "udp"))]
pub(crate) fn get_socket_addr<T: std::net::ToSocketAddrs>(
    address: &T,
) -> Result<std::net::SocketAddr, io::Error> {
    address
        .to_socket_addrs()?
        .next()
        .ok_or(io::Error::other("Host address lookup failed"))
}

/// A MAVLink connection address that can be connected to, establishing a [`MavConnection`]
pub trait Connectable: Display {
    /// Attempt to establish a blocking MAVLink connection
    ///
    /// # Errors
    ///
    /// When the connection could not be established a corresponding
    /// [`io::Error`] is returned
    fn connect<M: Message>(&self) -> io::Result<Connection<M>>;
}

impl Connectable for ConnectionAddress {
    fn connect<M>(&self) -> std::io::Result<Connection<M>>
    where
        M: Message,
    {
        match self {
            #[cfg(feature = "tcp")]
            Self::Tcp(config) => config.connect::<M>(),
            #[cfg(feature = "udp")]
            Self::Udp(config) => config.connect::<M>(),
            #[cfg(feature = "direct-serial")]
            Self::Serial(config) => config.connect::<M>(),
            Self::File(config) => config.connect::<M>(),
        }
    }
}
