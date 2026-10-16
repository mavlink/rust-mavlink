use core::fmt::Display;
use std::io::{self, Read, Write};
use std::sync::MutexGuard;

use crate::MavlinkReader;
#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;
use crate::connection_shared::{
    ConnectionState, read_message_with_dialect, read_raw_message_with_dialect,
    write_message_with_dialect, write_raw_message,
};
use crate::error::{MessageReadError, MessageWriteError};
use crate::static_dialect::StaticDialect;
use crate::{
    Dialect, MAVLinkMessageRaw, MavFrame, MavHeader, MavlinkVersion, Message,
    connectable::ConnectionAddress,
};

/// The byte-oriented half of a blocking transport.
pub(crate) trait SyncTransport: Send + Sync {
    /// Incoming bytes.
    type Reader: Read;
    /// Outgoing bytes.
    type Writer: Write;

    /// Locks the reader.
    fn reader(&self) -> MutexGuard<'_, MavlinkReader<Self::Reader>>;

    /// Locks the writer. Returns `None` to skip sending when there is no writer or peer.
    fn lock_writer(&self) -> Option<MutexGuard<'_, Self::Writer>>;

    /// Whether `recv` and `recv_raw` should retry an error. Unused by `try_recv`.
    fn retry_receive(&self, _error: &MessageReadError) -> bool {
        false
    }

    /// Toggles nonblocking reads for `try_recv`. Does nothing by default.
    fn set_nonblocking(
        &self,
        _reader: &mut MavlinkReader<Self::Reader>,
        _enabled: bool,
    ) -> io::Result<()> {
        Ok(())
    }
}

/// A concrete blocking connection using generated messages.
pub type Connection<M> = DialectConnection<StaticDialect<M>>;

/// A concrete blocking connection using a dialect.
pub struct DialectConnection<D> {
    transport: Transport,
    dialect: D,
    state: ConnectionState,
}

pub(crate) enum Transport {
    #[cfg(feature = "transport-tcp")]
    Tcp(super::tcp::TcpConnection),
    #[cfg(feature = "transport-udp")]
    Udp(super::udp::UdpConnection),
    #[cfg(feature = "transport-direct-serial")]
    Serial(super::direct_serial::SerialConnection),
    File(super::file::FileConnection),
}

impl<D> DialectConnection<D> {
    pub(crate) fn new(transport: Transport, dialect: D) -> Self {
        Self {
            transport,
            dialect,
            state: ConnectionState::new(),
        }
    }
}

/// A MAVLink connection
pub trait MavConnection<M>: Send + Sync {
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
    /// Returns any eror encounter while receiving or deserializing a message.
    fn try_recv(&self) -> Result<(MavHeader, M), MessageReadError>;

    /// Send a MAVLink message.
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError>;

    /// Send a raw, unparsed MAVLink message.
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError>;

    /// Sets the MAVLink version to use for receiving (when `allow_recv_any_version()` is `false`) and sending messages.
    fn set_protocol_version(&mut self, version: MavlinkVersion);
    /// Gets the currently used MAVLink version
    fn protocol_version(&self) -> MavlinkVersion;

    /// Set wether MAVLink messages of either version may be received.
    ///
    /// If set to false only messages of the version configured with `set_protocol_version()` are received.
    fn set_allow_recv_any_version(&mut self, allow: bool);

    /// Wether messages of any MAVLink version may be received.
    fn allow_recv_any_version(&self) -> bool;

    /// Write whole frame.
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send_frame(&self, frame: &MavFrame<M>) -> Result<usize, MessageWriteError>
    where
        M: Message,
    {
        self.send(&frame.header, &frame.msg)
    }

    /// Read whole frame.
    ///
    /// # Errors
    ///
    /// Returns any eror encounter while receiving or deserializing a message.
    fn recv_frame(&self) -> Result<MavFrame<M>, MessageReadError>
    where
        M: Message,
    {
        let (header, msg) = self.recv()?;
        Ok(MavFrame {
            header,
            msg,
            protocol_version: self.protocol_version(),
        })
    }

    /// Send a message with default header.
    ///
    /// # Errors
    ///
    /// This function will return a [`MessageWriteError::Io`] error when sending fails.
    fn send_default(&self, data: &M) -> Result<usize, MessageWriteError> {
        self.send(&MavHeader::default(), data)
    }

    /// Setup secret key used for message signing, or disable message signing
    #[cfg(feature = "mav2-message-signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>);
}

impl<D: Dialect + Send + Sync> MavConnection<D::Message> for DialectConnection<D> {
    fn recv(&self) -> Result<(MavHeader, D::Message), MessageReadError> {
        dispatch_transport!(&self.transport, |transport| {
            let mut reader = transport.reader();
            loop {
                match read_message_with_dialect(&mut reader, &self.state, &self.dialect) {
                    Ok(message) => return Ok(message),
                    Err(error) if transport.retry_receive(&error) => {}
                    Err(error) => return Err(error),
                }
            }
        })
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        dispatch_transport!(&self.transport, |transport| {
            let mut reader = transport.reader();
            loop {
                match read_raw_message_with_dialect(&mut reader, &self.state, &self.dialect) {
                    Ok(message) => return Ok(message),
                    Err(error) if transport.retry_receive(&error) => {}
                    Err(error) => return Err(error),
                }
            }
        })
    }

    fn try_recv(&self) -> Result<(MavHeader, D::Message), MessageReadError> {
        dispatch_transport!(&self.transport, |transport| {
            let mut reader = transport.reader();
            transport.set_nonblocking(&mut reader, true)?;
            let result = read_message_with_dialect(&mut reader, &self.state, &self.dialect);
            transport.set_nonblocking(&mut reader, false)?;
            result
        })
    }

    fn send(&self, header: &MavHeader, data: &D::Message) -> Result<usize, MessageWriteError> {
        dispatch_transport!(&self.transport, |transport| {
            let Some(mut writer) = transport.lock_writer() else {
                return Ok(0);
            };
            let header = self.state.next_header(header);
            write_message_with_dialect(&mut *writer, &self.state, header, &self.dialect, data)
        })
    }

    fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError> {
        dispatch_transport!(&self.transport, |transport| {
            let Some(mut writer) = transport.lock_writer() else {
                return Ok(0);
            };
            write_raw_message(&mut *writer, data)
        })
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
/// The address selects the transport stored in the returned connection.
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

/// Connect to a MAVLink node using a runtime-loaded dialect.
///
/// The accepted address formats and errors are the same as [`connect`].
pub fn connect_with_dialect<D: Dialect + Send + Sync>(
    address: &str,
    dialect: D,
) -> io::Result<DialectConnection<D>> {
    ConnectionAddress::parse_address(address)?.connect_with_dialect(dialect)
}

/// A MAVLink connection address that can be connected to, establishing a [`MavConnection`]
pub trait Connectable: Display {
    /// Attempt to establish a blocking MAVLink connection
    ///
    /// # Errors
    ///
    /// When the connection could not be established a corresponding
    /// [`io::Error`] is returned
    fn connect<M: Message>(&self) -> io::Result<Connection<M>> {
        self.connect_with_dialect(StaticDialect::<M>::new())
    }

    /// Attempt to establish a blocking MAVLink connection with a runtime-loaded dialect.
    fn connect_with_dialect<D: Dialect + Send + Sync>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>>;
}

impl Connectable for ConnectionAddress {
    fn connect_with_dialect<D: Dialect + Send + Sync>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        match self {
            #[cfg(feature = "transport-tcp")]
            Self::Tcp(c) => c.connect_with_dialect(dialect),
            #[cfg(feature = "transport-udp")]
            Self::Udp(c) => c.connect_with_dialect(dialect),
            #[cfg(feature = "transport-direct-serial")]
            Self::Serial(c) => c.connect_with_dialect(dialect),
            Self::File(c) => c.connect_with_dialect(dialect),
        }
    }
}
