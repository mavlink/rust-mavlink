use core::fmt::Display;
use std::io::{self, Read, Write};
use std::sync::MutexGuard;

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;
use crate::connection_shared::{
    ConnectionState, read_message_with_dialect, read_raw_message_with_dialect,
    write_message_with_dialect, write_raw_message,
};
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::static_dialect::StaticDialect;
use crate::{
    Dialect, MAVLinkMessageRaw, MavFrame, MavHeader, MavlinkVersion, Message,
    connectable::ConnectionAddress,
};

/// The byte-oriented half of a blocking transport.
pub(crate) trait SyncTransport: Send + Sync {
    type Reader: Read;
    type Writer: Write;

    fn reader(&self) -> MutexGuard<'_, PeekReader<Self::Reader>>;

    fn writer(&self) -> Option<MutexGuard<'_, Self::Writer>>;

    fn set_nonblocking(
        &self,
        _reader: &mut PeekReader<Self::Reader>,
        _enabled: bool,
    ) -> io::Result<()> {
        Ok(())
    }

    fn retry_receive(&self, _error: &MessageReadError) -> bool {
        false
    }

    fn can_send(&self, _writer: &Self::Writer) -> bool {
        true
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader;
}

/// The one blocking MAVLink connection implementation.
pub struct ConnectionCore<T, D = ()> {
    transport: T,
    dialect: D,
    state: ConnectionState,
}

impl<T, D> ConnectionCore<T, D> {
    pub(crate) fn new(transport: T, dialect: D) -> Self {
        Self {
            transport,
            dialect,
            state: ConnectionState::new(),
        }
    }
}

impl<T> ConnectionCore<T> {
    pub(crate) fn new_static(transport: T) -> Self {
        Self {
            transport,
            dialect: (),
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

macro_rules! impl_mav_connection {
    ([$($generics:tt)*] $connection:ty, $message:ty, |$core:ident| $dialect:expr) => {
        impl<$($generics)*> MavConnection<$message> for $connection {
            fn recv(&self) -> Result<(MavHeader, $message), MessageReadError> {
                let $core = self;
                let dialect = $dialect;
                let mut reader = $core.transport.reader();
                loop {
                    match read_message_with_dialect(&mut reader, &$core.state, dialect) {
                        Ok(message) => return Ok(message),
                        Err(error) if $core.transport.retry_receive(&error) => {}
                        Err(error) => return Err(error),
                    }
                }
            }

            fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
                let $core = self;
                let dialect = $dialect;
                let mut reader = $core.transport.reader();
                loop {
                    match read_raw_message_with_dialect(&mut reader, &$core.state, dialect) {
                        Ok(message) => return Ok(message),
                        Err(error) if $core.transport.retry_receive(&error) => {}
                        Err(error) => return Err(error),
                    }
                }
            }

            fn try_recv(&self) -> Result<(MavHeader, $message), MessageReadError> {
                let $core = self;
                let dialect = $dialect;
                let mut reader = $core.transport.reader();
                $core.transport.set_nonblocking(&mut reader, true)?;
                let result = read_message_with_dialect(&mut reader, &$core.state, dialect);
                $core.transport.set_nonblocking(&mut reader, false)?;
                result
            }

            fn send(&self, header: &MavHeader, data: &$message) -> Result<usize, MessageWriteError> {
                let $core = self;
                let dialect = $dialect;
                let Some(mut writer) = $core.transport.writer() else {
                    return Ok(0);
                };
                if !$core.transport.can_send(&writer) {
                    return Ok(0);
                }
                let header = $core.transport.next_send_header(&mut writer, header);
                write_message_with_dialect(&mut *writer, &$core.state, header, dialect, data)
            }

            fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError> {
                let Some(mut writer) = self.transport.writer() else {
                    return Ok(0);
                };
                if !self.transport.can_send(&writer) {
                    return Ok(0);
                }
                write_raw_message(&mut *writer, data)
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
    };
}

impl_mav_connection!([T: SyncTransport, D: Dialect + Send + Sync] ConnectionCore<T, D>, D::Message, |core| &core.dialect);
impl_mav_connection!([T: SyncTransport, M: Message] ConnectionCore<T>, M, |core| &StaticDialect::<M>::new());

/// A blocking MAVLink connection returned by [`connect`].
pub type Connection<M> = Box<dyn MavConnection<M>>;
/// A blocking MAVLink connection returned by [`connect_with_dialect`].
pub type DialectConnection<D> = Connection<<D as Dialect>::Message>;

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
/// The type of the connection is determined at runtime based on the address type, so the
/// connection is returned as a trait object.
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
pub fn connect_with_dialect<D: Dialect + Send + Sync + 'static>(
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
    fn connect<M: Message>(&self) -> io::Result<Connection<M>>;

    /// Attempt to establish a blocking MAVLink connection with a runtime-loaded dialect.
    fn connect_with_dialect<D: Dialect + Send + Sync + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>>;
}

impl Connectable for ConnectionAddress {
    fn connect<M: Message>(&self) -> io::Result<Connection<M>> {
        match self {
            #[cfg(feature = "transport-tcp")]
            Self::Tcp(c) => c.connect(),
            #[cfg(feature = "transport-udp")]
            Self::Udp(c) => c.connect(),
            #[cfg(feature = "transport-direct-serial")]
            Self::Serial(c) => c.connect(),
            Self::File(c) => c.connect(),
        }
    }

    fn connect_with_dialect<D: Dialect + Send + Sync + 'static>(
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
