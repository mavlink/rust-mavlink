use std::io;

use async_trait::async_trait;
use futures::{FutureExt, lock::Mutex};
use tokio::io::{AsyncRead, AsyncWrite};

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;
use crate::async_peek_reader::AsyncPeekReader;
use crate::error::{MessageReadError, MessageWriteError};
use crate::static_dialect::StaticDialect;
use crate::{
    Dialect, MAVLinkMessageRaw, MavFrame, MavHeader, MavlinkVersion, Message,
    connectable::ConnectionAddress,
};

/// The byte-oriented half of a Tokio transport.
#[doc(hidden)]
pub(crate) trait AsyncTransport: Send + Sync {
    type Reader: AsyncRead + Unpin + Send;
    type Writer: AsyncWrite + Unpin + Send;

    fn reader(&self) -> &Mutex<AsyncPeekReader<Self::Reader>>;

    fn writer(&self) -> Option<&Mutex<Self::Writer>>;

    fn retry_receive(&self, _error: &MessageReadError) -> bool {
        false
    }

    fn try_recv_is_nonblocking(&self) -> bool {
        false
    }

    fn can_send(&self, _writer: &Self::Writer) -> bool {
        true
    }

    fn next_send_header(&self, writer: &mut Self::Writer, header: &MavHeader) -> MavHeader;
}

/// The one Tokio MAVLink connection implementation.
pub struct AsyncConnectionCore<T, D = ()> {
    transport: T,
    dialect: D,
    state: crate::connection_shared::ConnectionState,
}

impl<T, D> AsyncConnectionCore<T, D> {
    pub(crate) fn new(transport: T, dialect: D) -> Self {
        Self {
            transport,
            dialect,
            state: crate::connection_shared::ConnectionState::new(),
        }
    }
}

impl<T> AsyncConnectionCore<T> {
    pub(crate) fn new_static(transport: T) -> Self {
        Self {
            transport,
            dialect: (),
            state: crate::connection_shared::ConnectionState::new(),
        }
    }
}

/// An async MAVLink connection
#[async_trait]
pub trait AsyncMavConnection<M: Sync + Send> {
    /// Receive a mavlink message.
    ///
    /// Yield until a valid frame is received, ignoring invalid messages.
    async fn recv(&self) -> Result<(MavHeader, M), MessageReadError>;

    /// Receive a raw, unparsed mavlink message.
    ///
    /// Yield until a valid frame is received, ignoring invalid messages.
    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError>;

    /// Try to receive a MAVLink message.
    ///
    /// Non-blocking variant of `recv()`, returns immediately with a `MessageReadError`
    /// if there is an error or no message is available.
    ///
    /// # Errors
    ///
    /// Returns any eror encounter while receiving or deserializing a message.
    async fn try_recv(&self) -> Result<(MavHeader, M), MessageReadError>;

    /// Send a mavlink message
    async fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError>;

    /// Send a raw, unparsed mavlink message
    async fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError>;

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
    async fn send_frame(&self, frame: &MavFrame<M>) -> Result<usize, MessageWriteError>
    where
        M: Message,
    {
        self.send(&frame.header, &frame.msg).await
    }

    /// Read whole frame.
    async fn recv_frame(&self) -> Result<MavFrame<M>, MessageReadError>
    where
        M: Message,
    {
        let (header, msg) = self.recv().await?;
        Ok(MavFrame {
            header,
            msg,
            protocol_version: self.protocol_version(),
        })
    }

    /// Send a message with default header.
    async fn send_default(&self, data: &M) -> Result<usize, MessageWriteError> {
        self.send(&MavHeader::default(), data).await
    }

    /// Setup secret key used for message signing, or disable message signing.
    #[cfg(feature = "mav2-message-signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>);
}

macro_rules! impl_async_mav_connection {
    ([$($generics:tt)*] [$($bounds:tt)*] $connection:ty, $message:ty, |$core:ident| $dialect:expr) => {
        #[async_trait]
        impl<$($generics)*> AsyncMavConnection<$message> for $connection where $($bounds)* {
            async fn recv(&self) -> Result<(MavHeader, $message), MessageReadError> {
                let $core = self;
                let dialect = $dialect;
                let mut reader = $core.transport.reader().lock().await;
                loop {
                    match crate::connection_shared::read_message_async_with_dialect(
                        &mut reader,
                        &$core.state,
                        dialect,
                    )
                    .await
                    {
                        Ok(message) => return Ok(message),
                        Err(error) if $core.transport.retry_receive(&error) => {}
                        Err(error) => return Err(error),
                    }
                }
            }

            async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
                let $core = self;
                let dialect = $dialect;
                let mut reader = $core.transport.reader().lock().await;
                loop {
                    match crate::connection_shared::read_raw_message_async_with_dialect(
                        &mut reader,
                        &$core.state,
                        dialect,
                    )
                    .await
                    {
                        Ok(message) => return Ok(message),
                        Err(error) if $core.transport.retry_receive(&error) => {}
                        Err(error) => return Err(error),
                    }
                }
            }

            async fn try_recv(&self) -> Result<(MavHeader, $message), MessageReadError> {
                let $core = self;
                let dialect = $dialect;
                if $core.transport.try_recv_is_nonblocking() {
                    let mut reader = $core.transport.reader().try_lock().ok_or_else(|| {
                        io::Error::from(io::ErrorKind::WouldBlock)
                    })?;
                    crate::connection_shared::read_message_async_with_dialect(
                        &mut reader,
                        &$core.state,
                        dialect,
                    )
                    .now_or_never()
                    .unwrap_or_else(|| Err(io::Error::from(io::ErrorKind::WouldBlock).into()))
                } else {
                    let mut reader = $core.transport.reader().lock().await;
                    crate::connection_shared::read_message_async_with_dialect(
                        &mut reader,
                        &$core.state,
                        dialect,
                    )
                    .await
                }
            }

            async fn send(&self, header: &MavHeader, data: &$message) -> Result<usize, MessageWriteError> {
                let $core = self;
                let dialect = $dialect;
                let Some(writer) = $core.transport.writer() else {
                    return Ok(0);
                };
                let mut writer = writer.lock().await;
                if !$core.transport.can_send(&writer) {
                    return Ok(0);
                }
                let header = $core.transport.next_send_header(&mut writer, header);
                crate::connection_shared::write_message_async_with_dialect(
                    &mut *writer,
                    &$core.state,
                    header,
                    dialect,
                    data,
                )
                .await
            }

            async fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError> {
                let Some(writer) = self.transport.writer() else {
                    return Ok(0);
                };
                let mut writer = writer.lock().await;
                if !self.transport.can_send(&writer) {
                    return Ok(0);
                }
                crate::connection_shared::write_raw_message_async(&mut *writer, data).await
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

impl_async_mav_connection!([T: AsyncTransport, D: Dialect + Send + Sync] [D::Message: Send + Sync] AsyncConnectionCore<T, D>, D::Message, |core| &core.dialect);
impl_async_mav_connection!([T: AsyncTransport, M: Message + Send + Sync] [M: Message + Send + Sync] AsyncConnectionCore<T>, M, |core| &StaticDialect::<M>::new());

pub type AsyncDialectConnection<D> =
    Box<dyn AsyncMavConnection<<D as Dialect>::Message> + Send + Sync>;

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
///
/// # Errors
///
/// - [`AddrNotAvailable`] if the address string could not be parsed as a valid MAVLink address
/// - When the connection could not be established a corresponding [`io::Error`] is returned
///
/// [`AddrNotAvailable`]: io::ErrorKind::AddrNotAvailable
pub async fn connect_async<M: Message + Sync + Send>(
    address: &str,
) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>> {
    ConnectionAddress::parse_address(address)?
        .connect_async()
        .await
}

/// Connect asynchronously to a MAVLink node using a runtime-loaded dialect.
///
/// The accepted address formats and errors are the same as [`connect_async`].
pub async fn connect_async_with_dialect<D: Dialect + Send + Sync + 'static>(
    address: &str,
    dialect: D,
) -> io::Result<AsyncDialectConnection<D>>
where
    D::Message: Send + Sync,
{
    ConnectionAddress::parse_address(address)?
        .connect_async_with_dialect(dialect)
        .await
}

/// A MAVLink connection address that can be connected to, establishing an [`AsyncMavConnection`]
///
/// This is the `async` version of `Connectable`.
#[async_trait]
pub trait AsyncConnectable {
    /// Attempt to establish an asynchronous MAVLink connection
    async fn connect_async<M: Message + Sync + Send>(
        &self,
    ) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>;
}

/// A MAVLink connection address that can be connected to using a runtime-loaded dialect.
#[async_trait]
pub trait AsyncDialectConnectable {
    /// Attempt to establish an asynchronous MAVLink connection with a runtime-loaded dialect.
    async fn connect_async_with_dialect<D: Dialect + Send + Sync + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D::Message: Send + Sync;
}

#[async_trait]
impl AsyncConnectable for ConnectionAddress {
    async fn connect_async<M: Message + Sync + Send>(
        &self,
    ) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>> {
        match self {
            #[cfg(feature = "transport-tcp")]
            Self::Tcp(c) => c.connect_async().await,
            #[cfg(feature = "transport-udp")]
            Self::Udp(c) => c.connect_async().await,
            #[cfg(feature = "transport-direct-serial")]
            Self::Serial(c) => c.connect_async().await,
            Self::File(c) => c.connect_async().await,
        }
    }
}

#[async_trait]
impl AsyncDialectConnectable for ConnectionAddress {
    async fn connect_async_with_dialect<D: Dialect + Send + Sync + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D::Message: Send + Sync,
    {
        match self {
            #[cfg(feature = "transport-tcp")]
            Self::Tcp(c) => c.connect_async_with_dialect(dialect).await,
            #[cfg(feature = "transport-udp")]
            Self::Udp(c) => c.connect_async_with_dialect(dialect).await,
            #[cfg(feature = "transport-direct-serial")]
            Self::Serial(c) => c.connect_async_with_dialect(dialect).await,
            Self::File(c) => c.connect_async_with_dialect(dialect).await,
        }
    }
}
