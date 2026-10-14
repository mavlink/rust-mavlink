use std::{future::Future, io};

use futures::lock::{Mutex, MutexGuard};
use tokio::io::{AsyncRead, AsyncWrite};

use crate::AsyncMavlinkReader;
#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;
use crate::error::{MessageReadError, MessageWriteError};
use crate::static_dialect::StaticDialect;
use crate::{
    Dialect, MAVLinkMessageRaw, MavFrame, MavHeader, MavlinkVersion, Message,
    connectable::ConnectionAddress,
};

/// The byte-oriented half of a Tokio transport.
#[doc(hidden)]
pub(crate) trait AsyncTransport: Send + Sync {
    /// Incoming bytes.
    type Reader: AsyncRead + Unpin + Send;
    /// Outgoing bytes.
    type Writer: AsyncWrite + Unpin + Send;

    /// Returns the reader mutex.
    fn reader(&self) -> &Mutex<AsyncMavlinkReader<Self::Reader>>;

    /// Locks the writer. Returns `None` to skip sending when there is no writer or peer.
    fn lock_writer(&self) -> impl Future<Output = Option<MutexGuard<'_, Self::Writer>>> + Send;

    /// Whether `recv` and `recv_raw` should retry an error. Unused by `try_recv`.
    fn retry_receive(&self, _error: &MessageReadError) -> bool {
        false
    }
}

/// A concrete async connection using generated messages.
pub type AsyncConnection<M> = AsyncDialectConnection<StaticDialect<M>>;

/// A concrete async connection using a dialect.
pub struct AsyncDialectConnection<D> {
    transport: Transport,
    dialect: D,
    state: crate::connection_shared::ConnectionState,
}

pub(crate) enum Transport {
    #[cfg(feature = "transport-tcp")]
    Tcp(super::tcp::AsyncTcpConnection),
    #[cfg(feature = "transport-udp")]
    Udp(super::udp::AsyncUdpConnection),
    #[cfg(feature = "transport-direct-serial")]
    Serial(super::direct_serial::AsyncSerialConnection),
    File(super::file::AsyncFileConnection),
}

impl<D> AsyncDialectConnection<D> {
    pub(crate) fn new(transport: Transport, dialect: D) -> Self {
        Self {
            transport,
            dialect,
            state: crate::connection_shared::ConnectionState::new(),
        }
    }
}

/// An async MAVLink connection
pub trait AsyncMavConnection<M: Sync + Send>: Send + Sync {
    /// Receive a mavlink message.
    ///
    /// Yield until a valid frame is received, ignoring invalid messages.
    fn recv(&self) -> impl Future<Output = Result<(MavHeader, M), MessageReadError>> + Send;

    /// Receive a raw, unparsed mavlink message.
    ///
    /// Yield until a valid frame is received, ignoring invalid messages.
    fn recv_raw(&self) -> impl Future<Output = Result<MAVLinkMessageRaw, MessageReadError>> + Send;

    /// Receive a MAVLink message without retrying read or parse errors.
    ///
    /// Waits for the reader lock and incoming data.
    fn try_recv(&self) -> impl Future<Output = Result<(MavHeader, M), MessageReadError>> + Send;

    /// Send a mavlink message
    fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> impl Future<Output = Result<usize, MessageWriteError>> + Send;

    /// Send a raw, unparsed mavlink message
    fn send_raw(
        &self,
        data: &MAVLinkMessageRaw,
    ) -> impl Future<Output = Result<usize, MessageWriteError>> + Send;

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
    fn send_frame(
        &self,
        frame: &MavFrame<M>,
    ) -> impl Future<Output = Result<usize, MessageWriteError>> + Send
    where
        M: Message,
    {
        self.send(&frame.header, &frame.msg)
    }

    /// Read whole frame.
    fn recv_frame(&self) -> impl Future<Output = Result<MavFrame<M>, MessageReadError>> + Send
    where
        M: Message,
    {
        async move {
            let (header, msg) = self.recv().await?;
            Ok(MavFrame {
                header,
                msg,
                protocol_version: self.protocol_version(),
            })
        }
    }

    /// Send a message with default header.
    fn send_default(
        &self,
        data: &M,
    ) -> impl Future<Output = Result<usize, MessageWriteError>> + Send {
        async move { self.send(&MavHeader::default(), data).await }
    }

    /// Setup secret key used for message signing, or disable message signing.
    #[cfg(feature = "mav2-message-signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>);
}

impl<D: Dialect + Send + Sync> AsyncMavConnection<D::Message> for AsyncDialectConnection<D>
where
    D::Message: Send + Sync,
{
    async fn recv(&self) -> Result<(MavHeader, D::Message), MessageReadError> {
        dispatch_transport!(&self.transport, |transport| {
            let mut reader = transport.reader().lock().await;
            loop {
                match crate::connection_shared::read_message_async_with_dialect(
                    &mut reader,
                    &self.state,
                    &self.dialect,
                )
                .await
                {
                    Ok(message) => return Ok(message),
                    Err(error) if transport.retry_receive(&error) => {}
                    Err(error) => return Err(error),
                }
            }
        })
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        dispatch_transport!(&self.transport, |transport| {
            let mut reader = transport.reader().lock().await;
            loop {
                match crate::connection_shared::read_raw_message_async_with_dialect(
                    &mut reader,
                    &self.state,
                    &self.dialect,
                )
                .await
                {
                    Ok(message) => return Ok(message),
                    Err(error) if transport.retry_receive(&error) => {}
                    Err(error) => return Err(error),
                }
            }
        })
    }

    async fn try_recv(&self) -> Result<(MavHeader, D::Message), MessageReadError> {
        dispatch_transport!(&self.transport, |transport| {
            let mut reader = transport.reader().lock().await;
            crate::connection_shared::read_message_async_with_dialect(
                &mut reader,
                &self.state,
                &self.dialect,
            )
            .await
        })
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &D::Message,
    ) -> Result<usize, MessageWriteError> {
        dispatch_transport!(&self.transport, |transport| {
            let Some(mut writer) = transport.lock_writer().await else {
                return Ok(0);
            };
            let header = self.state.next_header(header);
            crate::connection_shared::write_message_async_with_dialect(
                &mut *writer,
                &self.state,
                header,
                &self.dialect,
                data,
            )
            .await
        })
    }

    async fn send_raw(&self, data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError> {
        dispatch_transport!(&self.transport, |transport| {
            let Some(mut writer) = transport.lock_writer().await else {
                return Ok(0);
            };
            crate::connection_shared::write_raw_message_async(&mut *writer, data).await
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
/// The address selects the transport stored in the returned connection.
///
/// # Errors
///
/// - [`AddrNotAvailable`] if the address string could not be parsed as a valid MAVLink address
/// - When the connection could not be established a corresponding [`io::Error`] is returned
///
/// [`AddrNotAvailable`]: io::ErrorKind::AddrNotAvailable
pub async fn connect_async<M: Message + Sync + Send>(
    address: &str,
) -> io::Result<AsyncConnection<M>> {
    ConnectionAddress::parse_address(address)?
        .connect_async()
        .await
}

/// Connect asynchronously to a MAVLink node using a runtime-loaded dialect.
///
/// The accepted address formats and errors are the same as [`connect_async`].
pub async fn connect_async_with_dialect<D: Dialect + Send + Sync>(
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
pub trait AsyncConnectable: Sync {
    /// Attempt to establish an asynchronous MAVLink connection
    fn connect_async<M: Message + Sync + Send>(
        &self,
    ) -> impl Future<Output = io::Result<AsyncConnection<M>>> + Send {
        self.connect_async_with_dialect(StaticDialect::<M>::new())
    }

    /// Attempt to establish an asynchronous MAVLink connection with a runtime-loaded dialect.
    fn connect_async_with_dialect<D: Dialect + Send + Sync>(
        &self,
        dialect: D,
    ) -> impl Future<Output = io::Result<AsyncDialectConnection<D>>> + Send
    where
        D::Message: Send + Sync;
}

impl AsyncConnectable for ConnectionAddress {
    async fn connect_async_with_dialect<D: Dialect + Send + Sync>(
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
