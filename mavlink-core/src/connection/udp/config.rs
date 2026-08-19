use core::fmt::Display;
use std::io;
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Type of UDP connection
///
/// # Example
///
/// ```ignore
/// use mavlink::{Connectable, UdpConfig, UdpMode};
///
/// let config = mavlink::UdpConfig::new("0.0.0.0:14552".to_owned(), UdpMode::Udpin);
/// config
///     .connect::<mavlink::ardupilotmega::MavMessage>()
///     .unwrap();
/// ```
#[derive(Debug, Clone, Copy)]
pub enum UdpMode {
    /// Server connection waiting for a client connection
    Udpin,
    /// Client connection connecting to a server
    Udpout,
    /// Client connection that is allowed to send to broadcast addresses
    UdpBroadcast,
}

/// MAVLink address for a UDP server client or broadcast connection
#[derive(Debug, Clone)]
pub struct UdpConfig {
    pub(crate) address: String,
    pub(crate) mode: UdpMode,
    pub(crate) read_timeout: Option<Duration>,
    pub(crate) source: UdpSource,
}

#[derive(Debug, Clone)]
pub(crate) enum UdpSource {
    Address,
    Socket(Arc<Mutex<Option<UdpSocket>>>),
}

impl UdpConfig {
    /// Creates a UDP connection address.
    ///
    /// The type of connection depends on the [`UdpMode`]
    pub fn new(address: String, mode: UdpMode) -> Self {
        Self {
            address,
            mode,
            read_timeout: None,
            source: UdpSource::Address,
        }
    }

    /// Creates a UDP configuration from an existing socket.
    ///
    /// Input sockets must be bound. Output and broadcast sockets must also be
    /// connected; their peer address is used as the destination. Socket-backed
    /// configurations are one-shot, including across cloned configurations.
    pub fn from_socket(socket: UdpSocket, mode: UdpMode) -> io::Result<Self> {
        let address = match mode {
            UdpMode::Udpin => socket.local_addr()?,
            UdpMode::Udpout | UdpMode::UdpBroadcast => socket.peer_addr()?,
        };

        Ok(Self {
            address: address.to_string(),
            mode,
            read_timeout: None,
            source: UdpSource::Socket(Arc::new(Mutex::new(Some(socket)))),
        })
    }

    pub(crate) fn take_socket(&self) -> io::Result<Option<UdpSocket>> {
        match &self.source {
            UdpSource::Address => Ok(None),
            UdpSource::Socket(socket) => socket
                .lock()
                .map_err(|_| io::Error::other("UDP socket lock poisoned"))?
                .take()
                .map(Some)
                .ok_or_else(|| io::Error::other("UDP socket-backed configuration already used")),
        }
    }

    /// Sets the read timeout on the UDP socket.
    ///
    /// When set, `recv()` and `recv_raw()` will return an error after the
    /// specified duration instead of blocking indefinitely. This is useful
    /// for implementing graceful shutdown.
    pub fn read_timeout(mut self, timeout: Duration) -> Self {
        self.read_timeout = Some(timeout);
        self
    }
}

impl Display for UdpConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mode = match self.mode {
            UdpMode::Udpin => "udpin",
            UdpMode::Udpout => "udpout",
            UdpMode::UdpBroadcast => "udpbcast",
        };
        write!(f, "{mode}:{}", self.address)
    }
}
