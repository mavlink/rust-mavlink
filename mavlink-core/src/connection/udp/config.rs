use core::fmt::Display;
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
    Udpcast,
}

/// MAVLink address for a UDP server client or broadcast connection
#[derive(Debug, Clone)]
pub struct UdpConfig {
    pub(crate) address: String,
    pub(crate) mode: UdpMode,
    pub(crate) read_timeout: Option<Duration>,
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
            UdpMode::Udpcast => "udpcast",
        };
        write!(f, "{mode}:{}", self.address)
    }
}
