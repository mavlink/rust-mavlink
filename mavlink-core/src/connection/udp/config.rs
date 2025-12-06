use core::fmt::Display;
use std::sync::Arc;

#[cfg(all(feature = "udp", not(feature = "tokio-1")))]
use std::net::UdpSocket;
#[cfg(all(feature = "udp", feature = "tokio-1"))]
use tokio::net::UdpSocket;

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
pub struct UdpConfig<T> {
    pub address: Arc<T>,
    pub(crate) mode: UdpMode,
    pub(crate) target: Option<String>,
}

impl UdpConfig<UdpSocket> {
    /// Creates a UDP connection address.
    ///
    /// The type of connection depends on the [`UdpMode`]
    pub fn new(address: &str, mode: UdpMode, target: Option<String>) -> Self {
        let address = std::net::UdpSocket::bind(address).expect("Unable to bind UDP socket");

        #[cfg(all(feature = "udp", feature = "tokio-1"))]
        let address = tokio::net::UdpSocket::from_std(address).expect("Unable to bind UDP socket");

        Self {
            address: Arc::new(address),
            mode,
            target,
        }
    }
}

impl Display for UdpConfig<UdpSocket> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mode = match self.mode {
            UdpMode::Udpin => "udpin",
            UdpMode::Udpout => "udpout",
            UdpMode::Udpcast => "udpcast",
        };
        let address = match self.address.local_addr() {
            Ok(addr) => addr.to_string(),
            Err(_) => "<invalid address>".to_string(),
        };
        write!(f, "{mode}:{address}")
    }
}
