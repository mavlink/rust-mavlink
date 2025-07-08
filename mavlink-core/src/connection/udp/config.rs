use core::fmt::Display;

/// Type of UDP connection
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
}

impl UdpConfig {
    /// Creates a UDP connection address.
    ///
    /// The type of connection depends on the [`UdpMode`]
    pub fn new(address: String, mode: UdpMode) -> Self {
        Self { address, mode }
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
