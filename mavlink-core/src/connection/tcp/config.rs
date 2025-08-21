use core::fmt::Display;

/// Type of TCP connection
#[derive(Debug, Clone, Copy)]
pub enum TcpMode {
    /// Connection will open a TCP server that binds to the provided address
    TcpIn,
    /// Connection will connect to the provided TCP server address
    TcpOut,
}

/// MAVLink connection address for a TCP server or client
///
/// # Example
///
/// ```ignore
/// use mavlink::{Connectable, TcpConfig, TcpMode};
///
/// let config = TcpConfig::new("0.0.0.0:14551".to_owned(), false);
/// config.connect::<mavlink::ardupilotmega::MavMessage>();
/// ```
#[derive(Debug, Clone)]
pub struct TcpConfig {
    pub(crate) address: String,
    pub(crate) mode: TcpMode,
}

impl TcpConfig {
    /// Creates a TCP connection address.
    pub fn new(address: String, mode: TcpMode) -> Self {
        Self { address, mode }
    }
}
impl Display for TcpConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.mode {
            TcpMode::TcpIn => write!(f, "tcpin:{}", self.address),
            TcpMode::TcpOut => write!(f, "tcpout:{}", self.address),
        }
    }
}
