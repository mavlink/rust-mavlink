use core::fmt::Display;

/// MAVLink connection address for a TCP server or client
#[derive(Debug, Clone)]
pub struct TcpConfig {
    pub(crate) address: String,
    pub(crate) is_out: bool,
}

impl TcpConfig {
    /// Creates a TCP connection address.
    ///
    /// If `is_out` is `true` the connection will open a TCP server that binds to the provided address.
    /// If `is_out` is `false` the connection will connect to the provided TCP server address.
    pub fn new(address: String, is_out: bool) -> Self {
        Self { address, is_out }
    }
}
impl Display for TcpConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.is_out {
            write!(f, "tcpout:{}", self.address)
        } else {
            write!(f, "tcpin:{}", self.address)
        }
    }
}
