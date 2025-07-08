use core::fmt::Display;

/// MAVLink address for a serial connection
#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub(crate) port_name: String,
    pub(crate) baud_rate: u32,
}

impl SerialConfig {
    /// Creates a serial connection address with port name and baud rate.
    pub fn new(port_name: String, baud_rate: u32) -> Self {
        Self {
            port_name,
            baud_rate,
        }
    }
}

impl Display for SerialConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "serial:{}:{}", self.port_name, self.baud_rate)
    }
}
