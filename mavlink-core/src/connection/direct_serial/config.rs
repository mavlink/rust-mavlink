use core::fmt::Display;

/// MAVLink address for a serial connection
///
/// # Example
///
/// ```ignore
/// use mavlink::{Connectable, SerialConfig};
///
/// let config = SerialConfig::new("/dev/ttyTHS1".to_owned(), 115200);
/// config.connect::<mavlink::ardupilotmega::MavMessage>();
/// ```
#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub(crate) port_name: String,
    pub(crate) baud_rate: u32,
    read_buffer_capacity: usize,
}

impl SerialConfig {
    /// Creates a serial connection address with port name and baud rate.
    pub fn new(port_name: String, baud_rate: u32) -> Self {
        // Calculate a sane default buffer capacity based on the baud rate.
        let default_capacity = (baud_rate / 100).clamp(1024, 1024 * 8) as usize;

        Self {
            port_name,
            baud_rate,
            read_buffer_capacity: default_capacity,
        }
    }

    /// Updates the read buffer capacity.
    pub fn with_read_buffer_capacity(mut self, capacity: usize) -> Self {
        self.read_buffer_capacity = capacity;
        self
    }

    /// Returns the configured read buffer capacity.
    pub fn buffer_capacity(&self) -> usize {
        self.read_buffer_capacity
    }
}

impl Display for SerialConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "serial:{}:{}", self.port_name, self.baud_rate)
    }
}
