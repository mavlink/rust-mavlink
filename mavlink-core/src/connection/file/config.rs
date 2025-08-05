use core::fmt::Display;
use std::path::PathBuf;

/// MAVLink connection address for a file input
///
/// # Example
///
/// ```ignore
/// use mavlink::{Connectable, FileConfig};
/// use std::path::PathBuf;
///
/// let config = FileConfig::new(PathBuf::from("/some/path"));
/// config
///   .connect::<mavlink::ardupilotmega::MavMessage>()
///   .unwrap();
/// ```
#[derive(Debug, Clone)]
pub struct FileConfig {
    pub(crate) address: PathBuf,
}

impl FileConfig {
    /// Creates a file input address from a file path string.
    pub fn new(address: PathBuf) -> Self {
        Self { address }
    }
}
impl Display for FileConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "file:{}", self.address.display())
    }
}
