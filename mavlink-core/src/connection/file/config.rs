use core::fmt::Display;

/// MAVLink connection address for a file input
#[derive(Debug, Clone)]
pub struct FileConfig {
    pub(crate) address: String,
}

impl FileConfig {
    /// Creates a file input address from a file path string.
    pub fn new(address: String) -> Self {
        Self { address }
    }
}
impl Display for FileConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "file:{}", self.address)
    }
}
