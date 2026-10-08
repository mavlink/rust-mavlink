use core::fmt::Display;
use std::io;
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};

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
    pub(crate) source: TcpSource,
}

#[derive(Debug, Clone)]
pub(crate) enum TcpSource {
    Address,
    Stream(Arc<Mutex<Option<TcpStream>>>),
    Listener(Arc<Mutex<Option<TcpListener>>>),
}

impl TcpConfig {
    /// Creates a TCP connection address.
    pub fn new(address: String, mode: TcpMode) -> Self {
        Self {
            address,
            mode,
            source: TcpSource::Address,
        }
    }

    /// Creates a TCP client configuration from an already connected stream.
    ///
    /// Socket-backed configurations are one-shot, including across clones.
    pub fn from_stream(stream: TcpStream) -> io::Result<Self> {
        let address = stream.peer_addr()?.to_string();
        Ok(Self {
            address,
            mode: TcpMode::TcpOut,
            source: TcpSource::Stream(Arc::new(Mutex::new(Some(stream)))),
        })
    }

    /// Creates a TCP server configuration from an already bound listener.
    ///
    /// Socket-backed configurations are one-shot, including across clones.
    pub fn from_listener(listener: TcpListener) -> io::Result<Self> {
        let address = listener.local_addr()?.to_string();
        Ok(Self {
            address,
            mode: TcpMode::TcpIn,
            source: TcpSource::Listener(Arc::new(Mutex::new(Some(listener)))),
        })
    }

    pub(crate) fn take_stream(&self) -> io::Result<Option<TcpStream>> {
        match &self.source {
            TcpSource::Address => Ok(None),
            TcpSource::Stream(stream) => stream
                .lock()
                .map_err(|_| io::Error::other("TCP stream lock poisoned"))?
                .take()
                .map(Some)
                .ok_or_else(|| io::Error::other("TCP socket-backed configuration already used")),
            TcpSource::Listener(_) => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "TCP listener cannot be used as an outgoing connection",
            )),
        }
    }

    pub(crate) fn take_listener(&self) -> io::Result<Option<TcpListener>> {
        match &self.source {
            TcpSource::Address => Ok(None),
            TcpSource::Listener(listener) => listener
                .lock()
                .map_err(|_| io::Error::other("TCP listener lock poisoned"))?
                .take()
                .map(Some)
                .ok_or_else(|| io::Error::other("TCP socket-backed configuration already used")),
            TcpSource::Stream(_) => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "TCP stream cannot be used as an incoming connection",
            )),
        }
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
