use core::fmt::Display;
use std::io;

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
pub struct UdpConnectable {
    pub(crate) address: String,
    pub(crate) mode: UdpMode,
}

impl UdpConnectable {
    /// Creates a UDP connection address.
    ///
    /// The type of connection depends on the [`UdpMode`]
    pub fn new(address: String, mode: UdpMode) -> Self {
        Self { address, mode }
    }
}
impl Display for UdpConnectable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mode = match self.mode {
            UdpMode::Udpin => "udpin",
            UdpMode::Udpout => "udpout",
            UdpMode::Udpcast => "udpcast",
        };
        write!(f, "{mode}:{}", self.address)
    }
}

/// MAVLink address for a serial connection
#[derive(Debug, Clone)]
pub struct SerialConnectable {
    pub(crate) port_name: String,
    pub(crate) baud_rate: u32,
}

impl SerialConnectable {
    /// Creates a serial connection address with port name and baud rate.
    pub fn new(port_name: String, baud_rate: u32) -> Self {
        Self {
            port_name,
            baud_rate,
        }
    }
}
impl Display for SerialConnectable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "serial:{}:{}", self.port_name, self.baud_rate)
    }
}

/// MAVLink connection address for a TCP server or client
#[derive(Debug, Clone)]
pub struct TcpConnectable {
    pub(crate) address: String,
    pub(crate) is_out: bool,
}

impl TcpConnectable {
    /// Creates a TCP connection address.
    ///
    /// If `is_out` is `true` the connection will open a TCP server that binds to the provided address.
    /// If `is_out` is `false` the connection will connect to the provided TCP server address.
    pub fn new(address: String, is_out: bool) -> Self {
        Self { address, is_out }
    }
}
impl Display for TcpConnectable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.is_out {
            write!(f, "tcpout:{}", self.address)
        } else {
            write!(f, "tcpin:{}", self.address)
        }
    }
}

/// MAVLink connection address for a file input
#[derive(Debug, Clone)]
pub struct FileConnectable {
    pub(crate) address: String,
}

impl FileConnectable {
    /// Creates a file input address from a file path string.
    pub fn new(address: String) -> Self {
        Self { address }
    }
}
impl Display for FileConnectable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "file:{}", self.address)
    }
}

/// A parsed MAVLink connection address
pub enum ConnectionAddress {
    /// TCP client or server address
    #[cfg(feature = "tcp")]
    Tcp(TcpConnectable),
    /// UDP client, server or broadcast address
    #[cfg(feature = "udp")]
    Udp(UdpConnectable),
    /// Serial port address
    #[cfg(feature = "direct-serial")]
    Serial(SerialConnectable),
    /// File input address
    File(FileConnectable),
}

impl Display for ConnectionAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "tcp")]
            Self::Tcp(connectable) => write!(f, "{connectable}"),
            #[cfg(feature = "udp")]
            Self::Udp(connectable) => write!(f, "{connectable}"),
            #[cfg(feature = "direct-serial")]
            Self::Serial(connectable) => write!(f, "{connectable}"),
            Self::File(connectable) => write!(f, "{connectable}"),
        }
    }
}

impl ConnectionAddress {
    /// Parse a MAVLink address string.
    ///
    ///  The address must be in one of the following formats:
    ///
    ///  * `tcpin:<addr>:<port>` to create a TCP server, listening for an incoming connection
    ///  * `tcpout:<addr>:<port>` to create a TCP client
    ///  * `udpin:<addr>:<port>` to create a UDP server, listening for incoming packets
    ///  * `udpout:<addr>:<port>` to create a UDP client
    ///  * `udpbcast:<addr>:<port>` to create a UDP broadcast
    ///  * `serial:<port>:<baudrate>` to create a serial connection
    ///  * `file:<path>` to extract file data, writing to such a connection does nothing
    pub fn parse_address(address: &str) -> Result<Self, io::Error> {
        let (protocol, address) = address.split_once(':').ok_or(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Protocol unsupported",
        ))?;
        let conn = match protocol {
            #[cfg(feature = "direct-serial")]
            "serial" => {
                let (port_name, baud) = address.split_once(':').ok_or(io::Error::new(
                    io::ErrorKind::AddrNotAvailable,
                    "Incomplete port settings",
                ))?;
                Self::Serial(SerialConnectable::new(
                    port_name.to_string(),
                    baud.parse().map_err(|_| {
                        io::Error::new(io::ErrorKind::AddrNotAvailable, "Invalid baud rate")
                    })?,
                ))
            }
            #[cfg(feature = "tcp")]
            "tcpin" | "tcpout" => Self::Tcp(TcpConnectable::new(
                address.to_string(),
                protocol == "tcpout",
            )),
            #[cfg(feature = "udp")]
            "udpin" | "udpout" | "udpcast" => Self::Udp(UdpConnectable::new(
                address.to_string(),
                match protocol {
                    "udpin" => UdpMode::Udpin,
                    "udpout" => UdpMode::Udpout,
                    "udpcast" => UdpMode::Udpcast,
                    _ => unreachable!(),
                },
            )),
            "file" => Self::File(FileConnectable::new(address.to_string())),
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::AddrNotAvailable,
                    "Protocol unsupported",
                ))
            }
        };
        Ok(conn)
    }
}
