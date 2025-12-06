use core::fmt::Display;
use std::io;
#[cfg(all(feature = "udp", not(feature = "tokio-1")))]
use std::net::UdpSocket;
use std::path::PathBuf;
#[cfg(all(feature = "udp", feature = "tokio-1"))]
use tokio::net::UdpSocket;

#[cfg(feature = "direct-serial")]
use crate::connection::direct_serial::config::SerialConfig;
use crate::connection::file::config::FileConfig;
#[cfg(feature = "tcp")]
use crate::connection::tcp::config::{TcpConfig, TcpMode};
#[cfg(feature = "udp")]
use crate::connection::udp::config::{UdpConfig, UdpMode};

/// A parsed MAVLink connection address
pub enum ConnectionAddress {
    /// TCP client or server address
    #[cfg(feature = "tcp")]
    Tcp(TcpConfig),
    /// UDP client, server or broadcast address and not tokio-1
    #[cfg(all(feature = "udp", not(feature = "tokio-1")))]
    Udp(UdpConfig<UdpSocket>),
    /// UDP client, server or broadcast address and tokio-1
    #[cfg(all(feature = "udp", feature = "tokio-1"))]
    Udp(UdpConfig<UdpSocket>),
    /// Serial port address
    #[cfg(feature = "direct-serial")]
    Serial(SerialConfig),
    /// File input address
    File(FileConfig),
}

#[cfg(feature = "tcp")]
impl From<TcpConfig> for ConnectionAddress {
    fn from(value: TcpConfig) -> Self {
        Self::Tcp(value)
    }
}

#[cfg(all(feature = "udp", not(feature = "tokio-1")))]
impl From<UdpConfig<UdpSocket>> for ConnectionAddress {
    fn from(value: UdpConfig<UdpSocket>) -> Self {
        Self::Udp(value)
    }
}

#[cfg(all(feature = "udp", feature = "tokio-1"))]
impl From<UdpConfig<tokio::net::UdpSocket>> for ConnectionAddress {
    fn from(value: UdpConfig<tokio::net::UdpSocket>) -> Self {
        Self::Udp(value)
    }
}

#[cfg(feature = "direct-serial")]
impl From<SerialConfig> for ConnectionAddress {
    fn from(value: SerialConfig) -> Self {
        Self::Serial(value)
    }
}

impl From<FileConfig> for ConnectionAddress {
    fn from(value: FileConfig) -> Self {
        Self::File(value)
    }
}

impl Display for ConnectionAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "tcp")]
            Self::Tcp(connectable) => write!(f, "{connectable}"),
            #[cfg(feature = "udp")]
            Self::Udp(connectable) => write!(f, "{connectable:?}"),
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
    ///
    /// # Errors
    ///
    /// - [`AddrNotAvailable`] if the address string could not be parsed as a valid MAVLink address
    ///
    /// [`AddrNotAvailable`]: io::ErrorKind::AddrNotAvailable
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
                Self::Serial(SerialConfig::new(
                    port_name.to_string(),
                    baud.parse().map_err(|_| {
                        io::Error::new(io::ErrorKind::AddrNotAvailable, "Invalid baud rate")
                    })?,
                ))
            }
            #[cfg(feature = "tcp")]
            "tcpin" | "tcpout" => {
                let mode = if protocol == "tcpout" {
                    TcpMode::TcpOut
                } else {
                    TcpMode::TcpIn
                };
                Self::Tcp(TcpConfig::new(address.to_string(), mode))
            }
            #[cfg(all(feature = "udp"))]
            "udpin" | "udpout" | "udpcast" => Self::Udp(UdpConfig::new(
                match protocol {
                    "udpout" => address,
                    _ => "0.0.0.0:0",
                },
                match protocol {
                    "udpin" => UdpMode::Udpin,
                    "udpout" => UdpMode::Udpout,
                    "udpcast" => UdpMode::Udpcast,
                    _ => unreachable!(),
                },
                match protocol {
                    "udpin" | "udpcast" => Some(address.to_string()),
                    _ => None,
                },
            )),
            "file" => Self::File(FileConfig::new(PathBuf::from(address))),
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
