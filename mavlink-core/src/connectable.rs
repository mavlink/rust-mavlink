use core::fmt::Display;
use std::io;

use crate::connection::direct_serial::config::SerialConfig;
use crate::connection::file::config::FileConfig;
use crate::connection::tcp::config::TcpConfig;
use crate::connection::udp::config::{UdpConfig, UdpMode};

/// A parsed MAVLink connection address
pub enum ConnectionAddress {
    /// TCP client or server address
    #[cfg(feature = "tcp")]
    Tcp(TcpConfig),
    /// UDP client, server or broadcast address
    #[cfg(feature = "udp")]
    Udp(UdpConfig),
    /// Serial port address
    #[cfg(feature = "direct-serial")]
    Serial(SerialConfig),
    /// File input address
    File(FileConfig),
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
                Self::Serial(SerialConfig::new(
                    port_name.to_string(),
                    baud.parse().map_err(|_| {
                        io::Error::new(io::ErrorKind::AddrNotAvailable, "Invalid baud rate")
                    })?,
                ))
            }
            #[cfg(feature = "tcp")]
            "tcpin" | "tcpout" => {
                Self::Tcp(TcpConfig::new(address.to_string(), protocol == "tcpout"))
            }
            #[cfg(feature = "udp")]
            "udpin" | "udpout" | "udpcast" => Self::Udp(UdpConfig::new(
                address.to_string(),
                match protocol {
                    "udpin" => UdpMode::Udpin,
                    "udpout" => UdpMode::Udpout,
                    "udpcast" => UdpMode::Udpcast,
                    _ => unreachable!(),
                },
            )),
            "file" => Self::File(FileConfig::new(address.to_string())),
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
