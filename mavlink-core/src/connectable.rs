use core::fmt::Display;
use std::io;

#[derive(Debug, Clone, Copy)]
pub enum UdpMode {
    Udpin,
    Udpout,
    Udpcast,
}

#[derive(Debug, Clone)]
pub struct UdpConnectable {
    pub(crate) address: String,
    pub(crate) mode: UdpMode,
}

impl UdpConnectable {
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

#[derive(Debug, Clone)]
pub struct SerialConnectable {
    pub(crate) port_name: String,
    pub(crate) baud_rate: usize,
}

impl SerialConnectable {
    pub fn new(port_name: String, baud_rate: usize) -> Self {
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

#[derive(Debug, Clone)]
pub struct TcpConnectable {
    pub(crate) address: String,
    pub(crate) is_out: bool,
}

impl TcpConnectable {
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

#[derive(Debug, Clone)]
pub struct FileConnectable {
    pub(crate) address: String,
}

impl FileConnectable {
    pub fn new(address: String) -> Self {
        Self { address }
    }
}
impl Display for FileConnectable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "file:{}", self.address)
    }
}
pub enum ConnectionAddress {
    Tcp(TcpConnectable),
    Udp(UdpConnectable),
    Serial(SerialConnectable),
    File(FileConnectable),
}

impl Display for ConnectionAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Tcp(connectable) => write!(f, "{connectable}"),
            Self::Udp(connectable) => write!(f, "{connectable}"),
            Self::Serial(connectable) => write!(f, "{connectable}"),
            Self::File(connectable) => write!(f, "{connectable}"),
        }
    }
}

impl ConnectionAddress {
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
