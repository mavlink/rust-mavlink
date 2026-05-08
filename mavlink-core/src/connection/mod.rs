mod sync;

#[cfg(feature = "transport-tcp")]
pub mod tcp;

#[cfg(feature = "transport-udp")]
pub mod udp;

#[cfg(feature = "transport-direct-serial")]
pub mod direct_serial;

pub mod file;

use std::io;
pub use sync::{Connectable, Connection, MavConnection, connect};

/// Returns the socket address for the given address.
#[cfg(any(feature = "transport-tcp", feature = "transport-udp"))]
pub(crate) fn get_socket_addr<T: std::net::ToSocketAddrs>(
    address: &T,
) -> Result<std::net::SocketAddr, io::Error> {
    address
        .to_socket_addrs()?
        .next()
        .ok_or(io::Error::other("Host address lookup failed"))
}

#[cfg(feature = "tokio")]
mod r#async;
#[cfg(feature = "tokio")]
pub use r#async::{AsyncConnectable, AsyncMavConnection, connect_async};
