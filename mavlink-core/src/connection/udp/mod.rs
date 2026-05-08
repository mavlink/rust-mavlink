pub mod config;
mod sync;

#[cfg(feature = "tokio")]
mod r#async;

pub use sync::UdpConnection;
