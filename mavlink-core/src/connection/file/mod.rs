pub mod config;
mod sync;
pub(crate) use sync::FileConnection;

#[cfg(feature = "tokio")]
mod r#async;

#[cfg(feature = "tokio")]
pub(crate) use r#async::AsyncFileConnection;
