//! Async File MAVLINK connection

use std::io;
use std::path::PathBuf;

use async_trait::async_trait;
use futures::lock::{Mutex, MutexGuard};
use tokio::fs::File;
use tokio::io::Sink;

use crate::AsyncMavlinkReader;
use crate::Dialect;
use crate::connection::r#async::{
    AsyncConnectable, AsyncConnectionCore, AsyncDialectConnectable, AsyncDialectConnection,
    AsyncMavConnection, AsyncTransport,
};
use crate::connection::file::config::FileConfig;

pub async fn open(file_path: &PathBuf) -> io::Result<AsyncFileConnection> {
    Ok(AsyncFileConnection {
        file: Mutex::new(AsyncMavlinkReader::new(File::open(file_path).await?)),
    })
}

pub struct AsyncFileConnection {
    file: Mutex<AsyncMavlinkReader<File>>,
}

impl AsyncTransport for AsyncFileConnection {
    type Reader = File;
    type Writer = Sink;

    fn reader(&self) -> &Mutex<AsyncMavlinkReader<Self::Reader>> {
        &self.file
    }

    async fn lock_writer(&self) -> Option<MutexGuard<'_, Self::Writer>> {
        None
    }

    fn retry_receive(&self, error: &crate::error::MessageReadError) -> bool {
        !matches!(error, crate::error::MessageReadError::Io(error) if error.kind() == io::ErrorKind::UnexpectedEof)
    }
}

#[async_trait]
impl AsyncConnectable for FileConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: crate::Message + Sync + Send,
    {
        Ok(Box::new(AsyncConnectionCore::new_static(
            open(&self.address).await?,
        )))
    }
}

#[async_trait]
impl AsyncDialectConnectable for FileConfig {
    async fn connect_async_with_dialect<D>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D: Dialect + Send + Sync + 'static,
        D::Message: Send + Sync,
    {
        Ok(Box::new(AsyncConnectionCore::new(
            open(&self.address).await?,
            dialect,
        )))
    }
}
