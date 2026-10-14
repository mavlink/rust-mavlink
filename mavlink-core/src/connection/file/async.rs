//! Async File MAVLINK connection

use std::io;
use std::path::PathBuf;

use futures::lock::{Mutex, MutexGuard};
use tokio::fs::File;
use tokio::io::Sink;

use crate::AsyncMavlinkReader;
use crate::Dialect;
use crate::connection::r#async::{
    AsyncConnectable, AsyncDialectConnection, AsyncTransport, Transport,
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

impl AsyncConnectable for FileConfig {
    async fn connect_async_with_dialect<D>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D: Dialect + Send + Sync,
        D::Message: Send + Sync,
    {
        Ok(AsyncDialectConnection::new(
            Transport::File(open(&self.address).await?),
            dialect,
        ))
    }
}
