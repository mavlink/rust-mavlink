//! File MAVLINK connection

use super::config::FileConfig;

use crate::connection::sync::{
    Connectable, Connection, ConnectionCore, DialectConnection, SyncTransport,
};
use crate::peek_reader::PeekReader;
use crate::{Dialect, MavHeader};
use std::fs::File;
use std::io::{self, Sink};
use std::path::PathBuf;
use std::sync::Mutex;

pub fn open(file_path: &PathBuf) -> io::Result<FileConnection> {
    Ok(FileConnection {
        file: Mutex::new(PeekReader::new(File::open(file_path)?)),
    })
}

pub struct FileConnection {
    file: Mutex<PeekReader<File>>,
}

impl SyncTransport for FileConnection {
    type Reader = File;
    type Writer = Sink;

    fn reader(&self) -> std::sync::MutexGuard<'_, PeekReader<Self::Reader>> {
        self.file.lock().unwrap()
    }

    fn writer(&self) -> Option<std::sync::MutexGuard<'_, Self::Writer>> {
        None
    }

    fn retry_receive(&self, error: &crate::error::MessageReadError) -> bool {
        !matches!(error, crate::error::MessageReadError::Io(error) if error.kind() == io::ErrorKind::UnexpectedEof)
    }

    fn next_send_header(&self, _: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        *header
    }
}

impl Connectable for FileConfig {
    fn connect<M: crate::Message>(&self) -> io::Result<Connection<M>> {
        Ok(Box::new(ConnectionCore::new_static(open(&self.address)?)))
    }

    fn connect_with_dialect<D: Dialect + Send + Sync + 'static>(
        &self,
        dialect: D,
    ) -> io::Result<DialectConnection<D>> {
        Ok(Box::new(ConnectionCore::new(open(&self.address)?, dialect)))
    }
}
