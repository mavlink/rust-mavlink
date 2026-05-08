//! Async File MAVLINK connection
use core::ops::DerefMut;
use std::io;
use std::path::PathBuf;

use crate::connection::file::config::FileConfig;
use crate::connection::{AsyncConnectable, AsyncMavConnection};
use crate::connection_shared::{ConnectionState, read_message_async, read_raw_message_async};
use crate::error::{MessageReadError, MessageWriteError};
use crate::{
    MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message, async_peek_reader::AsyncPeekReader,
};

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio::fs::File;

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

pub async fn open(file_path: &PathBuf) -> io::Result<AsyncFileConnection> {
    let file = File::open(file_path).await?;
    Ok(AsyncFileConnection {
        file: Mutex::new(AsyncPeekReader::new(file)),
        state: ConnectionState::new(),
    })
}

pub struct AsyncFileConnection {
    file: Mutex<AsyncPeekReader<File>>,
    state: ConnectionState,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncFileConnection {
    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut file = self.file.lock().await;
        loop {
            let result = read_raw_message_async::<M, _>(file.deref_mut(), &self.state).await;
            match result {
                ok @ Ok(..) => {
                    return ok;
                }
                Err(MessageReadError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    return Err(MessageReadError::Io(e));
                }
                _ => {}
            }
        }
    }

    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().await;
        loop {
            let result = read_message_async::<M, _>(file.deref_mut(), &self.state).await;
            match result {
                ok @ Ok(..) => {
                    return ok;
                }
                Err(MessageReadError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    return Err(MessageReadError::Io(e));
                }
                _ => {}
            }
        }
    }

    async fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().await;
        read_message_async::<M, _>(file.deref_mut(), &self.state).await
    }

    async fn send(&self, _header: &MavHeader, _data: &M) -> Result<usize, MessageWriteError> {
        Ok(0)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.state.set_protocol_version(version);
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.state.protocol_version()
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.state.set_allow_recv_any_version(allow);
    }

    fn allow_recv_any_version(&self) -> bool {
        self.state.allow_recv_any_version()
    }

    #[cfg(feature = "mav2-message-signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.state.setup_signing(signing_data);
    }
}

#[async_trait]
impl AsyncConnectable for FileConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        Ok(Box::new(open(&self.address).await?))
    }
}
