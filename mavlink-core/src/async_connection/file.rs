//! Async File MAVLINK connection

use core::ops::DerefMut;

use super::{AsyncConnectable, AsyncMavConnection};
use crate::connectable::FileConnectable;
use crate::error::{MessageReadError, MessageWriteError};

use crate::ReadVersion;
use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message};

use async_trait::async_trait;
use tokio::fs::File;
use tokio::io;
use tokio::sync::Mutex;

#[cfg(not(feature = "signing"))]
use crate::read_versioned_msg_async;

#[cfg(feature = "signing")]
use crate::{read_versioned_msg_async_signed, SigningConfig, SigningData};

pub async fn open(file_path: &str) -> io::Result<AsyncFileConnection> {
    let file = File::open(file_path).await?;
    Ok(AsyncFileConnection {
        file: Mutex::new(AsyncPeekReader::new(file)),
        protocol_version: MavlinkVersion::V2,
        recv_any_version: false,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub struct AsyncFileConnection {
    file: Mutex<AsyncPeekReader<File>>,
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncFileConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().await;
        let version = ReadVersion::from_async_conn_cfg::<_, M>(self);
        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg_async(file.deref_mut(), version).await;
            #[cfg(feature = "signing")]
            let result = read_versioned_msg_async_signed(
                file.deref_mut(),
                version,
                self.signing_data.as_ref(),
            )
            .await;
            match result {
                ok @ Ok(..) => {
                    return ok;
                }
                Err(MessageReadError::Io(e)) => {
                    if e.kind() == io::ErrorKind::UnexpectedEof {
                        return Err(MessageReadError::Io(e));
                    }
                }
                _ => {}
            }
        }
    }

    async fn send(&self, _header: &MavHeader, _data: &M) -> Result<usize, MessageWriteError> {
        Ok(0)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.recv_any_version = allow
    }

    fn allow_recv_any_version(&self) -> bool {
        self.recv_any_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config)
    }
}

#[async_trait]
impl AsyncConnectable for FileConnectable {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        Ok(Box::new(open(&self.address).await?))
    }
}
