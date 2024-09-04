use core::ops::DerefMut;

use super::AsyncMavConnection;
use crate::error::{MessageReadError, MessageWriteError};

use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message};

use tokio::fs::File;
use tokio::io;
use tokio::sync::Mutex;

#[cfg(not(feature = "signing"))]
use crate::read_versioned_msg_async;

#[cfg(feature = "signing")]
use crate::{read_versioned_msg_async_signed, SigningConfig, SigningData};

/// File MAVLINK connection

pub async fn open(file_path: &str) -> io::Result<AsyncFileConnection> {
    let file = File::open(file_path).await?;
    Ok(AsyncFileConnection {
        file: Mutex::new(AsyncPeekReader::new(file)),
        protocol_version: MavlinkVersion::V2,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub struct AsyncFileConnection {
    file: Mutex<AsyncPeekReader<File>>,
    protocol_version: MavlinkVersion,

    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncFileConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().await;

        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg_async(file.deref_mut(), self.protocol_version).await;
            #[cfg(feature = "signing")]
            let result = read_versioned_msg_async_signed(
                file.deref_mut(),
                self.protocol_version,
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

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config)
    }
}
