//! Async File MAVLINK connection

use core::ops::DerefMut;

use super::AsyncMavConnection;
use crate::error::{MessageReadError, MessageWriteError};

use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message};
use crate::{
    read_v1_raw_message_async, read_v2_raw_message_async, read_v2_raw_message_async_signed,
    MAVLinkRawMessage, MAVLinkV2MessageRaw,
};

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

    async fn recv_raw(&self) -> Result<MAVLinkRawMessage, crate::error::MessageReadError> {
        let mut file = self.file.lock().await;
        #[cfg(not(feature = "signing"))]
        let result = match self.protocol_version {
            MavlinkVersion::V1 => {
                MAVLinkRawMessage::V1(read_v1_raw_message_async::<M, _>(file.deref_mut()).await?)
            }
            MavlinkVersion::V2 => {
                MAVLinkRawMessage::V2(read_v2_raw_message_async::<M, _>(file.deref_mut()).await?)
            }
        };
        #[cfg(feature = "signing")]
        let result = match self.protocol_version {
            MavlinkVersion::V1 => {
                MAVLinkRawMessage::V1(read_v1_raw_message_async::<M, _>(file.deref_mut()).await?)
            }
            MavlinkVersion::V2 => MAVLinkRawMessage::V2(
                read_v2_raw_message_async_signed::<M, _>(
                    file.deref_mut(),
                    self.signing_data.as_ref(),
                )
                .await?,
            ),
        };

        Ok(result)
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
