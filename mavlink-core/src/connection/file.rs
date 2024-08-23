//! File MAVLINK connection

use crate::connection::MavConnection;
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::{MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::fs::File;
use std::io;
use std::sync::Mutex;

#[cfg(not(feature = "signing"))]
use crate::read_versioned_msg;
#[cfg(feature = "signing")]
use crate::{read_versioned_msg_signed, SigningConfig, SigningData};

pub fn open(file_path: &str) -> io::Result<FileConnection> {
    let file = File::open(file_path)?;

    Ok(FileConnection {
        file: Mutex::new(PeekReader::new(file)),
        protocol_version: MavlinkVersion::V2,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub struct FileConnection {
    file: Mutex<PeekReader<File>>,
    protocol_version: MavlinkVersion,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

impl<M: Message> MavConnection<M> for FileConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        // TODO: fix that unwrap
        // not simple b/c PoisonError is not simple
        let mut file = self.file.lock().unwrap();

        loop {
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg(file.deref_mut(), self.protocol_version);
            #[cfg(feature = "signing")]
            let result = read_versioned_msg_signed(
                file.deref_mut(),
                self.protocol_version,
                self.signing_data.as_ref(),
            );
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

    fn send(&self, _header: &MavHeader, _data: &M) -> Result<usize, MessageWriteError> {
        Ok(0)
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    #[cfg(feature = "signing")]
    fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config)
    }
}
