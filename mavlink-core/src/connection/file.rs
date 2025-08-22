//! File MAVLINK connection

use crate::connection::MavConnection;
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::{Connectable, MAVLinkMessageRaw};
use crate::{MavHeader, MavlinkVersion, Message, ReadVersion};
use core::ops::DerefMut;
use std::fs::File;
use std::io;
use std::path::PathBuf;
use std::sync::Mutex;

#[cfg(not(feature = "signing"))]
use crate::{read_raw_versioned_msg, read_versioned_msg};
#[cfg(feature = "signing")]
use crate::{read_raw_versioned_msg_signed, read_versioned_msg_signed, SigningConfig, SigningData};

pub mod config;

use config::FileConfig;

pub fn open(file_path: &PathBuf) -> io::Result<FileConnection> {
    let file = File::open(file_path)?;

    Ok(FileConnection {
        file: Mutex::new(PeekReader::new(file)),
        protocol_version: MavlinkVersion::V2,
        #[cfg(feature = "signing")]
        signing_data: None,
        recv_any_version: false,
    })
}

pub struct FileConnection {
    file: Mutex<PeekReader<File>>,
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

impl<M: Message> MavConnection<M> for FileConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        // TODO: fix that unwrap
        // not simple b/c PoisonError is not simple
        let mut file = self.file.lock().unwrap();

        loop {
            let version = ReadVersion::from_conn_cfg::<_, M>(self);
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg(file.deref_mut(), version);
            #[cfg(feature = "signing")]
            let result =
                read_versioned_msg_signed(file.deref_mut(), version, self.signing_data.as_ref());
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

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        // TODO: fix that unwrap
        // not simple b/c PoisonError is not simple
        let mut file = self.file.lock().unwrap();

        loop {
            let version = ReadVersion::from_conn_cfg::<_, M>(self);
            #[cfg(not(feature = "signing"))]
            let result = read_raw_versioned_msg::<M, _>(file.deref_mut(), version);
            #[cfg(feature = "signing")]
            let result = read_raw_versioned_msg_signed::<M, _>(
                file.deref_mut(),
                version,
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

    fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().unwrap();
        let version = ReadVersion::from_conn_cfg::<_, M>(self);

        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg(file.deref_mut(), version);
        #[cfg(feature = "signing")]
        let result =
            read_versioned_msg_signed(file.deref_mut(), version, self.signing_data.as_ref());

        result
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

impl Connectable for FileConfig {
    fn connect<M: Message>(&self) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
        Ok(Box::new(open(&self.address)?))
    }
}
