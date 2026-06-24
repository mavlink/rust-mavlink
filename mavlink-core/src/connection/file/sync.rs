//! File MAVLINK connection

use crate::connection::{Connection, MavConnection};
use crate::connection_shared::{ConnectionState, read_message, read_raw_message};
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::{Connectable, MAVLinkMessageRaw};
use crate::{MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::fs::File;
use std::io;
use std::path::PathBuf;
use std::sync::Mutex;

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

use super::config::FileConfig;

pub fn open(file_path: &PathBuf) -> io::Result<FileConnection> {
    let file = File::open(file_path)?;

    Ok(FileConnection {
        file: Mutex::new(PeekReader::new(file)),
        state: ConnectionState::new(),
    })
}

pub struct FileConnection {
    file: Mutex<PeekReader<File>>,
    state: ConnectionState,
}

impl<M: Message> MavConnection<M> for FileConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().unwrap();

        loop {
            let result = read_message::<M, _>(file.deref_mut(), &self.state);
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

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut file = self.file.lock().unwrap();

        loop {
            let result = read_raw_message::<M, _>(file.deref_mut(), &self.state);
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

    fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut file = self.file.lock().unwrap();
        read_message::<M, _>(file.deref_mut(), &self.state)
    }

    fn send(&self, _header: &MavHeader, _data: &M) -> Result<usize, MessageWriteError> {
        Ok(0)
    }

    fn send_raw(&self, _data: &MAVLinkMessageRaw) -> Result<usize, MessageWriteError> {
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

impl Connectable for FileConfig {
    fn connect<M: Message>(&self) -> io::Result<Connection<M>> {
        Ok(open(&self.address)?.into())
    }
}
