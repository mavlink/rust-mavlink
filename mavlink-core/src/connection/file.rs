use crate::connection::MavConnection;
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::{read_versioned_msg, MavHeader, MavlinkVersion, Message};
use core::ops::DerefMut;
use std::fs::File;
use std::io;
use std::sync::Mutex;

/// File MAVLINK connection

pub fn open(file_path: &str) -> io::Result<FileConnection> {
    let file = File::open(file_path)?;

    Ok(FileConnection {
        file: Mutex::new(PeekReader::new(file)),
        protocol_version: MavlinkVersion::V2,
    })
}

pub struct FileConnection {
    file: Mutex<PeekReader<File>>,
    protocol_version: MavlinkVersion,
}

impl<M: Message> MavConnection<M> for FileConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        // TODO: fix that unwrap
        // not simple b/c PoisonError is not simple
        let mut file = self.file.lock().unwrap();

        loop {
            match read_versioned_msg(file.deref_mut(), self.protocol_version) {
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
}
