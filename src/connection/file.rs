use crate::connection::MavConnection;
use crate::error::{MessageReadError, MessageWriteError};
use crate::{read_versioned_msg, MavHeader, MavlinkVersion, Message};
use std::fs::File;
use std::io::{self};
use std::sync::Mutex;

/// File MAVLINK connection

pub fn open(file_path: &str) -> io::Result<FileConnection> {
    let file = match File::open(&file_path) {
        Err(e) => return Err(e),
        Ok(file) => file,
    };

    Ok(FileConnection {
        file: Mutex::new(file),
        protocol_version: MavlinkVersion::V2,
    })
}

pub struct FileConnection {
    file: Mutex<std::fs::File>,
    protocol_version: MavlinkVersion,
}

impl<M: Message> MavConnection<M> for FileConnection {
    fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        // TODO: fix that unwrap
        // not simple b/c PoisonError is not simple
        let mut file = self.file.lock().unwrap();

        loop {
            match read_versioned_msg(&mut *file, self.protocol_version) {
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

    fn send(&self, _header: &MavHeader, _data: &M) -> Result<(), MessageWriteError> {
        Ok(())
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    fn get_protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }
}
