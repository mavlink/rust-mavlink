//! Serial MAVLINK connection

use crate::connectable::SerialConnectable;
use crate::connection::MavConnection;
use crate::error::{MessageReadError, MessageWriteError};
use crate::peek_reader::PeekReader;
use crate::{MavHeader, MavlinkVersion, Message, ReadVersion};
use core::ops::DerefMut;
use core::sync::atomic::{self, AtomicU8};
use std::io;
use std::sync::Mutex;

use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg, write_versioned_msg};
#[cfg(feature = "signing")]
use crate::{read_versioned_msg_signed, write_versioned_msg_signed, SigningConfig, SigningData};

use super::Connectable;

pub struct SerialConnection {
    port: Mutex<PeekReader<Box<dyn SerialPort>>>,
    sequence: AtomicU8,
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

impl<M: Message> MavConnection<M> for SerialConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        let mut port = self.port.lock().unwrap();
        loop {
            let version = ReadVersion::from_conn_cfg::<_, M>(self);
            #[cfg(not(feature = "signing"))]
            let result = read_versioned_msg(port.deref_mut(), version);
            #[cfg(feature = "signing")]
            let result =
                read_versioned_msg_signed(port.deref_mut(), version, self.signing_data.as_ref());
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

    fn send(&self, header: &MavHeader, data: &M) -> Result<usize, MessageWriteError> {
        let mut port = self.port.lock().unwrap();

        let sequence = self.sequence.fetch_add(
            1,
            // Safety:
            //
            // We are using `Ordering::Relaxed` here because:
            // - We only need a unique sequence number per message
            // - `Mutex` on `self.port` already makes sure the rest of the code is synchronized
            // - No other thread reads or writes `self.sequence` without going through this `Mutex`
            //
            // Warning:
            //
            // If we later change this code to access `self.sequence` without locking `self.port` with the `Mutex`,
            // then we should upgrade this ordering to `Ordering::SeqCst`.
            atomic::Ordering::Relaxed,
        );

        let header = MavHeader {
            sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        #[cfg(not(feature = "signing"))]
        let result = write_versioned_msg(port.reader_mut(), self.protocol_version, header, data);
        #[cfg(feature = "signing")]
        let result = write_versioned_msg_signed(
            port.reader_mut(),
            self.protocol_version,
            header,
            data,
            self.signing_data.as_ref(),
        );
        result
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

impl Connectable for SerialConnectable {
    fn connect<M: Message>(&self) -> io::Result<Box<dyn MavConnection<M> + Sync + Send>> {
        let port = serialport::new(&self.port_name, self.baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .open()?;

        Ok(Box::new(SerialConnection {
            port: Mutex::new(PeekReader::new(port)),
            sequence: AtomicU8::new(0),
            protocol_version: MavlinkVersion::V2,
            #[cfg(feature = "signing")]
            signing_data: None,
            recv_any_version: false,
        }))
    }
}
