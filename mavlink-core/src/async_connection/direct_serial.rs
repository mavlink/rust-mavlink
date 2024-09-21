//! Async Serial MAVLINK connection

use core::ops::DerefMut;
use std::io;

use tokio::sync::Mutex;
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};

use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message};

#[cfg(not(feature = "signing"))]
use crate::{read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "signing")]
use crate::{
    read_versioned_msg_async_signed, write_versioned_msg_async_signed, SigningConfig, SigningData,
};

use super::AsyncMavConnection;

pub fn open(settings: &str) -> io::Result<AsyncSerialConnection> {
    let settings_toks: Vec<&str> = settings.split(':').collect();
    if settings_toks.len() < 2 {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Incomplete port settings",
        ));
    }

    let Ok(baud) = settings_toks[1].parse::<u32>() else {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "Invalid baud rate",
        ));
    };

    let port_name = settings_toks[0];
    let mut port = tokio_serial::new(port_name, baud).open_native_async()?;
    port.set_data_bits(tokio_serial::DataBits::Eight)?;
    port.set_parity(tokio_serial::Parity::None)?;
    port.set_stop_bits(tokio_serial::StopBits::One)?;
    port.set_flow_control(tokio_serial::FlowControl::None)?;

    Ok(AsyncSerialConnection {
        port: Mutex::new(AsyncPeekReader::new(port)),
        sequence: Mutex::new(0),
        protocol_version: MavlinkVersion::V2,
        #[cfg(feature = "signing")]
        signing_data: None,
    })
}

pub struct AsyncSerialConnection {
    port: Mutex<AsyncPeekReader<SerialStream>>,
    sequence: Mutex<u8>,
    protocol_version: MavlinkVersion,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncSerialConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut port = self.port.lock().await;

        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg_async(port.deref_mut(), self.protocol_version).await;
        #[cfg(feature = "signing")]
        let result = read_versioned_msg_async_signed(
            port.deref_mut(),
            self.protocol_version,
            self.signing_data.as_ref(),
        )
        .await;
        result
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> Result<usize, crate::error::MessageWriteError> {
        let mut port = self.port.lock().await;
        let mut sequence = self.sequence.lock().await;

        let header = MavHeader {
            sequence: *sequence,
            system_id: header.system_id,
            component_id: header.component_id,
        };

        *sequence = sequence.wrapping_add(1);

        #[cfg(not(feature = "signing"))]
        let result =
            write_versioned_msg_async(port.reader_mut(), self.protocol_version, header, data).await;
        #[cfg(feature = "signing")]
        let result = write_versioned_msg_async_signed(
            port.reader_mut(),
            self.protocol_version,
            header,
            data,
            self.signing_data.as_ref(),
        )
        .await;
        result
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
