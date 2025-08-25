//! Async Serial MAVLink connection

use core::ops::DerefMut;
use core::sync::atomic::{self, AtomicU8};
use std::io;

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};

use super::AsyncConnectable;
use crate::connection::direct_serial::config::SerialConfig;
use crate::MAVLinkMessageRaw;
use crate::{async_peek_reader::AsyncPeekReader, MavHeader, MavlinkVersion, Message, ReadVersion};

#[cfg(not(feature = "signing"))]
use crate::{read_raw_versioned_msg_async, read_versioned_msg_async, write_versioned_msg_async};
#[cfg(feature = "signing")]
use crate::{
    read_raw_versioned_msg_async_signed, read_versioned_msg_async_signed,
    write_versioned_msg_async_signed, SigningConfig, SigningData,
};

use super::AsyncMavConnection;

pub struct AsyncSerialConnection {
    port: Mutex<AsyncPeekReader<SerialStream>>,
    sequence: AtomicU8,
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    #[cfg(feature = "signing")]
    signing_data: Option<SigningData>,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncSerialConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut port = self.port.lock().await;
        let version = ReadVersion::from_async_conn_cfg::<_, M>(self);
        #[cfg(not(feature = "signing"))]
        let result = read_versioned_msg_async(port.deref_mut(), version).await;
        #[cfg(feature = "signing")]
        let result =
            read_versioned_msg_async_signed(port.deref_mut(), version, self.signing_data.as_ref())
                .await;
        result
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut port = self.port.lock().await;
        let version = ReadVersion::from_async_conn_cfg::<_, M>(self);
        #[cfg(not(feature = "signing"))]
        let result = read_raw_versioned_msg_async::<M, _>(port.deref_mut(), version).await;
        #[cfg(feature = "signing")]
        let result = read_raw_versioned_msg_async_signed::<M, _>(
            port.deref_mut(),
            version,
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
impl AsyncConnectable for SerialConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: Message + Sync + Send,
    {
        let mut port = tokio_serial::new(&self.port_name, self.baud_rate).open_native_async()?;
        port.set_data_bits(tokio_serial::DataBits::Eight)?;
        port.set_parity(tokio_serial::Parity::None)?;
        port.set_stop_bits(tokio_serial::StopBits::One)?;
        port.set_flow_control(tokio_serial::FlowControl::None)?;

        Ok(Box::new(AsyncSerialConnection {
            port: Mutex::new(AsyncPeekReader::new(port)),
            sequence: AtomicU8::new(0),
            protocol_version: MavlinkVersion::V2,
            recv_any_version: false,
            #[cfg(feature = "signing")]
            signing_data: None,
        }))
    }
}
