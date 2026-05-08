//! Async Serial MAVLink connection

use core::ops::DerefMut;
use core::sync::atomic::AtomicU8;
use std::io;

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio::io::{BufReader, ReadHalf, WriteHalf};
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};

use crate::MAVLinkMessageRaw;
use crate::connection::AsyncConnectable;
use crate::connection::direct_serial::config::SerialConfig;
use crate::connection_shared::{
    ConnectionState, next_atomic_send_header, read_message_async, read_raw_message_async,
    write_message_async,
};
use crate::error::MessageReadError;
use crate::{MavHeader, MavlinkVersion, Message, async_peek_reader::AsyncPeekReader};

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

use crate::connection::AsyncMavConnection;

pub struct AsyncSerialConnection {
    read_port: Mutex<AsyncPeekReader<BufReader<ReadHalf<SerialStream>>>>,
    write_port: Mutex<WriteHalf<SerialStream>>,
    sequence: AtomicU8,
    state: ConnectionState,
}

#[async_trait::async_trait]
impl<M: Message + Sync + Send> AsyncMavConnection<M> for AsyncSerialConnection {
    async fn recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut port = self.read_port.lock().await;
        loop {
            let result = read_message_async::<M, _>(port.deref_mut(), &self.state).await;
            match result {
                Ok(message) => return Ok(message),
                Err(MessageReadError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    return Err(MessageReadError::Io(e));
                }
                _ => {}
            }
        }
    }

    async fn recv_raw(&self) -> Result<MAVLinkMessageRaw, crate::error::MessageReadError> {
        let mut port = self.read_port.lock().await;
        loop {
            let result = read_raw_message_async::<M, _>(port.deref_mut(), &self.state).await;
            match result {
                Ok(message) => return Ok(message),
                Err(MessageReadError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => {
                    return Err(MessageReadError::Io(e));
                }
                _ => {}
            }
        }
    }

    async fn try_recv(&self) -> Result<(MavHeader, M), crate::error::MessageReadError> {
        let mut port = self.read_port.lock().await;
        read_message_async::<M, _>(port.deref_mut(), &self.state).await
    }

    async fn send(
        &self,
        header: &MavHeader,
        data: &M,
    ) -> Result<usize, crate::error::MessageWriteError> {
        let mut port = self.write_port.lock().await;

        let header = next_atomic_send_header(&self.sequence, header);
        write_message_async(&mut *port, &self.state, header, data).await
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

        let (reader, writer) = tokio::io::split(port);
        let read_buffer_capacity = self.buffer_capacity();
        let buf_reader = BufReader::with_capacity(read_buffer_capacity, reader);

        Ok(Box::new(AsyncSerialConnection {
            read_port: Mutex::new(AsyncPeekReader::new(buf_reader)),
            write_port: Mutex::new(writer),
            sequence: AtomicU8::new(0),
            state: ConnectionState::new(),
        }))
    }
}
