//! Async Serial MAVLink connection

use core::sync::atomic::AtomicU8;
use std::io;

use async_trait::async_trait;
use futures::lock::Mutex;
use tokio::io::{BufReader, ReadHalf, WriteHalf};
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};

use crate::async_peek_reader::AsyncPeekReader;
use crate::connection::r#async::{
    AsyncConnectable, AsyncConnectionCore, AsyncDialectConnectable, AsyncDialectConnection,
    AsyncMavConnection, AsyncTransport,
};
use crate::connection::direct_serial::config::SerialConfig;
use crate::connection_shared::next_atomic_send_header;
use crate::{Dialect, MavHeader};

pub struct AsyncSerialConnection {
    read_port: Mutex<AsyncPeekReader<BufReader<ReadHalf<SerialStream>>>>,
    write_port: Mutex<WriteHalf<SerialStream>>,
    sequence: AtomicU8,
}

impl AsyncTransport for AsyncSerialConnection {
    type Reader = BufReader<ReadHalf<SerialStream>>;
    type Writer = WriteHalf<SerialStream>;

    fn reader(&self) -> &Mutex<AsyncPeekReader<Self::Reader>> {
        &self.read_port
    }

    fn writer(&self) -> Option<&Mutex<Self::Writer>> {
        Some(&self.write_port)
    }

    fn retry_receive(&self, error: &crate::error::MessageReadError) -> bool {
        !matches!(error, crate::error::MessageReadError::Io(error) if error.kind() == io::ErrorKind::UnexpectedEof)
    }

    fn next_send_header(&self, _: &mut Self::Writer, header: &MavHeader) -> MavHeader {
        next_atomic_send_header(&self.sequence, header)
    }
}

impl SerialConfig {
    pub(crate) async fn open_async(&self) -> io::Result<AsyncSerialConnection> {
        let mut port = tokio_serial::new(&self.port_name, self.baud_rate).open_native_async()?;
        port.set_data_bits(tokio_serial::DataBits::Eight)?;
        port.set_parity(tokio_serial::Parity::None)?;
        port.set_stop_bits(tokio_serial::StopBits::One)?;
        port.set_flow_control(tokio_serial::FlowControl::None)?;

        let (reader, writer) = tokio::io::split(port);
        Ok(AsyncSerialConnection {
            read_port: Mutex::new(AsyncPeekReader::new(BufReader::with_capacity(
                self.buffer_capacity(),
                reader,
            ))),
            write_port: Mutex::new(writer),
            sequence: AtomicU8::new(0),
        })
    }
}

#[async_trait]
impl AsyncConnectable for SerialConfig {
    async fn connect_async<M>(&self) -> io::Result<Box<dyn AsyncMavConnection<M> + Sync + Send>>
    where
        M: crate::Message + Sync + Send,
    {
        Ok(Box::new(AsyncConnectionCore::new_static(
            self.open_async().await?,
        )))
    }
}

#[async_trait]
impl AsyncDialectConnectable for SerialConfig {
    async fn connect_async_with_dialect<D>(
        &self,
        dialect: D,
    ) -> io::Result<AsyncDialectConnection<D>>
    where
        D: Dialect + Send + Sync + 'static,
        D::Message: Send + Sync,
    {
        Ok(Box::new(AsyncConnectionCore::new(
            self.open_async().await?,
            dialect,
        )))
    }
}
