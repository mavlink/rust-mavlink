use core::sync::atomic::{AtomicU8, Ordering};

#[cfg(feature = "std")]
use std::io::{Read, Write};

#[cfg(feature = "tokio")]
use tokio::io::{AsyncRead, AsyncWrite};

#[cfg(feature = "tokio")]
use crate::async_peek_reader::AsyncPeekReader;
use crate::{
    MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message, ReadVersion, SigningData,
    error::{MessageReadError, MessageWriteError},
    peek_reader::PeekReader,
};

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

pub(crate) struct ConnectionState {
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    #[cfg(feature = "mav2-message-signing")]
    signing_data: Option<SigningData>,
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self::new()
    }
}

impl ConnectionState {
    pub(crate) fn new() -> Self {
        Self {
            protocol_version: MavlinkVersion::V2,
            recv_any_version: false,
            #[cfg(feature = "mav2-message-signing")]
            signing_data: None,
        }
    }

    pub(crate) fn read_version(&self) -> ReadVersion {
        if self.recv_any_version {
            ReadVersion::Any
        } else {
            self.protocol_version.into()
        }
    }

    pub(crate) fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.protocol_version = version;
    }

    pub(crate) fn protocol_version(&self) -> MavlinkVersion {
        self.protocol_version
    }

    pub(crate) fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.recv_any_version = allow;
    }

    pub(crate) fn allow_recv_any_version(&self) -> bool {
        self.recv_any_version
    }

    #[allow(dead_code)]
    pub(crate) fn signing_data(&self) -> Option<&SigningData> {
        #[cfg(feature = "mav2-message-signing")]
        {
            self.signing_data.as_ref()
        }

        #[cfg(not(feature = "mav2-message-signing"))]
        {
            None
        }
    }

    #[cfg(feature = "mav2-message-signing")]
    pub(crate) fn setup_signing(&mut self, signing_data: Option<SigningConfig>) {
        self.signing_data = signing_data.map(SigningData::from_config);
    }
}

#[allow(dead_code)]
pub(crate) fn next_send_header(sequence: &mut u8, header: &MavHeader) -> MavHeader {
    let next = MavHeader {
        sequence: *sequence,
        system_id: header.system_id,
        component_id: header.component_id,
    };
    *sequence = sequence.wrapping_add(1);
    next
}

#[allow(dead_code)]
pub(crate) fn next_atomic_send_header(sequence: &AtomicU8, header: &MavHeader) -> MavHeader {
    // The serial transports serialize writes behind a mutex; the atomic only provides
    // a wrapping packet sequence number, not cross-field synchronization.
    let sequence = sequence.fetch_add(1, Ordering::Relaxed);
    MavHeader {
        sequence,
        system_id: header.system_id,
        component_id: header.component_id,
    }
}

#[cfg(feature = "std")]
pub(crate) fn read_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    state: &ConnectionState,
) -> Result<(MavHeader, M), MessageReadError> {
    let version = state.read_version();

    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::read_versioned_msg(reader, version)
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::read_versioned_msg_signed(reader, version, state.signing_data())
    }
}

#[cfg(feature = "std")]
pub(crate) fn read_raw_message<M: Message, R: Read>(
    reader: &mut PeekReader<R>,
    state: &ConnectionState,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    let version = state.read_version();

    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::read_versioned_raw_message::<M, _>(reader, version)
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::read_versioned_raw_message_signed::<M, _>(reader, version, state.signing_data())
    }
}

#[cfg(feature = "std")]
#[allow(dead_code)]
pub(crate) fn write_message<M: Message, W: Write>(
    writer: &mut W,
    state: &ConnectionState,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::write_versioned_msg(writer, state.protocol_version(), header, data)
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::write_versioned_msg_signed(
            writer,
            state.protocol_version(),
            header,
            data,
            state.signing_data(),
        )
    }
}

#[cfg(feature = "tokio")]
pub(crate) async fn read_message_async<M: Message, R: AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    state: &ConnectionState,
) -> Result<(MavHeader, M), MessageReadError> {
    let version = state.read_version();

    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::read_versioned_msg_async(reader, version).await
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::read_versioned_msg_async_signed(reader, version, state.signing_data()).await
    }
}

#[cfg(feature = "tokio")]
pub(crate) async fn read_raw_message_async<M: Message, R: AsyncRead + Unpin>(
    reader: &mut AsyncPeekReader<R>,
    state: &ConnectionState,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    let version = state.read_version();

    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::read_versioned_raw_message_async::<M, _>(reader, version).await
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::read_versioned_raw_message_async_signed::<M, _>(
            reader,
            version,
            state.signing_data(),
        )
        .await
    }
}

#[cfg(feature = "tokio")]
#[allow(dead_code)]
pub(crate) async fn write_message_async<M: Message, W: AsyncWrite + Unpin>(
    writer: &mut W,
    state: &ConnectionState,
    header: MavHeader,
    data: &M,
) -> Result<usize, MessageWriteError> {
    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::write_versioned_msg_async(writer, state.protocol_version(), header, data).await
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::write_versioned_msg_async_signed(
            writer,
            state.protocol_version(),
            header,
            data,
            state.signing_data(),
        )
        .await
    }
}
