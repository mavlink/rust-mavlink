use core::sync::atomic::{AtomicU8, Ordering};

#[cfg(feature = "std")]
use std::io::{Read, Write};

#[cfg(feature = "tokio")]
use tokio::io::{AsyncRead, AsyncWrite, AsyncWriteExt};

use crate::{
    Dialect, MAVLinkMessageRaw, MavHeader, MavlinkReader, MavlinkVersion, SigningData,
    error::{MessageReadError, MessageWriteError},
    frame_decoder::VersionFilter,
};

#[cfg(feature = "tokio")]
use crate::AsyncMavlinkReader;

#[cfg(feature = "mav2-message-signing")]
use crate::SigningConfig;

pub(crate) struct ConnectionState {
    protocol_version: MavlinkVersion,
    recv_any_version: bool,
    sequence: AtomicU8,
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
            sequence: AtomicU8::new(0),
            #[cfg(feature = "mav2-message-signing")]
            signing_data: None,
        }
    }

    pub(crate) fn version_filter(&self) -> VersionFilter {
        if self.recv_any_version {
            VersionFilter::Any
        } else {
            VersionFilter::Exact(self.protocol_version)
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

    pub(crate) fn next_header(&self, header: &MavHeader) -> MavHeader {
        // Called with the writer locked, so sequence numbers follow write order.
        MavHeader {
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
            ..*header
        }
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

#[cfg(feature = "std")]
#[inline]
pub(crate) fn read_message_with_dialect<D: Dialect, R: Read>(
    reader: &mut MavlinkReader<R>,
    state: &ConnectionState,
    dialect: &D,
) -> Result<(MavHeader, D::Message), MessageReadError> {
    reader.read_message_with_dialect_inner(state.version_filter(), dialect, state.signing_data())
}

#[cfg(feature = "std")]
#[inline]
pub(crate) fn read_raw_message_with_dialect<D: Dialect, R: Read>(
    reader: &mut MavlinkReader<R>,
    state: &ConnectionState,
    dialect: &D,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    reader.read_raw_message_with_dialect_inner(
        state.version_filter(),
        dialect,
        state.signing_data(),
    )
}

#[cfg(feature = "std")]
#[allow(dead_code)]
pub(crate) fn write_raw_message<W: Write>(
    writer: &mut W,
    data: &MAVLinkMessageRaw,
) -> Result<usize, MessageWriteError> {
    let bytes = match data {
        MAVLinkMessageRaw::V1(msg) => msg.raw_bytes(),
        MAVLinkMessageRaw::V2(msg) => msg.raw_bytes(),
    };
    writer.write_all(bytes)?;
    Ok(bytes.len())
}

#[cfg(feature = "tokio")]
#[inline]
pub(crate) async fn read_message_async_with_dialect<D: Dialect, R: AsyncRead + Unpin>(
    reader: &mut AsyncMavlinkReader<R>,
    state: &ConnectionState,
    dialect: &D,
) -> Result<(MavHeader, D::Message), MessageReadError> {
    reader
        .read_message_with_dialect_inner(state.version_filter(), dialect, state.signing_data())
        .await
}

#[cfg(feature = "tokio")]
#[inline]
pub(crate) async fn read_raw_message_async_with_dialect<D: Dialect, R: AsyncRead + Unpin>(
    reader: &mut AsyncMavlinkReader<R>,
    state: &ConnectionState,
    dialect: &D,
) -> Result<MAVLinkMessageRaw, MessageReadError> {
    reader
        .read_raw_message_with_dialect_inner(state.version_filter(), dialect, state.signing_data())
        .await
}

#[cfg(feature = "tokio")]
#[allow(dead_code)]
pub(crate) async fn write_raw_message_async<W: AsyncWrite + Unpin>(
    writer: &mut W,
    data: &MAVLinkMessageRaw,
) -> Result<usize, MessageWriteError> {
    let bytes = match data {
        MAVLinkMessageRaw::V1(msg) => msg.raw_bytes(),
        MAVLinkMessageRaw::V2(msg) => msg.raw_bytes(),
    };
    writer.write_all(bytes).await?;
    Ok(bytes.len())
}

#[cfg(feature = "std")]
#[allow(dead_code)]
pub(crate) fn write_message_with_dialect<D: Dialect, W: Write>(
    writer: &mut W,
    state: &ConnectionState,
    header: MavHeader,
    dialect: &D,
    data: &D::Message,
) -> Result<usize, MessageWriteError> {
    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::write_versioned_msg_with_dialect(
            writer,
            state.protocol_version(),
            header,
            dialect,
            data,
        )
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::write_versioned_msg_with_dialect_signed(
            writer,
            state.protocol_version(),
            header,
            dialect,
            data,
            state.signing_data(),
        )
    }
}

#[cfg(feature = "tokio")]
#[allow(dead_code)]
pub(crate) async fn write_message_async_with_dialect<D: Dialect, W: AsyncWrite + Unpin>(
    writer: &mut W,
    state: &ConnectionState,
    header: MavHeader,
    dialect: &D,
    data: &D::Message,
) -> Result<usize, MessageWriteError> {
    #[cfg(not(feature = "mav2-message-signing"))]
    {
        crate::write_versioned_msg_async_with_dialect(
            writer,
            state.protocol_version(),
            header,
            dialect,
            data,
        )
        .await
    }

    #[cfg(feature = "mav2-message-signing")]
    {
        crate::write_versioned_msg_async_with_dialect_signed(
            writer,
            state.protocol_version(),
            header,
            dialect,
            data,
            state.signing_data(),
        )
        .await
    }
}
