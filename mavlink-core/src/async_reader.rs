//! Incremental asynchronous MAVLink reader.

use crate::{
    MAVLinkMessageRaw, MavHeader, MavlinkVersion, Message,
    error::MessageReadError,
    frame_decoder::{FrameDecoder, VersionFilter},
    reader::{try_decode_message, try_decode_raw_message},
};

#[cfg(feature = "tokio")]
use crate::SigningData;

#[cfg(all(feature = "embedded", not(feature = "std")))]
use embedded_io_async::Read;
#[cfg(feature = "tokio")]
use tokio::io::{AsyncRead, AsyncReadExt};

/// Incrementally reads complete MAVLink frames from an asynchronous byte stream.
///
/// Partial input remains buffered if a read is cancelled. Use the same MAVLink
/// dialect `M` for the lifetime of a stream, because CRC validation is
/// dialect-specific.
pub struct AsyncMavlinkReader<R> {
    source: R,
    decoder: FrameDecoder,
}

impl<R> AsyncMavlinkReader<R> {
    /// Creates a reader around `source` with the default read-ahead capacity.
    ///
    /// The buffer is allocated once and reused for the reader's lifetime.
    #[cfg(feature = "std")]
    pub fn new(source: R) -> Self {
        Self {
            source,
            decoder: FrameDecoder::new(),
        }
    }

    /// Creates an allocation-free reader around `source`.
    #[cfg(not(feature = "std"))]
    pub const fn new(source: R) -> Self {
        Self {
            source,
            decoder: FrameDecoder::new(),
        }
    }

    /// Creates a reader with at least `capacity` bytes of read-ahead space.
    ///
    /// Capacities smaller than one maximum-size MAVLink frame are raised to
    /// [`consts::MAX_FRAME_SIZE`](crate::consts::MAX_FRAME_SIZE). The buffer is
    /// allocated once and reused for the lifetime of the reader.
    #[cfg(feature = "std")]
    pub fn with_capacity(capacity: usize, source: R) -> Self {
        Self {
            source,
            decoder: FrameDecoder::with_capacity(capacity),
        }
    }

    /// Returns a shared reference to the underlying source.
    pub const fn get_ref(&self) -> &R {
        &self.source
    }

    /// Returns a mutable reference to the underlying source.
    ///
    /// Reading directly from the source can skip bytes already buffered by
    /// this reader and should therefore be avoided.
    pub const fn get_mut(&mut self) -> &mut R {
        &mut self.source
    }

    /// Returns the underlying source.
    ///
    /// Any bytes read ahead by this reader are discarded.
    pub fn into_inner(self) -> R {
        self.source
    }
}

#[cfg(feature = "tokio")]
impl<R: AsyncRead + Unpin> AsyncMavlinkReader<R> {
    /// Reads and parses the next CRC-valid message accepted by `version`.
    pub async fn read_message<M: Message>(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Exact(version), None)
            .await
    }

    /// Reads and parses the next CRC-valid MAVLink 1 or MAVLink 2 message.
    pub async fn read_any_message<M: Message>(
        &mut self,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Any, None).await
    }

    /// Reads the next CRC-valid raw message accepted by `version`.
    pub async fn read_raw_message<M: Message>(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Exact(version), None)
            .await
    }

    /// Reads the next CRC-valid MAVLink 1 or MAVLink 2 raw message.
    pub async fn read_any_raw_message<M: Message>(
        &mut self,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Any, None)
            .await
    }

    /// Reads, verifies, and parses the next message accepted by `version`.
    /// MAVLink 1 signing is ignored. MAVLink 2 frames must have a valid
    /// signature unless `SigningConfig::allow_unsigned` accepts them.
    #[cfg(feature = "mav2-message-signing")]
    pub async fn read_message_signed<M: Message>(
        &mut self,
        version: MavlinkVersion,
        signing_data: &SigningData,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Exact(version), Some(signing_data))
            .await
    }

    /// Reads, verifies, and parses the next MAVLink 1 or MAVLink 2 message.
    /// MAVLink 1 frames follow `SigningConfig::allow_unsigned`.
    #[cfg(feature = "mav2-message-signing")]
    pub async fn read_any_message_signed<M: Message>(
        &mut self,
        signing_data: &SigningData,
    ) -> Result<(MavHeader, M), MessageReadError> {
        self.read_message_inner(VersionFilter::Any, Some(signing_data))
            .await
    }

    /// Reads and verifies the next raw message accepted by `version`.
    /// MAVLink 1 signing is ignored. MAVLink 2 frames must have a valid
    /// signature unless `SigningConfig::allow_unsigned` accepts them.
    #[cfg(feature = "mav2-message-signing")]
    pub async fn read_raw_message_signed<M: Message>(
        &mut self,
        version: MavlinkVersion,
        signing_data: &SigningData,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Exact(version), Some(signing_data))
            .await
    }

    /// Reads and verifies the next MAVLink 1 or MAVLink 2 raw message.
    /// MAVLink 1 frames follow `SigningConfig::allow_unsigned`.
    #[cfg(feature = "mav2-message-signing")]
    pub async fn read_any_raw_message_signed<M: Message>(
        &mut self,
        signing_data: &SigningData,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        self.read_raw_message_inner::<M>(VersionFilter::Any, Some(signing_data))
            .await
    }

    #[inline]
    pub(crate) async fn read_message_inner<M: Message>(
        &mut self,
        filter: VersionFilter,
        signing_data: Option<&SigningData>,
    ) -> Result<(MavHeader, M), MessageReadError> {
        loop {
            if let Some(result) = try_decode_message::<M>(&mut self.decoder, filter, signing_data) {
                return result;
            }
            self.read_more().await?;
        }
    }

    #[inline]
    pub(crate) async fn read_raw_message_inner<M: Message>(
        &mut self,
        filter: VersionFilter,
        signing_data: Option<&SigningData>,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        loop {
            if let Some(message) =
                try_decode_raw_message::<M>(&mut self.decoder, filter, signing_data)
            {
                return Ok(message);
            }
            self.read_more().await?;
        }
    }

    async fn read_more(&mut self) -> Result<(), MessageReadError> {
        let destination = self.decoder.spare_capacity_mut();
        assert!(
            !destination.is_empty(),
            "a pending MAVLink frame must fit in the decoder buffer"
        );
        let count = loop {
            match self.source.read(destination).await {
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                result => break result?,
            }
        };
        if count == 0 {
            return Err(MessageReadError::eof());
        }
        self.decoder.commit(count);
        Ok(())
    }
}

#[cfg(all(feature = "embedded", not(feature = "std")))]
impl<R: Read> AsyncMavlinkReader<R> {
    /// Reads and parses the next CRC-valid message accepted by `version`.
    pub async fn read_message<M: Message>(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<(MavHeader, M), MessageReadError> {
        let filter = VersionFilter::Exact(version);
        loop {
            if let Some(result) = try_decode_message::<M>(&mut self.decoder, filter, None) {
                return result;
            }
            self.read_more().await?;
        }
    }

    /// Reads and parses the next CRC-valid MAVLink 1 or MAVLink 2 message.
    pub async fn read_any_message<M: Message>(
        &mut self,
    ) -> Result<(MavHeader, M), MessageReadError> {
        loop {
            if let Some(result) =
                try_decode_message::<M>(&mut self.decoder, VersionFilter::Any, None)
            {
                return result;
            }
            self.read_more().await?;
        }
    }

    /// Reads the next CRC-valid raw message accepted by `version`.
    pub async fn read_raw_message<M: Message>(
        &mut self,
        version: MavlinkVersion,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        let filter = VersionFilter::Exact(version);
        loop {
            if let Some(message) = try_decode_raw_message::<M>(&mut self.decoder, filter, None) {
                return Ok(message);
            }
            self.read_more().await?;
        }
    }

    /// Reads the next CRC-valid MAVLink 1 or MAVLink 2 raw message.
    pub async fn read_any_raw_message<M: Message>(
        &mut self,
    ) -> Result<MAVLinkMessageRaw, MessageReadError> {
        loop {
            if let Some(message) =
                try_decode_raw_message::<M>(&mut self.decoder, VersionFilter::Any, None)
            {
                return Ok(message);
            }
            self.read_more().await?;
        }
    }

    async fn read_more(&mut self) -> Result<(), MessageReadError> {
        let destination = self.decoder.spare_capacity_mut();
        assert!(
            !destination.is_empty(),
            "a pending MAVLink frame must fit in the decoder buffer"
        );
        let count = self
            .source
            .read(destination)
            .await
            .map_err(|_| MessageReadError::Io)?;
        if count == 0 {
            return Err(MessageReadError::eof());
        }
        self.decoder.commit(count);
        Ok(())
    }
}
