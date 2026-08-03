//! Compile-time dialect adapter.

use core::marker::PhantomData;

use crate::{Dialect, MavlinkVersion, Message, error::ParserError};

/// Adapts a generated [`Message`] type to the instance-backed [`Dialect`] API.
///
/// This hidden bridge preserves the existing compile-time API while letting
/// static and runtime dialects share reader framing and decoding logic.
#[doc(hidden)]
// A function-pointer marker keeps this zero-sized adapter from imposing an
// unnecessary `'static` requirement on the generated message type.
pub struct StaticDialect<M>(PhantomData<fn() -> M>);

impl<M> StaticDialect<M> {
    pub(crate) const fn new() -> Self {
        Self(PhantomData)
    }
}

impl<M: Message> Dialect for StaticDialect<M> {
    type Message = M;

    fn message_id(&self, message: &Self::Message) -> u32 {
        message.message_id()
    }

    fn extra_crc(&self, message_id: u32) -> Option<u8> {
        Some(M::extra_crc(message_id))
    }

    fn decode(
        &self,
        version: MavlinkVersion,
        message_id: u32,
        payload: &[u8],
    ) -> Result<Self::Message, ParserError> {
        M::parse(version, message_id, payload)
    }

    fn encode(
        &self,
        version: MavlinkVersion,
        message: &Self::Message,
        payload: &mut [u8],
    ) -> Result<usize, ParserError> {
        Ok(message.ser(version, payload))
    }
}
