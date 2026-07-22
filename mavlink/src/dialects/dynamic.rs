//! Dynamically loaded MAVLink dialects.
//!
//! This module complements, rather than replaces, the generated dialects in
//! [`super`]. A [`DynamicDialect`] loads a MAVLink XML definition and
//! decodes validated frames into [`DynamicMessage`] values. Dynamic messages
//! retain their schema and payload so callers can inspect vendor-specific
//! messages without compiling generated Rust types.

use std::{
    collections::{BTreeMap, HashSet},
    error::Error,
    fmt::{Display, Formatter},
    path::{Path, PathBuf},
    sync::Arc,
};

use mavlink_bindgen::{BindGenError, parser};
use mavlink_core::error::ParserError;

use crate::{Dialect, MavlinkVersion};

/// An error returned while loading a runtime MAVLink dialect.
#[derive(Debug)]
pub enum DynamicDialectError {
    /// The XML dialect file could not be resolved.
    Io {
        /// Path supplied by the caller.
        path: PathBuf,
        /// Underlying I/O error.
        source: std::io::Error,
    },
    /// The MAVLink XML definition could not be loaded.
    Xml(BindGenError),
    /// Two different messages use the same MAVLink message ID.
    DuplicateMessageId {
        /// Duplicate ID.
        id: u32,
        /// First message name.
        first: String,
        /// Conflicting message name.
        second: String,
    },
    /// A message ID cannot be represented in a MAVLink 2 frame.
    MessageIdOutOfRange {
        /// Invalid message ID.
        id: u32,
        /// Message name.
        name: String,
    },
    /// A message payload does not fit in a MAVLink frame.
    PayloadTooLong {
        /// Message name.
        name: String,
        /// Calculated encoded payload length.
        length: usize,
    },
}

impl Display for DynamicDialectError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(
                    f,
                    "failed to resolve runtime dialect {}: {source}",
                    path.display()
                )
            }
            Self::Xml(error) => write!(f, "failed to load runtime dialect XML: {error}"),
            Self::DuplicateMessageId { id, first, second } => write!(
                f,
                "runtime dialect defines message ID {id} for both {first} and {second}"
            ),
            Self::MessageIdOutOfRange { id, name } => write!(
                f,
                "runtime dialect message {name} has ID {id}, which exceeds the MAVLink 2 limit"
            ),
            Self::PayloadTooLong { name, length } => write!(
                f,
                "runtime dialect message {name} has a {length}-byte payload, exceeding MAVLink's 255-byte limit"
            ),
        }
    }
}

impl Error for DynamicDialectError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::Xml(error) => Some(error),
            Self::DuplicateMessageId { .. }
            | Self::MessageIdOutOfRange { .. }
            | Self::PayloadTooLong { .. } => None,
        }
    }
}

/// Metadata for a field in a runtime-loaded message definition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DynamicField {
    name: String,
    primitive_type: String,
    offset: usize,
    encoded_len: usize,
    is_extension: bool,
}

impl DynamicField {
    /// XML field name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The MAVLink primitive type, such as `uint32_t` or `float`.
    #[must_use]
    pub fn primitive_type(&self) -> &str {
        &self.primitive_type
    }

    /// Byte offset in the serialized message payload.
    #[must_use]
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// Number of bytes occupied by this field.
    #[must_use]
    pub fn encoded_len(&self) -> usize {
        self.encoded_len
    }

    /// Whether this is a MAVLink 2 extension field.
    #[must_use]
    pub fn is_extension(&self) -> bool {
        self.is_extension
    }
}

/// Metadata for one runtime-loaded MAVLink message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DynamicMessageDefinition {
    id: u32,
    name: String,
    extra_crc: u8,
    base_payload_len: usize,
    encoded_len: usize,
    fields: Vec<DynamicField>,
}

impl DynamicMessageDefinition {
    /// MAVLink message ID.
    #[must_use]
    pub fn id(&self) -> u32 {
        self.id
    }

    /// MAVLink message name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// MAVLink CRC extra byte.
    #[must_use]
    pub fn extra_crc(&self) -> u8 {
        self.extra_crc
    }

    /// Serialized length before MAVLink 2 extension fields.
    #[must_use]
    pub fn base_payload_len(&self) -> usize {
        self.base_payload_len
    }

    /// Maximum serialized payload length, including extension fields.
    #[must_use]
    pub fn encoded_len(&self) -> usize {
        self.encoded_len
    }

    /// Fields in MAVLink wire order.
    #[must_use]
    pub fn fields(&self) -> &[DynamicField] {
        &self.fields
    }
}

/// A message decoded through a [`DynamicDialect`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DynamicMessage {
    definition: Arc<DynamicMessageDefinition>,
    version: MavlinkVersion,
    payload: Vec<u8>,
}

impl DynamicMessage {
    /// Loaded definition used to decode this message.
    #[must_use]
    pub fn definition(&self) -> &DynamicMessageDefinition {
        &self.definition
    }

    /// MAVLink framing version for this message.
    #[must_use]
    pub fn version(&self) -> MavlinkVersion {
        self.version
    }

    /// Raw serialized MAVLink payload, without the frame header or checksum.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    /// Raw bytes for the named field, if the field was present in the frame.
    ///
    /// MAVLink 2 may omit trailing zero bytes. Such an omitted field returns
    /// `None`; callers that need its semantic value should treat it as a
    /// zero-filled field of the definition's encoded length.
    #[must_use]
    pub fn field_bytes(&self, name: &str) -> Option<&[u8]> {
        let field = self
            .definition
            .fields
            .iter()
            .find(|field| field.name == name)?;
        let end = field.offset.checked_add(field.encoded_len)?;
        self.payload.get(field.offset..end)
    }
}

#[derive(Debug)]
struct DynamicDialectInner {
    messages: BTreeMap<u32, Arc<DynamicMessageDefinition>>,
}

/// An immutable MAVLink dialect loaded from XML at runtime.
#[derive(Debug, Clone)]
pub struct DynamicDialect {
    inner: Arc<DynamicDialectInner>,
}

impl DynamicDialect {
    /// Load a dialect XML file and all of its relative `<include>` definitions.
    ///
    /// The XML is parsed once. The resulting dialect is immutable, cheap to
    /// clone, and can be shared safely between readers.
    pub fn from_xml_file(path: impl AsRef<Path>) -> Result<Self, DynamicDialectError> {
        let requested_path = path.as_ref();
        let source_file =
            requested_path
                .canonicalize()
                .map_err(|source| DynamicDialectError::Io {
                    path: requested_path.to_owned(),
                    source,
                })?;
        let definitions_dir = source_file.parent().unwrap_or_else(|| Path::new(""));
        let definition_file = Path::new(
            source_file
                .file_name()
                .expect("a canonical runtime dialect path must name a file"),
        );
        let mut parsed_files = HashSet::new();
        let profile = parser::parse_profile(definitions_dir, definition_file, &mut parsed_files)
            .map_err(DynamicDialectError::Xml)?;

        Self::from_profile(profile)
    }

    /// Return the loaded definition for `message_id`.
    #[must_use]
    pub fn message_definition(&self, message_id: u32) -> Option<&DynamicMessageDefinition> {
        self.inner.messages.get(&message_id).map(Arc::as_ref)
    }

    /// Return all loaded message definitions, ordered by message ID.
    pub fn message_definitions(&self) -> impl Iterator<Item = &DynamicMessageDefinition> {
        self.inner.messages.values().map(Arc::as_ref)
    }

    /// Construct an encodable runtime message from a known definition and its
    /// wire payload. This is the dynamic counterpart to constructing a
    /// generated `MavMessage` value before calling `send`.
    pub fn message_from_payload(
        &self,
        version: MavlinkVersion,
        message_id: u32,
        payload: impl Into<Vec<u8>>,
    ) -> Result<DynamicMessage, ParserError> {
        self.decode(version, message_id, &payload.into())
    }

    fn from_profile(profile: parser::MavProfile) -> Result<Self, DynamicDialectError> {
        let mut messages = BTreeMap::new();

        for message in profile.messages.into_values() {
            if message.id > 0x00ff_ffff {
                return Err(DynamicDialectError::MessageIdOutOfRange {
                    id: message.id,
                    name: message.name,
                });
            }

            let definition = Arc::new(runtime_message_definition(&message)?);
            if let Some(existing) = messages.insert(definition.id, Arc::clone(&definition)) {
                return Err(DynamicDialectError::DuplicateMessageId {
                    id: definition.id,
                    first: existing.name.clone(),
                    second: definition.name.clone(),
                });
            }
        }

        Ok(Self {
            inner: Arc::new(DynamicDialectInner { messages }),
        })
    }
}

impl Dialect for DynamicDialect {
    type Message = DynamicMessage;

    fn message_id(&self, message: &Self::Message) -> u32 {
        message.definition.id
    }

    fn extra_crc(&self, message_id: u32) -> Option<u8> {
        self.inner
            .messages
            .get(&message_id)
            .map(|definition| definition.extra_crc)
    }

    fn decode(
        &self,
        version: MavlinkVersion,
        message_id: u32,
        payload: &[u8],
    ) -> Result<Self::Message, ParserError> {
        let definition = self
            .inner
            .messages
            .get(&message_id)
            .ok_or(ParserError::UnknownMessage { id: message_id })?;
        if payload.len() > definition.encoded_len {
            return Err(ParserError::InvalidPayloadLength {
                id: message_id,
                maximum: definition.encoded_len,
                actual: payload.len(),
            });
        }

        Ok(DynamicMessage {
            definition: Arc::clone(definition),
            version,
            payload: payload.to_vec(),
        })
    }

    fn encode(
        &self,
        version: MavlinkVersion,
        message: &Self::Message,
        payload: &mut [u8],
    ) -> Result<usize, ParserError> {
        let definition =
            self.inner
                .messages
                .get(&message.definition.id)
                .ok_or(ParserError::UnknownMessage {
                    id: message.definition.id,
                })?;
        if !Arc::ptr_eq(definition, &message.definition) {
            return Err(ParserError::InvalidPayloadLength {
                id: message.definition.id,
                maximum: definition.encoded_len,
                actual: message.payload.len(),
            });
        }
        let encoded_len = match version {
            MavlinkVersion::V1 => definition.base_payload_len,
            MavlinkVersion::V2 => message.payload.len(),
        };
        if encoded_len > payload.len() {
            return Err(ParserError::InvalidPayloadLength {
                id: message.definition.id,
                maximum: payload.len(),
                actual: encoded_len,
            });
        }
        if version == MavlinkVersion::V1 {
            payload[..encoded_len].fill(0);
        }
        payload[..message.payload.len().min(encoded_len)]
            .copy_from_slice(&message.payload[..message.payload.len().min(encoded_len)]);
        Ok(encoded_len)
    }
}

fn runtime_message_definition(
    message: &parser::MavMessage,
) -> Result<DynamicMessageDefinition, DynamicDialectError> {
    let mut fields = Vec::with_capacity(message.fields.len());
    let mut offset = 0;
    let mut base_payload_len = 0;

    for field in &message.fields {
        let encoded_len = mav_type_len(&field.mavtype);
        fields.push(DynamicField {
            name: field.name.clone(),
            primitive_type: field.mavtype.primitive_type(),
            offset,
            encoded_len,
            is_extension: field.is_extension,
        });
        offset += encoded_len;
        if !field.is_extension {
            base_payload_len = offset;
        }
    }

    if offset > u8::MAX.into() {
        return Err(DynamicDialectError::PayloadTooLong {
            name: message.name.clone(),
            length: offset,
        });
    }

    Ok(DynamicMessageDefinition {
        id: message.id,
        name: message.name.clone(),
        extra_crc: parser::extra_crc(message),
        base_payload_len,
        encoded_len: offset,
        fields,
    })
}

fn mav_type_len(mav_type: &parser::MavType) -> usize {
    use parser::MavType;

    match mav_type {
        MavType::UInt8MavlinkVersion | MavType::UInt8 | MavType::Int8 | MavType::Char => 1,
        MavType::UInt16 | MavType::Int16 => 2,
        MavType::UInt32 | MavType::Int32 | MavType::Float => 4,
        MavType::UInt64 | MavType::Int64 | MavType::Double => 8,
        MavType::CharArray(length) => *length,
        MavType::Array(element, length) => mav_type_len(element) * length,
    }
}

#[cfg(all(test, feature = "dialect-common"))]
mod tests {
    use std::io::Cursor;

    use crate::{
        MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, ReadVersion,
        dialects::common::{
            COMMAND_LONG_DATA, HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag,
            MavState, MavType,
        },
        peek_reader::PeekReader,
        read_versioned_msg_with_dialect,
    };

    use super::DynamicDialect;

    #[test]
    fn runtime_loaded_dialect_byte_rep_check() {
        let xml = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/mavlink/message_definitions/v1.0/common.xml"
        );
        let dialect = DynamicDialect::from_xml_file(xml).unwrap();
        let header = MavHeader {
            sequence: 34,
            system_id: 1,
            component_id: 2,
        };
        let messages = [
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0x0102_0304,
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
                base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            }),
            MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                param1: 1.0,
                param2: 2.0,
                param3: 3.0,
                param4: 4.0,
                param5: 5.0,
                param6: 6.0,
                param7: 7.0,
                command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                target_system: 1,
                target_component: 2,
                confirmation: 3,
            }),
        ];

        for message in messages {
            let mut common_frame = MAVLinkV2MessageRaw::new();
            common_frame.serialize_message(header, &message);

            let mut reader = PeekReader::new(Cursor::new(common_frame.raw_bytes().to_vec()));
            let (decoded_header, dynamic_message) = read_versioned_msg_with_dialect(
                &mut reader,
                ReadVersion::Single(MavlinkVersion::V2),
                &dialect,
            )
            .unwrap();

            let mut dynamic_frame = MAVLinkV2MessageRaw::new();
            dynamic_frame
                .serialize_message_with_dialect(decoded_header, &dialect, &dynamic_message)
                .unwrap();

            assert_eq!(dynamic_frame.raw_bytes(), common_frame.raw_bytes());
        }
    }
}
