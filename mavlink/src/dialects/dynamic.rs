//! Dynamically loaded MAVLink dialects.
//!
//! This module complements, rather than replaces, the generated dialects in
//! [`super`]. A [`DynamicDialect`] loads a MAVLink XML definition and decodes
//! or constructs [`DynamicMessage`] values. Dynamic messages retain their
//! schema and payload so callers can use vendor-specific messages without
//! compiling generated Rust types.

use std::{
    borrow::Cow,
    collections::{BTreeMap, HashMap, HashSet},
    error::Error,
    fmt::{Display, Formatter},
    path::{Path, PathBuf},
    sync::Arc,
};

use mavlink_bindgen::{BindGenError, parser};
use mavlink_core::error::{EncoderError, ParserError};

use crate::{Dialect, MavlinkVersion};

pub use super::dynamic_parser::DynamicValue;
use super::dynamic_parser::{write_default, write_value};

/// An error returned while constructing a runtime message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DynamicMessageError {
    /// Dialect has no message with the requested name.
    UnknownMessage { name: String },
    /// Message has no field with the requested name.
    UnknownField { message: String, field: String },
    /// Field value does not match its dialect definition.
    InvalidValue {
        message: String,
        field: String,
        reason: String,
    },
}

impl Display for DynamicMessageError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownMessage { name } => write!(f, "unknown runtime dialect message {name}"),
            Self::UnknownField { message, field } => {
                write!(f, "runtime dialect message {message} has no field {field}")
            }
            Self::InvalidValue {
                message,
                field,
                reason,
            } => write!(f, "invalid value for {message}.{field}: {reason}"),
        }
    }
}

impl Error for DynamicMessageError {}

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
    pub(super) mavtype: parser::MavType,
    pub(super) enumeration: Option<Arc<DynamicEnumDefinition>>,
    offset: usize,
    encoded_size: usize,
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
    pub fn encoded_size(&self) -> usize {
        self.encoded_size
    }

    /// Whether this is a MAVLink 2 extension field.
    #[must_use]
    pub fn is_extension(&self) -> bool {
        self.is_extension
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct DynamicEnumDefinition {
    pub(super) name: String,
    pub(super) primitive: Option<parser::MavType>,
    pub(super) entries: Vec<(String, u64)>,
}

/// Metadata for one runtime-loaded MAVLink message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DynamicMessageDefinition {
    id: u32,
    name: String,
    extra_crc: u8,
    base_payload_len: usize,
    encoded_size: usize,
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

    /// Maximum serialized payload size including extension fields.
    #[must_use]
    pub fn encoded_size(&self) -> usize {
        self.encoded_size
    }

    /// Fields in MAVLink wire order.
    #[must_use]
    pub fn fields(&self) -> &[DynamicField] {
        &self.fields
    }
}

/// A message decoded or constructed through a [`DynamicDialect`].
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
    ///
    /// For MAVLink 2 this can include extension bytes not yet described by
    /// the loaded dialect.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    /// Serialized bytes for the named field, or `None` if the field is unknown.
    ///
    /// Missing payload bytes are zero-filled.
    #[must_use]
    pub fn field_bytes(&self, name: &str) -> Option<Cow<'_, [u8]>> {
        let field = self
            .definition
            .fields
            .iter()
            .find(|field| field.name == name)?;
        let end = field.offset.checked_add(field.encoded_size)?;
        if let Some(bytes) = self.payload.get(field.offset..end) {
            return Some(Cow::Borrowed(bytes));
        }

        let mut bytes = vec![0; field.encoded_size];
        if let Some(available) = self.payload.get(field.offset..) {
            bytes[..available.len()].copy_from_slice(available);
        }
        Some(Cow::Owned(bytes))
    }
}

/// An immutable MAVLink dialect loaded from XML at runtime.
#[derive(Debug, Clone)]
pub struct DynamicDialect {
    messages: Arc<BTreeMap<u32, Arc<DynamicMessageDefinition>>>,
    version: Option<u8>,
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
        let mut parsed_files = HashSet::new();
        let profile = parser::parse_profile(definitions_dir, &source_file, &mut parsed_files)
            .map_err(DynamicDialectError::Xml)?;

        Self::from_profile(profile)
    }

    /// Return the loaded definition for `message_id`.
    #[must_use]
    pub fn message_definition(&self, message_id: u32) -> Option<&DynamicMessageDefinition> {
        self.messages.get(&message_id).map(Arc::as_ref)
    }

    /// Return all loaded message definitions, ordered by message ID.
    pub fn message_definitions(&self) -> impl Iterator<Item = &DynamicMessageDefinition> {
        self.messages.values().map(Arc::as_ref)
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

    /// Construct a runtime message from its name and named field values.
    ///
    /// Numeric values must match the field's primitive type.
    pub fn message_from_fields<K>(
        &self,
        version: MavlinkVersion,
        message_name: &str,
        fields: impl IntoIterator<Item = (K, DynamicValue)>,
    ) -> Result<DynamicMessage, DynamicMessageError>
    where
        K: AsRef<str>,
    {
        let definition = self
            .messages
            .values()
            .find(|definition| definition.name == message_name)
            .ok_or_else(|| DynamicMessageError::UnknownMessage {
                name: message_name.to_owned(),
            })?;
        let mut payload = vec![0; definition.encoded_size];

        for field in &definition.fields {
            write_default(
                &field.mavtype,
                field.enumeration.as_deref(),
                self.version,
                &mut payload[field.offset..field.offset + field.encoded_size],
            );
        }

        for (field_name, value) in fields {
            let field_name = field_name.as_ref();
            let field = definition
                .fields
                .iter()
                .find(|field| field.name == field_name)
                .ok_or_else(|| DynamicMessageError::UnknownField {
                    message: message_name.to_owned(),
                    field: field_name.to_owned(),
                })?;
            write_value(
                &field.mavtype,
                field.enumeration.as_deref(),
                &value,
                &mut payload[field.offset..field.offset + field.encoded_size],
            )
            .map_err(|reason| DynamicMessageError::InvalidValue {
                message: message_name.to_owned(),
                field: field_name.to_owned(),
                reason,
            })?;
        }

        payload.truncate(match version {
            MavlinkVersion::V1 => definition.base_payload_len,
            MavlinkVersion::V2 => mavlink_core::utils::remove_trailing_zeroes(&payload),
        });

        Ok(DynamicMessage {
            definition: Arc::clone(definition),
            version,
            payload,
        })
    }

    fn from_profile(profile: parser::MavProfile) -> Result<Self, DynamicDialectError> {
        let version = profile.version;
        let mut enums = HashMap::new();
        for (name, enumeration) in profile.enums {
            for restriction in
                std::iter::once(None).chain(enumeration.additional_primitives.iter().map(Some))
            {
                let entries = enumeration
                    .entries_with_values(restriction)
                    .map(|(entry, value)| (entry.name.clone(), value))
                    .collect();
                let mut primitive = restriction.or(enumeration.primitive.as_ref());
                while let Some(parser::MavType::Array(element, _)) = primitive {
                    primitive = Some(element);
                }
                enums.insert(
                    (name.clone(), restriction.cloned()),
                    Arc::new(DynamicEnumDefinition {
                        name: enumeration.name.clone(),
                        primitive: primitive.cloned(),
                        entries,
                    }),
                );
            }
        }

        let mut messages = BTreeMap::new();
        for message in profile.messages.into_values() {
            if message.id > 0x00ff_ffff {
                return Err(DynamicDialectError::MessageIdOutOfRange {
                    id: message.id,
                    name: message.name,
                });
            }

            let definition = Arc::new(runtime_message_definition(&message, &enums)?);
            if let Some(existing) = messages.insert(definition.id, Arc::clone(&definition)) {
                return Err(DynamicDialectError::DuplicateMessageId {
                    id: definition.id,
                    first: existing.name.clone(),
                    second: definition.name.clone(),
                });
            }
        }

        Ok(Self {
            messages: Arc::new(messages),
            version,
        })
    }
}

impl Dialect for DynamicDialect {
    type Message = DynamicMessage;
    type EncodeError = EncoderError;

    fn message_id(&self, message: &Self::Message) -> u32 {
        message.definition.id
    }

    fn extra_crc(&self, message_id: u32) -> Option<u8> {
        self.messages
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
            .messages
            .get(&message_id)
            .ok_or(ParserError::UnknownMessage { id: message_id })?;
        // MAVLink 2 extension fields may have been added by a newer version
        // of this dialect.
        if version == MavlinkVersion::V1 && payload.len() > definition.encoded_size {
            return Err(ParserError::InvalidPayloadSize {
                id: message_id,
                maximum: definition.encoded_size,
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
    ) -> Result<usize, Self::EncodeError> {
        let definition =
            self.messages
                .get(&message.definition.id)
                .ok_or(EncoderError::UnknownMessage {
                    id: message.definition.id,
                })?;
        if !Arc::ptr_eq(definition, &message.definition) {
            return Err(EncoderError::DialectMismatch {
                id: message.definition.id,
            });
        }
        let encoded_size = match version {
            MavlinkVersion::V1 => definition.base_payload_len,
            MavlinkVersion::V2 => message.payload.len(),
        };
        if encoded_size > payload.len() {
            return Err(EncoderError::InvalidPayloadSize {
                id: message.definition.id,
                maximum: payload.len(),
                actual: encoded_size,
            });
        }
        if version == MavlinkVersion::V1 {
            payload[..encoded_size].fill(0);
        }
        payload[..message.payload.len().min(encoded_size)]
            .copy_from_slice(&message.payload[..message.payload.len().min(encoded_size)]);
        Ok(encoded_size)
    }
}

fn runtime_message_definition(
    message: &parser::MavMessage,
    enums: &HashMap<(String, Option<parser::MavType>), Arc<DynamicEnumDefinition>>,
) -> Result<DynamicMessageDefinition, DynamicDialectError> {
    let mut fields = Vec::with_capacity(message.fields.len());
    let mut offset = 0;
    let mut base_payload_len = 0;

    for field in &message.fields {
        let encoded_size = field.mavtype.size();
        fields.push(DynamicField {
            name: field.name.clone(),
            primitive_type: field.mavtype.primitive_type(),
            mavtype: field.mavtype.clone(),
            enumeration: field
                .enumtype
                .as_ref()
                .and_then(|name| {
                    enums.get(&(
                        name.clone(),
                        field.is_undersized.then(|| field.mavtype.clone()),
                    ))
                })
                .cloned(),
            offset,
            encoded_size,
            is_extension: field.is_extension,
        });
        offset += encoded_size;
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
        encoded_size: offset,
        fields,
    })
}

#[cfg(all(test, feature = "dialect-common"))]
mod tests {
    use std::io::Cursor;

    use crate::{
        MAVLinkV2MessageRaw, MavHeader, MavlinkReader, MavlinkVersion, Message,
        dialects::common::{
            COMMAND_LONG_DATA, HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag,
            MavState, MavType,
        },
    };

    use super::DynamicDialect;

    fn common_dialect() -> DynamicDialect {
        DynamicDialect::from_xml_file(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/mavlink/message_definitions/v1.0/common.xml"
        ))
        .unwrap()
    }

    #[test]
    fn runtime_loaded_dialect_byte_rep_check() {
        let dialect = common_dialect();
        let header = MavHeader {
            sequence: 34,
            system_id: 1,
            component_id: 2,
        };
        let messages = [
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 42,
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
                command: MavCmd::MAV_CMD_NAV_WAYPOINT,
                target_system: 0,
                target_component: 0,
                confirmation: 0,
            }),
        ];

        for message in messages {
            let mut common_frame = MAVLinkV2MessageRaw::new();
            common_frame.serialize_message(header, &message);

            let mut reader = MavlinkReader::new(Cursor::new(common_frame.raw_bytes().to_vec()));
            let (decoded_header, dynamic_message) = reader
                .read_message_with_dialect(&dialect, MavlinkVersion::V2)
                .unwrap();

            if let MavMessage::COMMAND_LONG(expected) = &message {
                assert_eq!(
                    dynamic_message.field_bytes("command").unwrap().as_ref(),
                    (expected.command as u16).to_le_bytes()
                );
                assert_eq!(
                    dynamic_message.field_bytes("confirmation").unwrap(),
                    &[0][..]
                );
            }

            let mut dynamic_frame = MAVLinkV2MessageRaw::new();
            dynamic_frame
                .serialize_message_with_dialect(decoded_header, &dialect, &dynamic_message)
                .unwrap();

            assert_eq!(dynamic_frame.raw_bytes(), common_frame.raw_bytes());
        }
    }

    #[test]
    fn construct_message() {
        let dialect = common_dialect();
        let expected = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 42,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            ..Default::default()
        });

        for version in [MavlinkVersion::V1, MavlinkVersion::V2] {
            let message = dialect
                .message_from_fields(
                    version,
                    "HEARTBEAT",
                    [
                        ("custom_mode", 42u32.into()),
                        ("mavtype", "MAV_TYPE_QUADROTOR".into()),
                    ],
                )
                .unwrap();
            let mut payload = [0; 255];
            let length = expected.ser(version, &mut payload);
            assert_eq!(message.payload(), &payload[..length]);
        }

        for (field, value) in [
            ("custom_mode", 42u8.into()),
            ("mavtype", 2u32.into()),
            ("base_mode", 1i8.into()),
        ] {
            assert!(
                dialect
                    .message_from_fields(MavlinkVersion::V2, "HEARTBEAT", [(field, value)])
                    .is_err()
            );
        }
    }

    #[test]
    #[cfg(feature = "serde")]
    fn serialize_message() {
        use crate::dialects::common::{CAMERA_IMAGE_CAPTURED_DATA, MavBool};

        let dialect = common_dialect();
        let expected = MavMessage::CAMERA_IMAGE_CAPTURED(CAMERA_IMAGE_CAPTURED_DATA {
            capture_result: MavBool::from_bits_retain(u8::MAX as _),
            file_url: "image.jpg".into(),
            q: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        });
        let expected = serde_json::to_value(expected).unwrap();

        for flags in [
            (-1i8).into(),
            expected["capture_result"].as_str().unwrap().into(),
        ] {
            let message = dialect
                .message_from_fields(
                    MavlinkVersion::V2,
                    "CAMERA_IMAGE_CAPTURED",
                    [
                        ("capture_result", flags),
                        ("file_url", "image.jpg".into()),
                        ("q", [1.0f32, 0.0, 0.0, 0.0].into()),
                    ],
                )
                .unwrap();
            assert_eq!(serde_json::to_value(message).unwrap(), expected);
        }
    }
}
