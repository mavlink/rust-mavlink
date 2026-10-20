//! Serialization support for runtime loaded MAVLink messages.

use mavlink_bindgen::parser::MavType;
use serde::{
    Serialize, Serializer,
    ser::{Error as _, SerializeMap, SerializeTuple},
};

use super::{
    dynamic::{DynamicEnumDefinition, DynamicMessage},
    dynamic_parser::{cast_integer, enum_value_fits},
};

impl Serialize for DynamicMessage {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let definition = self.definition();
        let mut message = serializer.serialize_map(Some(definition.fields().len() + 1))?;
        message.serialize_entry("type", definition.name())?;
        for field in definition.fields() {
            message.serialize_entry(
                field.name(),
                &SerializableField {
                    mavtype: &field.mavtype,
                    enumeration: field.enumeration.as_deref(),
                    bytes: available_bytes(self.payload(), field.offset(), field.encoded_size()),
                },
            )?;
        }
        message.end()
    }
}

struct SerializableField<'a> {
    mavtype: &'a MavType,
    enumeration: Option<&'a DynamicEnumDefinition>,
    bytes: &'a [u8],
}

impl Serialize for SerializableField<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if let MavType::Array(element_type, length) = self.mavtype {
            let element_size = element_type.size();
            let mut sequence = serializer.serialize_tuple(*length)?;
            for index in 0..*length {
                let offset = index * element_size;
                sequence.serialize_element(&Self {
                    mavtype: element_type,
                    enumeration: None,
                    bytes: available_bytes(self.bytes, offset, element_size),
                })?;
            }
            return sequence.end();
        }

        if let MavType::CharArray(length) = self.mavtype {
            let length = (*length).min(self.bytes.len());
            let bytes = &self.bytes[..length];
            let bytes = &bytes[..bytes.iter().position(|byte| *byte == 0).unwrap_or(length)];
            return serializer.serialize_str(std::str::from_utf8(bytes).map_err(S::Error::custom)?);
        }

        if let Some(enumeration) = self.enumeration {
            let value = read_integer(self.mavtype, self.bytes).ok_or_else(|| {
                S::Error::custom(format!(
                    "enum {} must use an integer field type",
                    enumeration.name
                ))
            })?;

            if let Some(primitive) = &enumeration.primitive {
                let value = cast_integer(primitive, value)
                    .ok_or_else(|| S::Error::custom("bitmask must use an integer type"))?;
                #[cfg(not(feature = "serde-raw-ints"))]
                if serializer.is_human_readable() {
                    let bits = value & (u64::MAX >> (64 - primitive.size() * 8));
                    return serializer.serialize_str(&bitmask_names(enumeration, bits));
                }
                return SerializableField {
                    mavtype: primitive,
                    enumeration: None,
                    bytes: &value.to_le_bytes()[..primitive.size()],
                }
                .serialize(serializer);
            }

            #[cfg(feature = "serde-raw-ints")]
            {
                return SerializableField {
                    mavtype: self.mavtype,
                    enumeration: None,
                    bytes: self.bytes,
                }
                .serialize(serializer);
            }
            #[cfg(not(feature = "serde-raw-ints"))]
            {
                let (name, _) = enumeration
                    .entries
                    .iter()
                    .find(|(_, entry_value)| {
                        *entry_value == value && enum_value_fits(self.mavtype, value)
                    })
                    .ok_or_else(|| {
                        S::Error::custom(format!(
                            "invalid enum value for {}: {value}",
                            enumeration.name
                        ))
                    })?;
                let mut variant = serializer.serialize_map(Some(1))?;
                variant.serialize_entry("type", name)?;
                return variant.end();
            }
        }

        match self.mavtype {
            MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => {
                serializer.serialize_u8(self.bytes.first().copied().unwrap_or_default())
            }
            MavType::UInt16 => serializer.serialize_u16(u16::from_le_bytes(padded(self.bytes))),
            MavType::UInt32 => serializer.serialize_u32(u32::from_le_bytes(padded(self.bytes))),
            MavType::UInt64 => serializer.serialize_u64(u64::from_le_bytes(padded(self.bytes))),
            MavType::Int8 => {
                serializer.serialize_i8(self.bytes.first().copied().unwrap_or_default() as i8)
            }
            MavType::Int16 => serializer.serialize_i16(i16::from_le_bytes(padded(self.bytes))),
            MavType::Int32 => serializer.serialize_i32(i32::from_le_bytes(padded(self.bytes))),
            MavType::Int64 => serializer.serialize_i64(i64::from_le_bytes(padded(self.bytes))),
            MavType::Float => serializer.serialize_f32(f32::from_le_bytes(padded(self.bytes))),
            MavType::Double => serializer.serialize_f64(f64::from_le_bytes(padded(self.bytes))),
            MavType::CharArray(_) | MavType::Array(_, _) => unreachable!(),
        }
    }
}

fn read_integer(mavtype: &MavType, bytes: &[u8]) -> Option<u64> {
    cast_integer(mavtype, u64::from_le_bytes(padded(bytes)))
}

fn bitmask_names(enumeration: &DynamicEnumDefinition, value: u64) -> String {
    let mut names = Vec::new();
    let mut remaining = value;

    for (name, bits) in &enumeration.entries {
        let bits = *bits;
        if bits != 0 && value & bits == bits && remaining & bits != 0 {
            names.push(name.clone());
            remaining &= !bits;
        }
    }

    if remaining != 0 {
        names.push(format!("0x{remaining:x}"));
    }

    names.join(" | ")
}

fn available_bytes(bytes: &[u8], offset: usize, length: usize) -> &[u8] {
    let bytes = bytes.get(offset..).unwrap_or_default();
    &bytes[..bytes.len().min(length)]
}

fn padded<const N: usize>(bytes: &[u8]) -> [u8; N] {
    let mut padded = [0; N];
    let length = N.min(bytes.len());
    padded[..length].copy_from_slice(&bytes[..length]);
    padded
}

#[cfg(test)]
mod tests {
    use super::{DynamicEnumDefinition, MavType, SerializableField};
    use crate::dialects::dynamic_parser::write_value;

    #[test]
    #[cfg(not(feature = "serde-raw-ints"))]
    fn bitmask_with_different_field_widths() {
        let mut enumeration = DynamicEnumDefinition {
            name: "Flags".into(),
            primitive: Some(MavType::Int16),
            entries: vec![("ENABLED".into(), 1)],
        };
        let json = serde_json::to_value(SerializableField {
            mavtype: &MavType::Int8,
            enumeration: Some(&enumeration),
            bytes: &[255],
        })
        .unwrap();
        let names = json.as_str().unwrap().into();
        let mut output = [0];
        write_value(&MavType::Int8, Some(&enumeration), &names, &mut output).unwrap();
        assert_eq!(output, [255]);

        enumeration.primitive = Some(MavType::UInt8);
        for value in [256u16.into(), names] {
            assert!(
                write_value(&MavType::UInt16, Some(&enumeration), &value, &mut [0; 2]).is_err()
            );
        }
    }
}
