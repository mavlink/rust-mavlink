//! Field value encoding for runtime loaded dialects.

use mavlink_bindgen::parser::MavType;
use mavlink_core::bytes_mut::BytesMut;

use super::dynamic::DynamicEnumDefinition;

/// A field value used to construct a runtime loaded MAVLink message.
///
/// Numeric values must match the field's primitive type, including array elements.
#[derive(Debug, Clone, PartialEq)]
pub enum DynamicValue {
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    F32(f32),
    F64(f64),
    /// Character array, enum name or `|` separated bitmask names.
    String(String),
    /// Array with the field's declared length.
    Array(Vec<Self>),
}

impl From<u8> for DynamicValue {
    fn from(value: u8) -> Self {
        Self::U8(value)
    }
}

impl From<u16> for DynamicValue {
    fn from(value: u16) -> Self {
        Self::U16(value)
    }
}

impl From<u32> for DynamicValue {
    fn from(value: u32) -> Self {
        Self::U32(value)
    }
}

impl From<u64> for DynamicValue {
    fn from(value: u64) -> Self {
        Self::U64(value)
    }
}

impl From<i8> for DynamicValue {
    fn from(value: i8) -> Self {
        Self::I8(value)
    }
}

impl From<i16> for DynamicValue {
    fn from(value: i16) -> Self {
        Self::I16(value)
    }
}

impl From<i32> for DynamicValue {
    fn from(value: i32) -> Self {
        Self::I32(value)
    }
}

impl From<i64> for DynamicValue {
    fn from(value: i64) -> Self {
        Self::I64(value)
    }
}

impl From<f32> for DynamicValue {
    fn from(value: f32) -> Self {
        Self::F32(value)
    }
}

impl From<f64> for DynamicValue {
    fn from(value: f64) -> Self {
        Self::F64(value)
    }
}

impl From<String> for DynamicValue {
    fn from(value: String) -> Self {
        Self::String(value)
    }
}

impl From<&str> for DynamicValue {
    fn from(value: &str) -> Self {
        Self::String(value.into())
    }
}

impl<T: Into<Self>> From<Vec<T>> for DynamicValue {
    fn from(value: Vec<T>) -> Self {
        Self::Array(value.into_iter().map(Into::into).collect())
    }
}

impl<T: Into<Self>, const N: usize> From<[T; N]> for DynamicValue {
    fn from(value: [T; N]) -> Self {
        Self::Array(value.into_iter().map(Into::into).collect())
    }
}

pub(super) fn write_value(
    mavtype: &MavType,
    enumeration: Option<&DynamicEnumDefinition>,
    value: &DynamicValue,
    output: &mut [u8],
) -> Result<(), String> {
    if let MavType::Array(element_type, length) = mavtype {
        let DynamicValue::Array(values) = value else {
            return Err(format!("expected an array of {length} values"));
        };
        if values.len() != *length {
            return Err(format!(
                "expected an array of {length} values, got {}",
                values.len()
            ));
        }

        let element_size = element_type.size();
        for (element, output) in values.iter().zip(output.chunks_exact_mut(element_size)) {
            // Array elements stay primitive, as in generated dialects.
            write_value(element_type, None, element, output)?;
        }
        return Ok(());
    }

    if let MavType::CharArray(length) = mavtype {
        return write_char_array(*length, value, output);
    }

    if let DynamicValue::String(name) = value {
        let enumeration = enumeration.ok_or_else(|| "field has no associated enum".to_owned())?;
        let value = parse_enum_value(name, enumeration)?;
        let primitive = enumeration.primitive.as_ref().unwrap_or(mavtype);
        let fits = if enumeration.primitive.is_some() {
            value <= u64::MAX >> (64 - primitive.size() * 8)
        } else {
            cast_integer(mavtype, value.into()) == Some(value.into())
        };
        if !fits {
            return Err(format!(
                "enum value does not fit in {}",
                primitive.rust_type()
            ));
        }
        let value = cast_integer(primitive, value.into())
            .ok_or_else(|| "enums require an integer type".to_owned())?;
        output.copy_from_slice(&value.to_le_bytes()[..output.len()]);
        return Ok(());
    }

    if let Some(enumeration) = enumeration {
        let value = integer_value(value)?;
        if let Some(primitive) = &enumeration.primitive {
            let encoded =
                cast_integer(primitive, value).and_then(|bits| cast_integer(mavtype, bits));
            if encoded != Some(value) {
                return Err(format!(
                    "bitmask value cannot be represented by {}",
                    primitive.rust_type()
                ));
            }
        } else if !enumeration
            .entries
            .iter()
            .any(|(_, entry)| i128::from(*entry) == value)
        {
            return Err(format!("{value} is not a valid enum value"));
        }
    }

    let mut output = BytesMut::new(output);
    match (mavtype, value) {
        (
            MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char,
            DynamicValue::U8(value),
        ) => {
            output.put_u8(*value);
        }
        (MavType::UInt16, DynamicValue::U16(value)) => output.put_u16_le(*value),
        (MavType::UInt32, DynamicValue::U32(value)) => output.put_u32_le(*value),
        (MavType::UInt64, DynamicValue::U64(value)) => output.put_u64_le(*value),
        (MavType::Int8, DynamicValue::I8(value)) => output.put_i8(*value),
        (MavType::Int16, DynamicValue::I16(value)) => output.put_i16_le(*value),
        (MavType::Int32, DynamicValue::I32(value)) => output.put_i32_le(*value),
        (MavType::Int64, DynamicValue::I64(value)) => output.put_i64_le(*value),
        (MavType::Float, DynamicValue::F32(value)) => output.put_f32_le(*value),
        (MavType::Double, DynamicValue::F64(value)) => output.put_f64_le(*value),
        _ => return Err(format!("expected {}", mavtype.rust_type())),
    }
    Ok(())
}

pub(super) fn write_default(
    mavtype: &MavType,
    enumeration: Option<&DynamicEnumDefinition>,
    dialect_version: Option<u8>,
    output: &mut [u8],
) {
    if let MavType::Array(element_type, _) = mavtype {
        let element_size = element_type.size();
        for output in output.chunks_exact_mut(element_size) {
            write_default(element_type, None, dialect_version, output);
        }
    } else if let Some((_, value)) = enumeration.and_then(|enumeration| enumeration.entries.first())
    {
        for (output, value) in output.iter_mut().zip(value.to_le_bytes()) {
            *output = value;
        }
    } else if matches!(mavtype, MavType::UInt8MavlinkVersion) {
        output[0] = dialect_version.unwrap_or(0);
    }
}

pub(super) fn cast_integer(mavtype: &MavType, value: i128) -> Option<i128> {
    Some(match mavtype {
        MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => i128::from(value as u8),
        MavType::UInt16 => i128::from(value as u16),
        MavType::UInt32 => i128::from(value as u32),
        MavType::UInt64 => i128::from(value as u64),
        MavType::Int8 => i128::from(value as i8),
        MavType::Int16 => i128::from(value as i16),
        MavType::Int32 => i128::from(value as i32),
        MavType::Int64 => i128::from(value as i64),
        _ => return None,
    })
}

fn write_char_array(length: usize, value: &DynamicValue, output: &mut [u8]) -> Result<(), String> {
    match value {
        DynamicValue::String(value) => {
            let value = value.as_bytes();
            let copied = value.len().min(length);
            output.fill(0);
            output[..copied].copy_from_slice(&value[..copied]);
            Ok(())
        }
        DynamicValue::Array(values) if values.len() == length => {
            for (value, byte) in values.iter().zip(output) {
                let DynamicValue::U8(value) = value else {
                    return Err("expected u8 character array elements".to_owned());
                };
                *byte = *value;
            }
            Ok(())
        }
        DynamicValue::Array(values) => Err(format!(
            "expected a character array of {length} values, got {}",
            values.len()
        )),
        _ => Err(format!("expected a string or array of {length} bytes")),
    }
}

fn integer_value(value: &DynamicValue) -> Result<i128, String> {
    Ok(match value {
        DynamicValue::U8(value) => i128::from(*value),
        DynamicValue::U16(value) => i128::from(*value),
        DynamicValue::U32(value) => i128::from(*value),
        DynamicValue::U64(value) => i128::from(*value),
        DynamicValue::I8(value) => i128::from(*value),
        DynamicValue::I16(value) => i128::from(*value),
        DynamicValue::I32(value) => i128::from(*value),
        DynamicValue::I64(value) => i128::from(*value),
        _ => return Err("expected an integer or enum name".to_owned()),
    })
}

fn parse_enum_value(value: &str, enumeration: &DynamicEnumDefinition) -> Result<u64, String> {
    if enumeration.primitive.is_some() {
        if value.trim().is_empty() {
            return Ok(0);
        }
        value.split('|').try_fold(0, |bits, name| {
            let name = name.trim();
            let value = if let Some(hex) = name.strip_prefix("0x") {
                u64::from_str_radix(hex, 16).map_err(|_| format!("invalid bitmask value {name}"))?
            } else {
                enum_entry_value(enumeration, name)?
            };
            Ok(bits | value)
        })
    } else {
        enum_entry_value(enumeration, value)
    }
}

fn enum_entry_value(enumeration: &DynamicEnumDefinition, name: &str) -> Result<u64, String> {
    enumeration
        .entries
        .iter()
        .find_map(|(entry, value)| (entry == name).then_some(*value))
        .ok_or_else(|| format!("unknown entry {name} in enum {}", enumeration.name))
}

#[cfg(test)]
mod tests {
    use super::{MavType, write_value};

    #[test]
    fn numeric_types_must_match() {
        let values = [
            (MavType::UInt8, 1u8.into()),
            (MavType::UInt16, 1u16.into()),
            (MavType::UInt32, 1u32.into()),
            (MavType::UInt64, 1u64.into()),
            (MavType::Int8, 1i8.into()),
            (MavType::Int16, 1i16.into()),
            (MavType::Int32, 1i32.into()),
            (MavType::Int64, 1i64.into()),
            (MavType::Float, 1.0f32.into()),
            (MavType::Double, 1.0f64.into()),
        ];
        for (field_type, _) in &values {
            for (value_type, value) in &values {
                let result = write_value(field_type, None, value, &mut [0; 8][..field_type.size()]);
                assert_eq!(result.is_ok(), field_type == value_type);
            }
        }
    }

    #[test]
    fn array_types_must_match() {
        let mavtype = MavType::Array(Box::new(MavType::UInt8), 1);
        for mavtype in [mavtype, MavType::CharArray(1)] {
            assert!(write_value(&mavtype, None, &[1u8].into(), &mut [0]).is_ok());
            assert!(write_value(&mavtype, None, &[1u16].into(), &mut [0]).is_err());
        }
    }
}
