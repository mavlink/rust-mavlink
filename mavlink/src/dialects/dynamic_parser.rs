//! Field value conversion for runtime loaded dialects.

use mavlink_bindgen::parser::MavType;
use mavlink_core::bytes_mut::BytesMut;

use super::dynamic::DynamicEnumDefinition;

/// A field value used to construct a runtime loaded MAVLink message.
#[derive(Debug, Clone, PartialEq)]
pub enum DynamicValue {
    /// Unsigned integer value.
    Unsigned(u64),
    /// Signed integer value.
    Signed(i64),
    /// Floating-point value.
    Float(f64),
    /// Character array, enum name or `|` separated bitmask names.
    String(String),
    /// Array with the field's declared length.
    Array(Vec<Self>),
}

impl From<u8> for DynamicValue {
    fn from(value: u8) -> Self {
        Self::Unsigned(value.into())
    }
}

impl From<u16> for DynamicValue {
    fn from(value: u16) -> Self {
        Self::Unsigned(value.into())
    }
}

impl From<u32> for DynamicValue {
    fn from(value: u32) -> Self {
        Self::Unsigned(value.into())
    }
}

impl From<u64> for DynamicValue {
    fn from(value: u64) -> Self {
        Self::Unsigned(value)
    }
}

impl From<i8> for DynamicValue {
    fn from(value: i8) -> Self {
        Self::Signed(value.into())
    }
}

impl From<i16> for DynamicValue {
    fn from(value: i16) -> Self {
        Self::Signed(value.into())
    }
}

impl From<i32> for DynamicValue {
    fn from(value: i32) -> Self {
        Self::Signed(value.into())
    }
}

impl From<i64> for DynamicValue {
    fn from(value: i64) -> Self {
        Self::Signed(value)
    }
}

impl From<f32> for DynamicValue {
    fn from(value: f32) -> Self {
        Self::Float(value.into())
    }
}

impl From<f64> for DynamicValue {
    fn from(value: f64) -> Self {
        Self::Float(value)
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
            let resolved;
            let element = if let DynamicValue::String(name) = element {
                resolved = DynamicValue::Unsigned(parse_enum_value(name, enumeration)?);
                &resolved
            } else {
                element
            };
            // Array elements stay primitive, as in generated dialects.
            write_value(element_type, None, element, output)?;
        }
        return Ok(());
    }

    if let MavType::CharArray(length) = mavtype {
        return write_char_array(*length, value, output);
    }

    if let Some(enumeration) = enumeration {
        let mut value: i128 = integer_value(value, Some(enumeration))?;
        if let Some(primitive) = &enumeration.primitive {
            let bits_value = cast_integer(primitive, value)
                .ok_or_else(|| "bitmasks require an integer type".to_owned())?;
            let bits = primitive.size() * 8;
            if value < -(1_i128 << (bits - 1)) || value >= (1_i128 << bits) {
                return Err(format!("bitmask value does not fit in {bits} bits"));
            }
            value = bits_value;
        }
        let value = cast_integer(mavtype, value)
            .ok_or_else(|| "enums require an integer field type".to_owned())?;
        output.copy_from_slice(&value.to_le_bytes()[..output.len()]);
        return Ok(());
    }

    let mut output = BytesMut::new(output);
    match mavtype {
        MavType::UInt8 | MavType::UInt8MavlinkVersion | MavType::Char => {
            output.put_u8(integer_value(value, enumeration)?);
        }
        MavType::UInt16 => output.put_u16_le(integer_value(value, enumeration)?),
        MavType::UInt32 => output.put_u32_le(integer_value(value, enumeration)?),
        MavType::UInt64 => output.put_u64_le(integer_value(value, enumeration)?),
        MavType::Int8 => output.put_i8(integer_value(value, enumeration)?),
        MavType::Int16 => output.put_i16_le(integer_value(value, enumeration)?),
        MavType::Int32 => output.put_i32_le(integer_value(value, enumeration)?),
        MavType::Int64 => output.put_i64_le(integer_value(value, enumeration)?),
        MavType::Float => output.put_f32_le(match value {
            DynamicValue::Unsigned(value) => *value as f32,
            DynamicValue::Signed(value) => *value as f32,
            _ => float_value(value)? as f32,
        }),
        MavType::Double => output.put_f64_le(float_value(value)?),
        MavType::CharArray(_) | MavType::Array(_, _) => unreachable!(),
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
                *byte = integer_value(value, None)?;
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

fn integer_value<T: TryFrom<i128>>(
    value: &DynamicValue,
    enumeration: Option<&DynamicEnumDefinition>,
) -> Result<T, String> {
    // i128 holds every supported signed and unsigned integer.
    let value = match value {
        DynamicValue::Unsigned(value) => i128::from(*value),
        DynamicValue::Signed(value) => i128::from(*value),
        DynamicValue::String(value) => i128::from(parse_enum_value(value, enumeration)?),
        _ => return Err("expected an integer, enum name, or bitmask names".to_owned()),
    };
    if let Some(enumeration) = enumeration.filter(|enumeration| enumeration.primitive.is_none()) {
        if value < 0 {
            return Err("enum values cannot be negative".to_owned());
        }
        if !enumeration
            .entries
            .iter()
            .any(|(_, entry)| i128::from(*entry) == value)
        {
            return Err(format!("{value} is not a valid enum value"));
        }
    }
    T::try_from(value).map_err(|_| format!("value does not fit in {}", std::any::type_name::<T>()))
}

fn float_value(value: &DynamicValue) -> Result<f64, String> {
    match value {
        DynamicValue::Float(value) => Ok(*value),
        DynamicValue::Unsigned(value) => Ok(*value as f64),
        DynamicValue::Signed(value) => Ok(*value as f64),
        _ => Err("expected a numeric value".to_owned()),
    }
}

fn parse_enum_value(
    value: &str,
    enumeration: Option<&DynamicEnumDefinition>,
) -> Result<u64, String> {
    let enumeration = enumeration.ok_or_else(|| "field has no associated enum".to_owned())?;
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
    use super::{DynamicEnumDefinition, MavType, write_value};

    #[test]
    fn enum_arrays_check_element_range() {
        let enumeration = DynamicEnumDefinition {
            name: "TestEnum".into(),
            primitive: None,
            entries: vec![("LARGE".into(), 256)],
        };
        let mavtype = MavType::Array(Box::new(MavType::UInt8), 1);
        assert!(write_value(&mavtype, Some(&enumeration), &["LARGE"].into(), &mut [0]).is_err());
    }
}
