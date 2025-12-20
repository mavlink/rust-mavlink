//! Utilities for processing MAVLink messages

/// Removes the trailing zeroes in the payload
///
/// # Note:
///
/// There must always be at least one remaining byte even if it is a
/// zero byte.
pub fn remove_trailing_zeroes(data: &[u8]) -> usize {
    let mut len = data.len();

    while len > 1 && data[len - 1] == 0 {
        len -= 1;
    }

    len
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_remove_trailing_zeroes_empty_slice() {
        remove_trailing_zeroes(&[]);
    }
}

#[cfg(feature = "serde")]
pub mod nulstr {
    use serde::de::Deserializer;
    use serde::ser::Serializer;
    use serde::Deserialize;
    use std::str;

    pub fn serialize<S, const N: usize>(value: &[u8; N], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let nul_pos = value.iter().position(|&b| b == 0).unwrap_or(N);
        let s = str::from_utf8(&value[..nul_pos]).map_err(serde::ser::Error::custom)?;
        serializer.serialize_str(s)
    }

    pub fn deserialize<'de, D, const N: usize>(deserializer: D) -> Result<[u8; N], D::Error>
    where
        D: Deserializer<'de>,
    {
        let s: String = Deserialize::deserialize(deserializer)?;
        let mut buf = [0u8; N];
        let bytes = s.as_bytes();
        let len = bytes.len().min(N);
        buf[..len].copy_from_slice(&bytes[..len]);
        Ok(buf)
    }
}
