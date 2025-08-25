use core::ops::{Deref, DerefMut};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(feature = "arbitrary")]
use arbitrary::{Arbitrary, Unstructured};

/// Abstraction around a byte array that represents a string.
///
/// MAVLink encodes strings as C char arrays and the handling is field dependent.
/// This abstration allows to choose if one wants to handle the field as
/// a raw byte array or if one wants the convenience of a str that stops at the first null byte.
///
/// # Example
/// ```
/// use mavlink_core::types::CharArray;
///
/// let data = [0x48, 0x45, 0x4c, 0x4c, 0x4f, 0x00, 0x57, 0x4f, 0x52, 0x4c, 0x44, 0x00, 0x00, 0x00];
/// let ca = CharArray::new(data);
/// assert_eq!(ca.to_str(), "HELLO");
/// ```
#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct CharArray<const N: usize> {
    #[cfg_attr(feature = "serde", serde(with = "serde_arrays"))]
    data: [u8; N],

    #[cfg_attr(feature = "serde", serde(skip))]
    str_len: usize,
}

impl<const N: usize> CharArray<N> {
    pub const fn new(data: [u8; N]) -> Self {
        // Note: The generated code uses this in const contexts, so this is a const fn
        // and so we can't use iterators or other fancy stuff unfortunately.
        let mut first_null = N;
        let mut i = 0;
        loop {
            if i >= N {
                break;
            }
            if data[i] == 0 {
                first_null = i;
                break;
            }
            i += 1;
        }
        Self {
            data,
            str_len: first_null,
        }
    }

    /// Get the string representation of the char array.
    /// Returns the string stopping at the first null byte and if the string is not valid utf8
    /// the returned string will be empty.
    pub fn to_str(&self) -> &str {
        std::str::from_utf8(&self.data[..self.str_len]).unwrap_or("")
    }
}

impl<const N: usize> Deref for CharArray<N> {
    type Target = [u8; N];

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<const N: usize> DerefMut for CharArray<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

impl<'a, const N: usize> IntoIterator for &'a CharArray<N> {
    type Item = &'a u8;
    type IntoIter = core::slice::Iter<'a, u8>;

    fn into_iter(self) -> Self::IntoIter {
        self.data.iter()
    }
}

impl<const N: usize> From<[u8; N]> for CharArray<N> {
    fn from(data: [u8; N]) -> Self {
        Self::new(data)
    }
}

impl<const N: usize> From<CharArray<N>> for [u8; N] {
    fn from(value: CharArray<N>) -> Self {
        value.data
    }
}

#[cfg(feature = "serde")]
impl<'de, const N: usize> Deserialize<'de> for CharArray<N> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data: [u8; N] = serde_arrays::deserialize(deserializer)?;
        Ok(Self::new(data))
    }
}

#[cfg(feature = "arbitrary")]
impl<'a, const N: usize> Arbitrary<'a> for CharArray<N> {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        let mut data = [0u8; N];
        u.fill_buffer(&mut data)?;
        Ok(CharArray::new(data))
    }
}

#[cfg(test)]
mod tests {
    use super::CharArray;

    #[test]
    fn char_array_to_str_handles_no_nulls() {
        let data = *b"HELLOWORLD";
        let ca = CharArray::new(data);
        assert_eq!(ca.len(), 10);
        assert_eq!(ca.to_str(), "HELLOWORLD");
    }

    #[test]
    fn char_array_to_str_trims_after_first_null() {
        let mut data = [0u8; 10];
        data[..3].copy_from_slice(b"abc");
        // data[3..] are zeros
        let ca = CharArray::new(data);
        assert_eq!(ca.len(), 10);
        assert_eq!(ca.to_str(), "abc");
    }
}
