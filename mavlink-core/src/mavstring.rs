use std::{
    convert::TryFrom,
    fmt,
    ops::{Deref, DerefMut},
};

use arrayvec::ArrayString;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::error::MavStringError;

/// A MAVLink-compatible string: null-terminated, max length N, ASCII/UTF-8
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavString<const CAP: usize>(pub ArrayString<CAP>);

impl<const CAP: usize> MavString<CAP> {
    pub fn new() -> Self {
        Self(ArrayString::<CAP>::new())
    }

    pub const fn new_const() -> Self {
        Self(ArrayString::<CAP>::new_const())
    }
}

impl<const CAP: usize> Default for MavString<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a, const CAP: usize> TryFrom<&'a str> for MavString<CAP> {
    type Error = MavStringError;

    fn try_from(s: &'a str) -> Result<Self, Self::Error> {
        ArrayString::try_from(s)
            .map(Self)
            .map_err(|_| MavStringError::TooLong)
    }
}

impl<const CAP: usize> TryFrom<String> for MavString<CAP> {
    type Error = MavStringError;

    fn try_from(s: String) -> Result<Self, Self::Error> {
        ArrayString::try_from(s.as_str())
            .map(Self)
            .map_err(|_| MavStringError::TooLong)
    }
}

impl<const CAP: usize> Deref for MavString<CAP> {
    type Target = ArrayString<CAP>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const CAP: usize> DerefMut for MavString<CAP> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const CAP: usize> fmt::Debug for MavString<CAP> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&self.0, f)
    }
}

impl<const CAP: usize> fmt::Display for MavString<CAP> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(&self.0, f)
    }
}

#[cfg(feature = "arbitrary")]
use arbitrary::{Arbitrary, Unstructured};

#[cfg(feature = "arbitrary")]
impl<const CAP: usize> Arbitrary<'_> for MavString<CAP> {
    fn arbitrary(u: &mut Unstructured<'_>) -> arbitrary::Result<Self> {
        let s: &str = u.arbitrary()?;

        let filtered = s
            .chars()
            .filter(|&c| c != '\0' && c.is_ascii())
            .take(CAP)
            .collect::<String>();

        Self::try_from(filtered.as_str()).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
