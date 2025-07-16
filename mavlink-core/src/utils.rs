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

/// A trait very similar to [`Default`] but is only implemented for the equivalent Rust types to
/// `MavType`s.
///
/// This is only needed because rust doesn't currently implement `Default` for arrays
/// of all sizes. In particular this trait is only ever used when the "serde" feature is enabled.
/// For more information, check out [this issue](https://users.rust-lang.org/t/issue-for-derives-for-arrays-greater-than-size-32/59055/3).
pub trait RustDefault: Copy {
    fn rust_default() -> Self;
}

impl<T: RustDefault, const N: usize> RustDefault for [T; N] {
    #[inline(always)]
    fn rust_default() -> Self {
        let val: T = RustDefault::rust_default();
        [val; N]
    }
}

macro_rules! impl_rust_default {
    ($($t:ty => $val:expr),* $(,)?) => {
        $(impl RustDefault for $t {
            #[inline(always)]
            fn rust_default() -> Self { $val }
        })*
    };
}

impl_rust_default! {
    u8 => 0,
    i8 => 0,
    u16 => 0,
    i16 => 0,
    u32 => 0,
    i32 => 0,
    u64 => 0,
    i64 => 0,
    f32 => 0.0,
    f64 => 0.0,
    char => '\0',
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_remove_trailing_zeroes_empty_slice() {
        remove_trailing_zeroes(&[]);
    }
}
