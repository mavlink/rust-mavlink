/// Removes the trailing zeroes in the payload
///
/// # Note:
///
/// There must always be at least one remaining byte even if it is a
/// zero byte.
pub fn remove_trailing_zeroes(data: &[u8]) -> usize {
    let mut len = data.len();

    for b in data[1..].iter().rev() {
        if *b != 0 {
            break;
        }

        len -= 1;
    }

    len
}

/// A trait very similar to `Default` but is only implemented for the equivalent Rust types to
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

impl RustDefault for u8 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for i8 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for u16 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for i16 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for u32 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for i32 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for u64 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for i64 {
    #[inline(always)]
    fn rust_default() -> Self {
        0
    }
}

impl RustDefault for char {
    #[inline(always)]
    fn rust_default() -> Self {
        '\0'
    }
}

impl RustDefault for f32 {
    #[inline(always)]
    fn rust_default() -> Self {
        0.0
    }
}

impl RustDefault for f64 {
    #[inline(always)]
    fn rust_default() -> Self {
        0.0
    }
}
