#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

/// Removes the trailing zeroes in the payload
///
/// # Note:
///
/// There must always be at least one remaining byte even if it is a
/// zero byte.
#[allow(dead_code)]
pub(crate) fn remove_trailing_zeroes(buf: &mut Vec<u8>) {
    while let Some(&0) = buf.last() {
        if buf.len() <= 1 {
            break;
        }
        buf.pop();
    }
}
