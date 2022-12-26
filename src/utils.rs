/// Removes the trailing zeroes in the payload
///
/// # Note:
///
/// There must always be at least one remaining byte even if it is a
/// zero byte.
pub(crate) fn remove_trailing_zeroes(data: &mut [u8]) -> usize {
    let mut len = data.len();

    for b in data[1..].iter().rev() {
        if *b != 0 {
            break;
        }

        len -= 1;
    }

    len
}
