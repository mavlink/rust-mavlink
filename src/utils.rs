use crate::{MAX_FRAME_SIZE, bytes_mut::BytesMut};

/// Removes the trailing zeroes in the payload
///
/// # Note:
///
/// There must always be at least one remaining byte even if it is a
/// zero byte.
#[allow(dead_code)]
pub(crate) fn remove_trailing_zeroes(buf: &mut BytesMut<MAX_FRAME_SIZE>) {
    let data = &**buf;
    let mut len = data.len();

    for b in data[1..].iter().rev() {
        if *b != 0 {
            break;
        }

        len -= 1;
    }

    buf.set_len(len);
}
