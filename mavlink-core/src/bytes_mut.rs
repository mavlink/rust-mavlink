pub struct BytesMut<'a> {
    data: &'a mut [u8],
    len: usize,
}

impl<'a> BytesMut<'a> {
    pub fn new(data: &'a mut [u8]) -> Self {
        Self { data, len: 0 }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[inline]
    pub fn remaining(&self) -> usize {
        self.data.len() - self.len
    }

    #[inline]
    fn check_remaining(&self, count: usize) {
        assert!(
            self.remaining() >= count,
            "write buffer overflow; remaining {} bytes, try add {count} bytes",
            self.remaining(),
        );
    }

    /// # Panics
    ///
    /// Will panic if not enough space is remaining in the buffer to store the whole slice
    #[inline]
    pub fn put_slice(&mut self, src: &[u8]) {
        self.check_remaining(src.len());

        unsafe {
            core::ptr::copy_nonoverlapping(src.as_ptr(), &mut self.data[self.len], src.len());
        }
        self.len += src.len();
    }

    /// # Panics
    ///
    /// Will panic if no space is remaing in the buffer
    #[inline]
    pub fn put_u8(&mut self, val: u8) {
        self.check_remaining(1);

        self.data[self.len] = val;
        self.len += 1;
    }

    /// # Panics
    ///
    /// Will panic if no space is remaing in the buffer
    #[inline]
    pub fn put_i8(&mut self, val: i8) {
        self.check_remaining(1);

        self.data[self.len] = val as u8;
        self.len += 1;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 2 bytes required by a `u16` remain in the buffer
    #[inline]
    pub fn put_u16_le(&mut self, val: u16) {
        const SIZE: usize = core::mem::size_of::<u16>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 2 bytes required by a `i16` remain in the buffer
    #[inline]
    pub fn put_i16_le(&mut self, val: i16) {
        const SIZE: usize = core::mem::size_of::<i16>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if `val` is not a valid 24 bit unsigned integer or if not
    /// enough space is remaing in the buffer to store 3 bytes
    #[inline]
    pub fn put_u24_le(&mut self, val: u32) {
        const SIZE: usize = 3;
        const MAX: u32 = 2u32.pow(24) - 1;

        assert!(
            val <= MAX,
            "Attempted to put value that is too large for 24 bits, \
	     attempted to push: {val}, max allowed: {MAX}",
        );

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..3]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if `val` is not a valid 24 bit signed integer or if not
    /// enough space is remaing in the buffer to store 3 bytes
    #[inline]
    pub fn put_i24_le(&mut self, val: i32) {
        const SIZE: usize = 3;
        const MIN: i32 = -(1i32 << 23);
        const MAX: i32 = 2i32.pow(23) - 1;

        assert!(
            val <= MAX,
            "Attempted to put value that is too large for 24 bits, \
	     attempted to push: {val}, max allowed: {MAX}",
        );
        assert!(
            val >= MIN,
            "Attempted to put value that is too negative for 24 bits, \
	     attempted to push: {val}, min allowed: {MIN}",
        );

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..3]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 4 bytes required by a `u32` remain in the buffer
    #[inline]
    pub fn put_u32_le(&mut self, val: u32) {
        const SIZE: usize = core::mem::size_of::<u32>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 4 bytes required by a `i32` remain in the buffer
    #[inline]
    pub fn put_i32_le(&mut self, val: i32) {
        const SIZE: usize = core::mem::size_of::<i32>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 8 bytes required by a `u64` remain in the buffer
    #[inline]
    pub fn put_u64_le(&mut self, val: u64) {
        const SIZE: usize = core::mem::size_of::<u64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 8 bytes required by a `i64` remain in the buffer
    #[inline]
    pub fn put_i64_le(&mut self, val: i64) {
        const SIZE: usize = core::mem::size_of::<i64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 4 bytes required by a `f32` remain in the buffer
    #[inline]
    pub fn put_f32_le(&mut self, val: f32) {
        const SIZE: usize = core::mem::size_of::<f32>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    /// # Panics
    ///
    /// Will panic if less space then the 8 bytes required by a `f64` remain in the buffer
    #[inline]
    pub fn put_f64_le(&mut self, val: f64) {
        const SIZE: usize = core::mem::size_of::<f64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }
}

#[cfg(test)]
mod tests {
    use super::BytesMut;

    #[test]
    fn put_i24_negative_one() {
        // -1 in 24-bit two's complement is 0xFF_FF_FF.
        let mut buffer = [0u8; 3];
        let mut bytes = BytesMut::new(&mut buffer);
        bytes.put_i24_le(-1);
        assert_eq!(bytes.len(), 3);
        assert_eq!(buffer, [0xff, 0xff, 0xff]);
    }

    #[test]
    fn put_i24_min_value() {
        // Min 24-bit signed value is 0x80_00_00 (little-endian: 00 00 80).
        let mut buffer = [0u8; 3];
        let mut bytes = BytesMut::new(&mut buffer);
        bytes.put_i24_le(-8_388_608);
        assert_eq!(bytes.len(), 3);
        assert_eq!(buffer, [0x00, 0x00, 0x80]);
    }

    #[test]
    fn put_i24_max_positive() {
        // Max 24-bit signed value is 0x7F_FF_FF (little-endian: FF FF 7F).
        let mut buffer = [0u8; 3];
        let mut bytes = BytesMut::new(&mut buffer);
        bytes.put_i24_le(8_388_607);
        assert_eq!(bytes.len(), 3);
        assert_eq!(buffer, [0xff, 0xff, 0x7f]);
    }
}
