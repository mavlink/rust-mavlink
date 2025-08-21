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
    /// Will panic if not enough space is remaing in the buffer to store a `u16`
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
    /// Will panic if not enough space is remaing in the buffer to store an `i16`
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
        const MIN: i32 = 2i32.pow(23);
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
    /// Will panic if not enough space is remaing in the buffer to store a `u32`
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
    /// Will panic if not enough space is remaing in the buffer to store an `i32`
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
    /// Will panic if not enough space is remaing in the buffer to store a `u64`
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
    /// Will panic if not enough space is remaing in the buffer to store an `i64`
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
    /// Will panic if not enough space is remaing in the buffer to store a `f32`
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
    /// Will panic if not enough space is remaing in the buffer to store a `f64`
    #[inline]
    pub fn put_f64_le(&mut self, val: f64) {
        const SIZE: usize = core::mem::size_of::<f64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }
}
