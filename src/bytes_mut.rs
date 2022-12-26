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
            "write buffer overflow; remaining {} bytes, try add {} bytes",
            self.remaining(),
            count
        );
    }

    pub fn put_slice(&mut self, src: &[u8]) {
        self.check_remaining(src.len());

        unsafe {
            core::ptr::copy_nonoverlapping(src.as_ptr(), &mut self.data[self.len], src.len());
        }
        self.len += src.len();
    }

    pub fn put_u8(&mut self, val: u8) {
        self.check_remaining(1);

        self.data[self.len] = val;
        self.len += 1;
    }

    pub fn put_i8(&mut self, val: i8) {
        self.check_remaining(1);

        self.data[self.len] = val as u8;
        self.len += 1;
    }

    pub fn put_u16_le(&mut self, val: u16) {
        const SIZE: usize = core::mem::size_of::<u16>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_i16_le(&mut self, val: i16) {
        const SIZE: usize = core::mem::size_of::<i16>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_u32_le(&mut self, val: u32) {
        const SIZE: usize = core::mem::size_of::<u32>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_i32_le(&mut self, val: i32) {
        const SIZE: usize = core::mem::size_of::<i32>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_u64_le(&mut self, val: u64) {
        const SIZE: usize = core::mem::size_of::<u64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_i64_le(&mut self, val: i64) {
        const SIZE: usize = core::mem::size_of::<i64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_f32_le(&mut self, val: f32) {
        const SIZE: usize = core::mem::size_of::<f32>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }

    pub fn put_f64_le(&mut self, val: f64) {
        const SIZE: usize = core::mem::size_of::<f64>();
        self.check_remaining(SIZE);

        let src = val.to_le_bytes();
        self.data[self.len..self.len + SIZE].copy_from_slice(&src[..]);
        self.len += SIZE;
    }
}
