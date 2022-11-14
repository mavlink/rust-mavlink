pub struct BytesMut<const N: usize> {
    data: [u8; N],
    len: usize,
}

impl<const N: usize> BytesMut<N> {
    pub fn new() -> Self {
        Self {
            data: [0; N],
            len: 0
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn remaining(&self) -> usize {
        N - self.len
    }

    pub fn set_len(&mut self, len: usize) {
        assert!(len >= 1);
        assert!(len <= N);
        self.len = len;
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
        self.check_remaining(2);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.len += 2;
    }

    pub fn put_i16_le(&mut self, val: i16) {
        self.check_remaining(2);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.len += 2;
    }

    pub fn put_u32_le(&mut self, val: u32) {
        self.check_remaining(4);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.data[self.len + 2] = src[2];
        self.data[self.len + 3] = src[3];
        self.len += 4;
    }

    pub fn put_i32_le(&mut self, val: i32) {
        self.check_remaining(4);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.data[self.len + 2] = src[2];
        self.data[self.len + 3] = src[3];
        self.len += 4;
    }

    pub fn put_u64_le(&mut self, val: u64) {
        self.check_remaining(8);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.data[self.len + 2] = src[2];
        self.data[self.len + 3] = src[3];
        self.data[self.len + 4] = src[4];
        self.data[self.len + 5] = src[5];
        self.data[self.len + 6] = src[6];
        self.data[self.len + 7] = src[7];
        self.len += 8;
    }

    pub fn put_i64_le(&mut self, val: i64) {
        self.check_remaining(8);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.data[self.len + 2] = src[2];
        self.data[self.len + 3] = src[3];
        self.data[self.len + 4] = src[4];
        self.data[self.len + 5] = src[5];
        self.data[self.len + 6] = src[6];
        self.data[self.len + 7] = src[7];
        self.len += 8;
    }

    pub fn put_f32_le(&mut self, val: f32) {
        self.check_remaining(4);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.data[self.len + 2] = src[2];
        self.data[self.len + 3] = src[3];
        self.len += 4;
    }

    pub fn put_f64_le(&mut self, val: f64) {
        self.check_remaining(8);

        let src = val.to_le_bytes();
        self.data[self.len + 0] = src[0];
        self.data[self.len + 1] = src[1];
        self.data[self.len + 2] = src[2];
        self.data[self.len + 3] = src[3];
        self.data[self.len + 4] = src[4];
        self.data[self.len + 5] = src[5];
        self.data[self.len + 6] = src[6];
        self.data[self.len + 7] = src[7];
        self.len += 8;
    }
}

impl<const N: usize> core::ops::Deref for BytesMut<N> {
    type Target = [u8];

    #[inline]
    fn deref(&self) -> &[u8] {
        &self.data[..self.len]
    }
}
