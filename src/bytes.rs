pub struct Bytes<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Bytes<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    #[inline]
    fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    pub fn remaining_bytes(&self) -> &'a [u8] {
        &self.data[self.pos..]
    }

    fn check_remaining(&self, count: usize) {
        assert!(
            self.remaining() >= count,
            "read buffer exhausted; remaining {} bytes, try read {} bytes",
            self.remaining(),
            count
        );
    }

    pub fn get_bytes(&mut self, count: usize) -> &[u8] {
        self.check_remaining(count);

        let bytes = &self.data[self.pos..(self.pos + count)];
        self.pos += count;
        bytes
    }

    pub fn get_array<const SIZE: usize>(&mut self) -> [u8; SIZE] {
        let bytes = self.get_bytes(SIZE);
        let mut arr = [0u8; SIZE];

        arr.copy_from_slice(bytes);

        debug_assert_eq!(arr.as_slice(), bytes);

        arr
    }

    pub fn get_u8(&mut self) -> u8 {
        self.check_remaining(1);

        let val = self.data[self.pos];
        self.pos += 1;
        val
    }

    pub fn get_i8(&mut self) -> i8 {
        self.check_remaining(1);

        let val = self.data[self.pos] as i8;
        self.pos += 1;
        val
    }

    pub fn get_u16_le(&mut self) -> u16 {
        u16::from_le_bytes(self.get_array())
    }

    pub fn get_i16_le(&mut self) -> i16 {
        i16::from_le_bytes(self.get_array())
    }

    pub fn get_u24_le(&mut self) -> u32 {
        const SIZE: usize = 3;
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE + 1];
        val[..3].copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;

        debug_assert_eq!(val[3], 0);
        u32::from_le_bytes(val)
    }

    pub fn get_i24_le(&mut self) -> i32 {
        const SIZE: usize = 3;
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE + 1];
        val[..3].copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;

        debug_assert_eq!(val[3], 0);
        i32::from_le_bytes(val)
    }

    pub fn get_u32_le(&mut self) -> u32 {
        u32::from_le_bytes(self.get_array())
    }

    pub fn get_i32_le(&mut self) -> i32 {
        i32::from_le_bytes(self.get_array())
    }

    pub fn get_u64_le(&mut self) -> u64 {
        u64::from_le_bytes(self.get_array())
    }

    pub fn get_i64_le(&mut self) -> i64 {
        i64::from_le_bytes(self.get_array())
    }

    pub fn get_f32_le(&mut self) -> f32 {
        f32::from_le_bytes(self.get_array())
    }

    pub fn get_f64_le(&mut self) -> f64 {
        f64::from_le_bytes(self.get_array())
    }
}
