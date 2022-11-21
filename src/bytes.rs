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
        const SIZE: usize = core::mem::size_of::<u16>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        u16::from_le_bytes(val)
    }

    pub fn get_i16_le(&mut self) -> i16 {
        const SIZE: usize = core::mem::size_of::<i16>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        i16::from_le_bytes(val)
    }

    pub fn get_u32_le(&mut self) -> u32 {
        const SIZE: usize = core::mem::size_of::<u32>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        u32::from_le_bytes(val)
    }

    pub fn get_i32_le(&mut self) -> i32 {
        const SIZE: usize = core::mem::size_of::<i32>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        i32::from_le_bytes(val)
    }

    pub fn get_u64_le(&mut self) -> u64 {
        const SIZE: usize = core::mem::size_of::<u64>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        u64::from_le_bytes(val)
    }

    pub fn get_i64_le(&mut self) -> i64 {
        const SIZE: usize = core::mem::size_of::<i64>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        i64::from_le_bytes(val)
    }

    pub fn get_f32_le(&mut self) -> f32 {
        const SIZE: usize = core::mem::size_of::<f32>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        f32::from_le_bytes(val)
    }

    pub fn get_f64_le(&mut self) -> f64 {
        const SIZE: usize = core::mem::size_of::<f64>();
        self.check_remaining(SIZE);

        let mut val = [0u8; SIZE];
        val.copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;
        f64::from_le_bytes(val)
    }
}
