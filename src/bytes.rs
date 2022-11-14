pub struct Bytes<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Bytes<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0
        }
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
        self.check_remaining(2);

        let mut val = [0u8; 2];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        self.pos += 2;
        u16::from_le_bytes(val)
    }

    pub fn get_i16_le(&mut self) -> i16 {
        self.check_remaining(2);

        let mut val = [0u8; 2];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        self.pos += 2;
        i16::from_le_bytes(val)
    }

    pub fn get_u32_le(&mut self) -> u32 {
        self.check_remaining(4);

        let mut val = [0u8; 4];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        val[2] = self.data[self.pos + 2];
        val[3] = self.data[self.pos + 3];
        self.pos += 4;
        u32::from_le_bytes(val)
    }

    pub fn get_i32_le(&mut self) -> i32 {
        self.check_remaining(4);

        let mut val = [0u8; 4];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        val[2] = self.data[self.pos + 2];
        val[3] = self.data[self.pos + 3];
        self.pos += 4;
        i32::from_le_bytes(val)
    }

    pub fn get_u64_le(&mut self) -> u64 {
        self.check_remaining(8);

        let mut val = [0u8; 8];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        val[2] = self.data[self.pos + 2];
        val[3] = self.data[self.pos + 3];
        val[4] = self.data[self.pos + 4];
        val[5] = self.data[self.pos + 5];
        val[6] = self.data[self.pos + 6];
        val[7] = self.data[self.pos + 7];
        self.pos += 8;
        u64::from_le_bytes(val)
    }

    pub fn get_i64_le(&mut self) -> i64 {
        self.check_remaining(8);

        let mut val = [0u8; 8];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        val[2] = self.data[self.pos + 2];
        val[3] = self.data[self.pos + 3];
        val[4] = self.data[self.pos + 4];
        val[5] = self.data[self.pos + 5];
        val[6] = self.data[self.pos + 6];
        val[7] = self.data[self.pos + 7];
        self.pos += 8;
        i64::from_le_bytes(val)
    }

    pub fn get_f32_le(&mut self) -> f32 {
        self.check_remaining(4);

        let mut val = [0u8; 4];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        val[2] = self.data[self.pos + 2];
        val[3] = self.data[self.pos + 3];
        self.pos += 4;
        f32::from_le_bytes(val)
    }

    pub fn get_f64_le(&mut self) -> f64 {
        self.check_remaining(8);

        let mut val = [0u8; 8];
        val[0] = self.data[self.pos + 0];
        val[1] = self.data[self.pos + 1];
        val[2] = self.data[self.pos + 2];
        val[3] = self.data[self.pos + 3];
        val[4] = self.data[self.pos + 4];
        val[5] = self.data[self.pos + 5];
        val[6] = self.data[self.pos + 6];
        val[7] = self.data[self.pos + 7];
        self.pos += 8;
        f64::from_le_bytes(val)
    }
}
