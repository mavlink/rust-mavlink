use std::io;

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

    #[inline]
    pub fn remaining_bytes(&self) -> &'a [u8] {
        &self.data[self.pos..]
    }

    #[inline]
    fn check_remaining(&self, count: usize) -> io::Result<()> {
        if self.remaining() >= count {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "buffer underflow: tried to read {} bytes, but only {} bytes remaining",
                    count,
                    self.remaining()
                ),
            ))
        }
    }

    /// # Errors
    ///
    /// Will return an error if not at least `count` bytes remain in the buffer
    #[inline]
    pub fn get_bytes(&mut self, count: usize) -> Result<&[u8], io::Error> {
        self.check_remaining(count)?;

        let bytes = &self.data[self.pos..(self.pos + count)];
        self.pos += count;
        Ok(bytes)
    }

    /// # Errors
    ///
    /// Will return an error if not at least `SIZE` bytes remain in the buffer
    #[inline]
    pub fn get_array<const SIZE: usize>(&mut self) -> Result<[u8; SIZE], io::Error> {
        let bytes = self.get_bytes(SIZE)?;
        let mut arr = [0u8; SIZE];

        arr.copy_from_slice(bytes);

        debug_assert_eq!(arr.as_slice(), bytes);

        Ok(arr)
    }

    /// # Errors
    ///
    /// Will return an error if nothing is remaining in the buffer
    #[inline]
    pub fn get_u8(&mut self) -> Result<u8, io::Error> {
        self.check_remaining(1)?;

        let val = self.data[self.pos];
        self.pos += 1;
        Ok(val)
    }

    /// # Errors
    ///
    /// Will return an error if nothing is remaining in the buffer
    #[inline]
    pub fn get_i8(&mut self) -> Result<i8, io::Error> {
        self.check_remaining(1)?;

        let val = self.data[self.pos] as i8;
        self.pos += 1;
        Ok(val)
    }

    /// # Errors
    ///
    /// Will return an error if less then the 2 required bytes for a `u16` remain
    #[inline]
    pub fn get_u16_le(&mut self) -> Result<u16, io::Error> {
        Ok(u16::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 2 required bytes for a `i16` remain
    #[inline]
    pub fn get_i16_le(&mut self) -> Result<i16, io::Error> {
        Ok(i16::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if not at least 3 bytes remain
    #[inline]
    pub fn get_u24_le(&mut self) -> Result<u32, io::Error> {
        const SIZE: usize = 3;
        self.check_remaining(SIZE)?;

        let mut val = [0u8; SIZE + 1];
        val[..3].copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;

        debug_assert_eq!(val[3], 0);
        Ok(u32::from_le_bytes(val))
    }

    /// # Errors
    ///
    /// Will return an error if not at least 3 bytes remain
    #[inline]
    pub fn get_i24_le(&mut self) -> Result<i32, io::Error> {
        const SIZE: usize = 3;
        self.check_remaining(SIZE)?;

        let mut val = [0u8; SIZE + 1];
        val[..3].copy_from_slice(&self.data[self.pos..self.pos + SIZE]);
        self.pos += SIZE;

        debug_assert_eq!(val[3], 0);
        Ok(i32::from_le_bytes(val))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 4 required bytes for a `u32` remain
    #[inline]
    pub fn get_u32_le(&mut self) -> Result<u32, io::Error> {
        Ok(u32::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 4 required bytes for a `i32` remain
    #[inline]
    pub fn get_i32_le(&mut self) -> Result<i32, io::Error> {
        Ok(i32::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 8 required bytes for a `u64` remain
    #[inline]
    pub fn get_u64_le(&mut self) -> Result<u64, io::Error> {
        Ok(u64::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 8 required bytes for a `i64` remain
    #[inline]
    pub fn get_i64_le(&mut self) -> Result<i64, io::Error> {
        Ok(i64::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 4 required bytes for a `f32` remain
    #[inline]
    pub fn get_f32_le(&mut self) -> Result<f32, io::Error> {
        Ok(f32::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 8 required bytes for a `f64` remain
    #[inline]
    pub fn get_f64_le(&mut self) -> Result<f64, io::Error> {
        Ok(f64::from_le_bytes(self.get_array()?))
    }
}
