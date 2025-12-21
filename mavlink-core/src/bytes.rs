pub struct Bytes<'a> {
    data: &'a [u8],
    pos: usize,
}

/// Simple error types for the bytes module.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Attempted to to read more bytes than available.
    NotEnoughBuffer { requested: usize, available: usize },
}

impl Error {
    #[inline]
    fn not_enough_buffer(requested: usize, bytes: &Bytes) -> Self {
        Self::NotEnoughBuffer {
            requested,
            available: bytes.remaining(),
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotEnoughBuffer {
                requested,
                available,
            } => write!(
                f,
                "Attempted to read {requested} bytes but only {available} available.",
            ),
        }
    }
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

    /// # Errors
    ///
    /// Will return an error if not at least `count` bytes remain in the buffer
    #[inline]
    pub fn get_bytes(&mut self, count: usize) -> Result<&[u8], Error> {
        let bytes = &self
            .data
            .get(self.pos..(self.pos + count))
            .ok_or_else(|| Error::not_enough_buffer(count, self))?;
        self.pos += count;
        Ok(bytes)
    }

    /// # Errors
    ///
    /// Will return an error if not at least `SIZE` bytes remain in the buffer
    #[inline]
    pub fn get_array<const SIZE: usize>(&mut self) -> Result<[u8; SIZE], Error> {
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
    pub fn get_u8(&mut self) -> Result<u8, Error> {
        let val = *self
            .data
            .get(self.pos)
            .ok_or_else(|| Error::not_enough_buffer(1, self))?;
        self.pos += 1;
        Ok(val)
    }

    /// # Errors
    ///
    /// Will return an error if nothing is remaining in the buffer
    #[inline]
    pub fn get_i8(&mut self) -> Result<i8, Error> {
        let val = *self
            .data
            .get(self.pos)
            .ok_or_else(|| Error::not_enough_buffer(1, self))? as i8;
        self.pos += 1;
        Ok(val)
    }

    /// # Errors
    ///
    /// Will return an error if less then the 2 required bytes for a `u16` remain
    #[inline]
    pub fn get_u16_le(&mut self) -> Result<u16, Error> {
        Ok(u16::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 2 required bytes for a `i16` remain
    #[inline]
    pub fn get_i16_le(&mut self) -> Result<i16, Error> {
        Ok(i16::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if not at least 3 bytes remain
    #[inline]
    pub fn get_u24_le(&mut self) -> Result<u32, Error> {
        const SIZE: usize = 3;

        let mut val = [0u8; SIZE + 1];
        val[..3].copy_from_slice(
            self.data
                .get(self.pos..self.pos + SIZE)
                .ok_or_else(|| Error::not_enough_buffer(SIZE, self))?,
        );
        self.pos += SIZE;

        debug_assert_eq!(val[3], 0);
        Ok(u32::from_le_bytes(val))
    }

    /// # Errors
    ///
    /// Will return an error if not at least 3 bytes remain
    #[inline]
    pub fn get_i24_le(&mut self) -> Result<i32, Error> {
        const SIZE: usize = 3;

        let mut val = [0u8; SIZE + 1];
        val[..3].copy_from_slice(
            self.data
                .get(self.pos..self.pos + SIZE)
                .ok_or_else(|| Error::not_enough_buffer(SIZE, self))?,
        );
        self.pos += SIZE;

        if val[2] & 0x80 != 0 {
            val[3] = 0xff;
        }
        Ok(i32::from_le_bytes(val))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 4 required bytes for a `u32` remain
    #[inline]
    pub fn get_u32_le(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 4 required bytes for a `i32` remain
    #[inline]
    pub fn get_i32_le(&mut self) -> Result<i32, Error> {
        Ok(i32::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 8 required bytes for a `u64` remain
    #[inline]
    pub fn get_u64_le(&mut self) -> Result<u64, Error> {
        Ok(u64::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 8 required bytes for a `i64` remain
    #[inline]
    pub fn get_i64_le(&mut self) -> Result<i64, Error> {
        Ok(i64::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 4 required bytes for a `f32` remain
    #[inline]
    pub fn get_f32_le(&mut self) -> Result<f32, Error> {
        Ok(f32::from_le_bytes(self.get_array()?))
    }

    /// # Errors
    ///
    /// Will return an error if less then the 8 required bytes for a `f64` remain
    #[inline]
    pub fn get_f64_le(&mut self) -> Result<f64, Error> {
        Ok(f64::from_le_bytes(self.get_array()?))
    }
}

#[cfg(test)]
mod tests {
    use super::Bytes;
    use crate::bytes_mut::BytesMut;

    #[test]
    fn get_i24_negative_one() {
        // 0xFF_FF_FF (24-bit two's complement) == -1
        let data = [0xff, 0xff, 0xff];
        let mut bytes = Bytes::new(&data);
        assert_eq!(bytes.get_i24_le().unwrap(), -1);
    }

    #[test]
    fn get_i24_min_value() {
        // 0x80_00_00 is the minimum 24-bit signed value.
        let data = [0x00, 0x00, 0x80];
        let mut bytes = Bytes::new(&data);
        assert_eq!(bytes.get_i24_le().unwrap(), -8_388_608);
    }

    #[test]
    fn get_i24_max_positive() {
        // 0x7F_FF_FF is the maximum 24-bit signed value.
        let data = [0xff, 0xff, 0x7f];
        let mut bytes = Bytes::new(&data);
        assert_eq!(bytes.get_i24_le().unwrap(), 8_388_607);
    }

    #[test]
    fn i24_round_trip_values() {
        let values = [-8_388_608, -1, 0, 1, 8_388_607];
        for val in values {
            let mut buffer = [0u8; 3];
            let mut writer = BytesMut::new(&mut buffer);
            writer.put_i24_le(val);
            assert_eq!(writer.len(), 3);

            let mut reader = Bytes::new(&buffer);
            assert_eq!(reader.get_i24_le().unwrap(), val);
        }
    }
}
