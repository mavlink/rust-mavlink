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

    #[inline]
    fn bounds_error(&self, requested: usize) -> Error {
        Error::NotEnoughBuffer {
            requested,
            available: self.remaining(),
        }
    }

    /// # Errors
    ///
    /// Will return an error if not at least `count` bytes remain in the buffer
    #[inline]
    pub fn get_bytes(&mut self, count: usize) -> Result<&[u8], Error> {
        let bytes = &self
            .data
            .get(self.pos..(self.pos + count))
            .ok_or_else(|| self.bounds_error(count))?;
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
            .ok_or_else(|| self.bounds_error(1))?;
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
            .ok_or_else(|| self.bounds_error(1))? as i8;
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
                .ok_or_else(|| self.bounds_error(SIZE))?,
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
                .ok_or_else(|| self.bounds_error(SIZE))?,
        );
        self.pos += SIZE;

        debug_assert_eq!(val[3], 0);
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
