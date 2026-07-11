//! MAVLink frame constants.
//!
//! These constants can be used when allocating buffers or inspecting
//! MAVLink 1 and MAVLink 2 frame layouts.
//!
//! ```
//! use mavlink_core::consts;
//!
//! let v1_buffer = [0_u8; consts::v1::FRAME_SIZE];
//! let v2_buffer = [0_u8; consts::v2::FRAME_SIZE];
//!
//! assert_eq!(v1_buffer.len(), consts::v1::FRAME_SIZE);
//! assert_eq!(v2_buffer.len(), consts::MAX_FRAME_SIZE);
//! ```

/// Size of the STX marker.
pub const STX_SIZE: usize = 1;
/// Offset of the STX marker.
pub const STX_OFFSET: usize = 0;
/// Maximum payload length.
pub const MAX_PAYLOAD_LEN: usize = 255;
/// Offset of the payload length field.
pub const PAYLOAD_LEN_OFFSET: usize = 1;
/// Size of the checksum field.
pub const CHECKSUM_SIZE: usize = 2;

/// Maximum MAVLink frame size.
pub const MAX_FRAME_SIZE: usize = v2::FRAME_SIZE;

/// MAVLink 1 frame constants.
pub mod v1 {
    /// Header size, excluding the STX marker.
    pub const HEADER_SIZE: usize = 5;

    /// Maximum frame size.
    pub const FRAME_SIZE: usize =
        super::STX_SIZE + HEADER_SIZE + super::MAX_PAYLOAD_LEN + super::CHECKSUM_SIZE;
}

/// MAVLink 2 frame constants.
pub mod v2 {
    /// Header size, excluding the STX marker.
    pub const HEADER_SIZE: usize = 9;

    /// Offset of the incompatibility flags field.
    pub const INCOMPAT_FLAGS_OFFSET: usize = 2;

    /// Signed-frame incompatibility flag.
    pub const IFLAG_SIGNED: u8 = 0x01;

    /// Incompatibility flags handled by this crate.
    pub const SUPPORTED_IFLAGS: u8 = IFLAG_SIGNED;

    /// Signature link id size.
    pub const SIGNATURE_LINK_ID_SIZE: usize = 1;

    /// Signature timestamp size.
    pub const SIGNATURE_TIMESTAMP_SIZE: usize = 6;

    /// Signature value size.
    pub const SIGNATURE_VALUE_SIZE: usize = 6;

    /// Total signature trailer size.
    pub const SIGNATURE_SIZE: usize =
        SIGNATURE_LINK_ID_SIZE + SIGNATURE_TIMESTAMP_SIZE + SIGNATURE_VALUE_SIZE;

    /// Maximum frame size.
    pub const FRAME_SIZE: usize = super::STX_SIZE
        + HEADER_SIZE
        + super::MAX_PAYLOAD_LEN
        + super::CHECKSUM_SIZE
        + SIGNATURE_SIZE;
}
