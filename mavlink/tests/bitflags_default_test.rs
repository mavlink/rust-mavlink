#[cfg(feature = "dialect-common")]
mod bitflags_default_tests {
    use mavlink::dialects::common::{
        GIMBAL_DEVICE_INFORMATION_DATA, GimbalDeviceCapFlags, HlFailureFlag, MavProtocolCapability,
    };

    #[test]
    fn bitflags_default_to_no_flags_set() {
        assert!(GimbalDeviceCapFlags::DEFAULT.is_empty());
        assert!(GimbalDeviceCapFlags::default().is_empty());
        assert!(HlFailureFlag::default().is_empty());
        assert!(MavProtocolCapability::default().is_empty());
    }

    #[test]
    fn default_message_has_no_bitflags_set() {
        let msg = GIMBAL_DEVICE_INFORMATION_DATA::default();
        assert!(msg.cap_flags.is_empty());
        assert_eq!(msg.cap_flags.bits(), 0);
    }
}
