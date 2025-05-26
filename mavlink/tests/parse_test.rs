#[cfg(any(feature = "std", feature = "tokio-1"))]
mod parse_tests {
    use mavlink::ConnectionAddress;

    fn assert_parse(addr: &str) {
        assert_eq!(
            format!("{}", ConnectionAddress::parse_address(addr).unwrap()),
            addr
        );
    }

    #[cfg(feature = "tcp")]
    #[test]
    fn test_parse_tcp() {
        assert_parse("tcpin:example.com:99");
        assert_parse("tcpout:127.0.0.1:14549");
    }

    #[cfg(feature = "tcp")]
    #[test]
    fn test_parse_file() {
        assert_parse("file:/mnt/12_44-mav.bin");
        assert_parse("file:C:\\mav_logs\\test.bin");
    }

    #[cfg(feature = "udp")]
    #[test]
    fn test_parse_udp() {
        assert_parse("udpcast:[::1]:4567");
        assert_parse("udpin:[2001:db8:85a3:8d3:1319:8a2e:370:7348]:443");
        assert_parse("udpout:1.1.1.1:1");
    }

    #[cfg(feature = "direct-serial")]
    #[test]
    fn test_parse_serial() {
        assert_parse("serial:/dev/ttyUSB0:9600");
        assert_parse("serial:COM0:115200");
    }

    #[test]
    fn test_parse_errors() {
        assert!(ConnectionAddress::parse_address("serial:/dev/ttyUSB0").is_err());
        assert!(ConnectionAddress::parse_address("updout:1.1.1.1:1").is_err());
        assert!(ConnectionAddress::parse_address("tcp:127.0.0.1:14540").is_err());
        assert!(ConnectionAddress::parse_address("tcpin127.0.0.1:14540").is_err());
        assert!(ConnectionAddress::parse_address(" udpout:1.1.1.1:1 ").is_err());
        assert!(ConnectionAddress::parse_address(":udpcast:[::1]:4567").is_err());
    }
}
