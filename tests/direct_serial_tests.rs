extern crate mavlink;

#[cfg(test)]
#[cfg(all(feature = "std", feature = "direct-serial"))]
mod test_direct_serial {
    #[test]
    pub fn test_incomplete_address() {
        let conn_result = mavlink::connect("serial:");
        assert!(conn_result.is_err(), "Incomplete address should error");
    }

    #[test]
    pub fn test_bogus_baud() {
        let conn_result = mavlink::connect("serial:port1:badbaud");
        assert!(conn_result.is_err(), "Invalid baud should error");
    }

    #[test]
    pub fn test_nonexistent_port() {
        let bogus_port_str = "serial:8d73ba8c-eb87-4105-8d0c-2931940e13be:57600";
        let conn_result = mavlink::connect(bogus_port_str);
        assert!(conn_result.is_err(), "Invalid port should error");
    }
}
