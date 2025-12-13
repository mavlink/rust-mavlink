mod test_shared;

#[cfg(all(feature = "std", feature = "udp", feature = "common"))]
mod test_udp_connections {
    use std::thread;

    use mavlink::{MavConnection, MessageData};

    /// Test whether we can send a message via UDP and receive it OK
    #[test]
    fn test_udp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 3;

        let server = mavlink::connect("udpin:0.0.0.0:14551").expect("Couldn't create server");

        // have the client send one heartbeat per second
        thread::spawn({
            move || {
                let msg =
                    mavlink::common::MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg());
                let client =
                    mavlink::connect("udpout:127.0.0.1:14551").expect("Couldn't create client");
                loop {
                    client.send_default(&msg).ok();
                }
            }
        });

        //TODO use std::sync::WaitTimeoutResult to timeout ourselves if recv fails?
        let mut recv_count = 0;
        for _i in 0..RECEIVE_CHECK_COUNT {
            match server.recv() {
                Ok((_header, msg)) => {
                    if let mavlink::common::MavMessage::HEARTBEAT(_heartbeat_msg) = msg {
                        recv_count += 1;
                    } else {
                        // one message parse failure fails the test
                        break;
                    }
                }
                Err(..) => {
                    // one message read failure fails the test
                    break;
                }
            }
        }
        assert_eq!(recv_count, RECEIVE_CHECK_COUNT);
    }

    /// Test whether we can send a message via UDP and receive it OK using recv_raw
    #[test]
    fn test_udp_loopback_recv_raw() {
        const RECEIVE_CHECK_COUNT: i32 = 3;

        let server = mavlink::connect::<mavlink::common::MavMessage>("udpin:0.0.0.0:14561")
            .expect("Couldn't create server");

        // have the client send one heartbeat per second
        thread::spawn({
            move || {
                let msg =
                    mavlink::common::MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg());
                let client =
                    mavlink::connect("udpout:127.0.0.1:14561").expect("Couldn't create client");
                loop {
                    client.send_default(&msg).ok();
                }
            }
        });

        //TODO use std::sync::WaitTimeoutResult to timeout ourselves if recv fails?
        let mut recv_count = 0;
        for _i in 0..RECEIVE_CHECK_COUNT {
            match server.recv_raw() {
                Ok(message) => {
                    if message.message_id() == mavlink::common::HEARTBEAT_DATA::ID {
                        recv_count += 1;
                    } else {
                        // one message parse failure fails the test
                        break;
                    }
                }
                Err(..) => {
                    // one message read failure fails the test
                    break;
                }
            }
        }
        assert_eq!(recv_count, RECEIVE_CHECK_COUNT);
    }
}
