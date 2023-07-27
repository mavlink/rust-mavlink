mod test_shared;

#[cfg(all(feature = "std", feature = "udp", feature = "common"))]
mod test_udp_connections {
    use crate::test_shared::get_heartbeat_msg;
    use mavlink::common::MavMessage;
    use mavlink::connection::routing::RawMavV2Connection;
    use mavlink::connection::udp::{udpin, udpout};
    use mavlink::{read_versioned_msg, MAVLinkV2MessageRaw, MavConnection, MavHeader, MAX_SIZE_V2};
    use std::io::Cursor;
    use std::thread;

    /// Test whether we can send a raw message via UDP and receive it OK with the correct minimal
    /// updates to it.
    #[test]
    pub fn test_raw_v2_udp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 3;
        let server = udpin("0.0.0.0:14551").expect("Couldn't create server");

        // have the client send one heartbeat per second
        thread::spawn({
            move || {
                let msg = MavMessage::HEARTBEAT(get_heartbeat_msg());
                let mut raw_msg = MAVLinkV2MessageRaw::new();
                let header = MavHeader {
                    system_id: 3,
                    component_id: 2,
                    sequence: 42, // As it will be rerouted, this sequence should be rewritted.
                };
                raw_msg.serialize_message(header, &msg);
                let client = udpout("127.0.0.1:14551").expect("Couldn't create client");
                let raw_client = &client as &dyn RawMavV2Connection<MavMessage>;
                loop {
                    raw_client.raw_write(&mut raw_msg).ok();
                }
            }
        });

        let mut recv_count = 0;
        let mut current_sequence_number = 0u8; // it should start back at 0.
        let raw_server = &server as &dyn RawMavV2Connection<MavMessage>;
        for _ in 0..RECEIVE_CHECK_COUNT {
            match raw_server.raw_read() {
                Ok(raw_msg) => {
                    // Check if we have a correctly patched sequence number.
                    assert_eq!(raw_msg.sequence(), current_sequence_number);
                    current_sequence_number += 1;
                    // The CRC should have been patched too.
                    assert!(raw_msg.has_valid_crc::<MavMessage>());
                    let mut cursor = Cursor::<&[u8; MAX_SIZE_V2]>::new(&raw_msg.0);
                    let (_hdr, msg) = read_versioned_msg::<MavMessage, Cursor<&[u8; MAX_SIZE_V2]>>(
                        &mut cursor,
                        (&server as &dyn MavConnection<MavMessage>).get_protocol_version(),
                    )
                    .unwrap();

                    if let MavMessage::HEARTBEAT(_heartbeat_msg) = msg {
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
