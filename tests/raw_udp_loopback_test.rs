mod test_shared;

#[cfg(all(feature = "std", feature = "udp", feature = "common"))]
mod test_udp_connections {
    use crate::test_shared::get_heartbeat_msg;
    use mavlink::common::MavMessage;
    use mavlink::connection::routing::RawConnection;
    use mavlink::connection::udp::{udpin, udpout};
    use mavlink::CommonMessageRaw;
    use mavlink::{read_versioned_msg, MAVLinkV2MessageRaw, MavConnection, MavHeader};
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
                let raw_client = &client as &dyn RawConnection<MavMessage>;
                loop {
                    raw_client.raw_write(&mut raw_msg).ok();
                }
            }
        });

        let mut recv_count = 0;
        let raw_server = &server as &dyn RawConnection<MavMessage>;
        for _ in 0..RECEIVE_CHECK_COUNT {
            match raw_server.raw_read() {
                Ok(raw_msg) => {
                    let mut cursor = Cursor::<&[u8]>::new(&raw_msg.full());
                    let (_hdr, msg) = read_versioned_msg::<MavMessage, Cursor<&[u8]>>(
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
