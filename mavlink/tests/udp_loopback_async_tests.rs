mod test_shared;

#[cfg(all(feature = "tokio-1", feature = "udp", feature = "common"))]
mod test_udp_connections {

    /// Test whether we can send a message via UDP and receive it OK using async_connect
    #[tokio::test]
    pub async fn test_udp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 3;

        let server = mavlink::connect_async("udpin:0.0.0.0:14552").await.expect("Couldn't create server");

        // have the client send one heartbeat per second
        tokio::spawn({
            async move {
                let msg =
                    mavlink::common::MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg());
                let client =
                    mavlink::connect_async("udpout:127.0.0.1:14552").await.expect("Couldn't create client");
                loop {
                    client.send_default(&msg).await.ok();
                }
            }
        });

        //TODO use std::sync::WaitTimeoutResult to timeout ourselves if recv fails?
        let mut recv_count = 0;
        for _i in 0..RECEIVE_CHECK_COUNT {
            match server.recv().await {
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
}
