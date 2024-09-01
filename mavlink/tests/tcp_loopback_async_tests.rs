mod test_shared;

#[cfg(all(feature = "tokio-1", feature = "tcp", feature = "common"))]
mod test_tcp_connections {
    #[cfg(feature = "signing")]
    use crate::test_shared;
    #[cfg(feature = "signing")]
    use mavlink::SigningConfig;

    /// Test whether we can send a message via TCP and receive it OK using async_connect.
    /// This also test signing as a property of a MavConnection if the signing feature is enabled.
    #[tokio::test]
    pub async fn test_tcp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 5;

        #[cfg(feature = "signing")]
        let singing_cfg_server = SigningConfig::new(test_shared::SECRET_KEY, 0, true, false);
        #[cfg(feature = "signing")]
        let singing_cfg_client = singing_cfg_server.clone();

        let server_thread = tokio::spawn(async move {
            //TODO consider using get_available_port to use a random port
            let mut server = mavlink::connect_async("tcpin:0.0.0.0:14551")
                .await
                .expect("Couldn't create server");

            #[cfg(feature = "signing")]
            server.setup_signing(Some(singing_cfg_server));

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
        });

        // Give some time for the server to connect
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // have the client send a few hearbeats
        tokio::spawn(async move {
            let msg =
                mavlink::common::MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg());
            let mut client = mavlink::connect_async("tcpout:127.0.0.1:14551")
                .await
                .expect("Couldn't create client");

            #[cfg(feature = "signing")]
            client.setup_signing(Some(singing_cfg_client));

            for _i in 0..RECEIVE_CHECK_COUNT {
                client.send_default(&msg).await.ok();
            }
        });

        server_thread.await.unwrap();
    }
}
