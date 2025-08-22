mod test_shared;

#[cfg(all(feature = "std", feature = "tcp", feature = "common"))]
mod test_tcp_connections {
    use std::thread;

    #[cfg(feature = "signing")]
    use crate::test_shared;
    use mavlink::MessageData;
    #[cfg(feature = "signing")]
    use mavlink::SigningConfig;

    /// Test whether we can send a message via TCP and receive it OK. This also test signing as a property of a MavConnection if the signing feature is enabled.
    #[test]
    pub fn test_tcp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 5;

        #[cfg(feature = "signing")]
        let singing_cfg_server = SigningConfig::new(test_shared::SECRET_KEY, 0, true, false);
        #[cfg(feature = "signing")]
        let singing_cfg_client = singing_cfg_server.clone();

        let server_thread = thread::spawn(move || {
            //TODO consider using get_available_port to use a random port
            #[allow(unused_mut)]
            let mut server =
                mavlink::connect("tcpin:0.0.0.0:14550").expect("Couldn't create server");

            #[cfg(feature = "signing")]
            server.setup_signing(Some(singing_cfg_server));

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
        });

        // Give some time for the server to connect
        thread::sleep(std::time::Duration::from_millis(100));

        // have the client send a few hearbeats
        thread::spawn(move || {
            let msg =
                mavlink::common::MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg());
            #[allow(unused_mut)]
            let mut client =
                mavlink::connect("tcpout:127.0.0.1:14550").expect("Couldn't create client");

            #[cfg(feature = "signing")]
            client.setup_signing(Some(singing_cfg_client));

            for _i in 0..RECEIVE_CHECK_COUNT {
                client.send_default(&msg).ok();
            }
        });

        server_thread.join().unwrap();
    }

    /// Test whether we can send a message via TCP and receive it OK as a raw message. This also test signing as a property of a MavConnection if the signing feature is enabled.
    #[test]
    pub fn test_tcp_loopback_recv_raw() {
        const RECEIVE_CHECK_COUNT: i32 = 5;

        #[cfg(feature = "signing")]
        let singing_cfg_server = SigningConfig::new(test_shared::SECRET_KEY, 0, true, false);
        #[cfg(feature = "signing")]
        let singing_cfg_client = singing_cfg_server.clone();

        let server_thread = thread::spawn(move || {
            //TODO consider using get_available_port to use a random port
            #[allow(unused_mut)]
            let mut server = mavlink::connect::<mavlink::common::MavMessage>("tcpin:0.0.0.0:14560")
                .expect("Couldn't create server");

            #[cfg(feature = "signing")]
            server.setup_signing(Some(singing_cfg_server));

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
        });

        // Give some time for the server to connect
        thread::sleep(std::time::Duration::from_millis(100));

        // have the client send a few hearbeats
        thread::spawn(move || {
            let msg =
                mavlink::common::MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg());
            #[allow(unused_mut)]
            let mut client =
                mavlink::connect("tcpout:127.0.0.1:14560").expect("Couldn't create client");

            #[cfg(feature = "signing")]
            client.setup_signing(Some(singing_cfg_client));

            for _i in 0..RECEIVE_CHECK_COUNT {
                client.send_default(&msg).ok();
            }
        });

        server_thread.join().unwrap();
    }
}
