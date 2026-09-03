mod test_shared;

#[cfg(all(
    feature = "std",
    feature = "dialect-common",
    feature = "transport-udp",
    feature = "transport-tcp"
))]
mod socket_configs {
    use std::net::{TcpListener, TcpStream, UdpSocket};
    use std::time::Duration;

    use mavlink::dialects::common::MavMessage;
    use mavlink::{Connectable, MavConnection, TcpConfig, UdpConfig, UdpMode};

    fn heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(crate::test_shared::get_heartbeat_msg())
    }

    #[test]
    fn supplied_udp_sockets_exchange_messages_and_are_one_shot() {
        let input = UdpSocket::bind("127.0.0.1:0").unwrap();
        input
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let input_address = input.local_addr().unwrap();
        let input_config = UdpConfig::from_socket(input, UdpMode::Udpin).unwrap();
        assert_eq!(input_config.to_string(), format!("udpin:{input_address}"));

        let output = UdpSocket::bind("127.0.0.1:0").unwrap();
        output.connect(input_address).unwrap();
        let output_config = UdpConfig::from_socket(output, UdpMode::Udpout).unwrap();

        let receiver = input_config.connect::<MavMessage>().unwrap();
        let sender = output_config.connect::<MavMessage>().unwrap();
        sender.send_default(&heartbeat()).unwrap();
        assert!(matches!(
            receiver.recv().unwrap().1,
            MavMessage::HEARTBEAT(_)
        ));

        assert!(input_config.connect::<MavMessage>().is_err());
        assert!(output_config.connect::<MavMessage>().is_err());
    }

    #[test]
    fn supplied_udp_broadcast_requires_a_peer_and_enables_broadcast() {
        let unconnected = UdpSocket::bind("127.0.0.1:0").unwrap();
        assert!(UdpConfig::from_socket(unconnected, UdpMode::UdpBroadcast).is_err());

        let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
        socket.set_broadcast(true).unwrap();
        socket.connect("255.255.255.255:14550").unwrap();
        socket.set_broadcast(false).unwrap();
        let check = socket.try_clone().unwrap();
        let config = UdpConfig::from_socket(socket, UdpMode::UdpBroadcast).unwrap();
        let _connection = config.connect::<MavMessage>().unwrap();
        assert!(check.broadcast().unwrap());
    }

    #[test]
    fn supplied_tcp_listener_and_stream_exchange_messages_and_are_one_shot() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let server_config = TcpConfig::from_listener(listener).unwrap();
        assert_eq!(server_config.to_string(), format!("tcpin:{address}"));

        let server_config_for_thread = server_config.clone();
        let server =
            std::thread::spawn(move || server_config_for_thread.connect::<MavMessage>().unwrap());
        let stream = TcpStream::connect(address).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let client_config = TcpConfig::from_stream(stream).unwrap();
        let client = client_config.connect::<MavMessage>().unwrap();
        let server = server.join().unwrap();

        client.send_default(&heartbeat()).unwrap();
        assert!(matches!(server.recv().unwrap().1, MavMessage::HEARTBEAT(_)));
        assert!(server_config.connect::<MavMessage>().is_err());
        assert!(client_config.connect::<MavMessage>().is_err());
    }

    #[cfg(feature = "tokio")]
    mod asynchronous {
        use super::*;
        use mavlink::AsyncConnectable;

        #[tokio::test]
        async fn supplied_udp_sockets_work_async() {
            let input = UdpSocket::bind("127.0.0.1:0").unwrap();
            let address = input.local_addr().unwrap();
            let output = UdpSocket::bind("127.0.0.1:0").unwrap();
            output.connect(address).unwrap();

            let input_config = UdpConfig::from_socket(input, UdpMode::Udpin).unwrap();
            let output_config = UdpConfig::from_socket(output, UdpMode::Udpout).unwrap();
            let receiver = input_config.connect_async::<MavMessage>().await.unwrap();
            let sender = output_config.connect_async::<MavMessage>().await.unwrap();
            sender.send_default(&heartbeat()).await.unwrap();
            let received = tokio::time::timeout(Duration::from_secs(2), receiver.recv())
                .await
                .unwrap()
                .unwrap();
            assert!(matches!(received.1, MavMessage::HEARTBEAT(_)));
            assert!(input_config.connect_async::<MavMessage>().await.is_err());
        }

        #[tokio::test]
        async fn supplied_tcp_listener_and_stream_work_async() {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let address = listener.local_addr().unwrap();
            let server_config = TcpConfig::from_listener(listener).unwrap();
            let stream = TcpStream::connect(address).unwrap();
            let client_config = TcpConfig::from_stream(stream).unwrap();

            let (server, client) = tokio::join!(
                server_config.connect_async::<MavMessage>(),
                client_config.connect_async::<MavMessage>()
            );
            let server = server.unwrap();
            let client = client.unwrap();
            client.send_default(&heartbeat()).await.unwrap();
            let received = tokio::time::timeout(Duration::from_secs(2), server.recv())
                .await
                .unwrap()
                .unwrap();
            assert!(matches!(received.1, MavMessage::HEARTBEAT(_)));
            assert!(client_config.connect_async::<MavMessage>().await.is_err());
        }
    }
}
