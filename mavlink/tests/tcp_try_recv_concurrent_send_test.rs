mod test_shared;

#[cfg(all(feature = "std", feature = "transport-tcp", feature = "dialect-common"))]
mod tcp_try_recv_concurrent_send {
    use std::io::{ErrorKind, Write};
    use std::net::{TcpListener, TcpStream};
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread;
    use std::time::{Duration, Instant};

    use mavlink::dialects::common::{FILE_TRANSFER_PROTOCOL_DATA, HEARTBEAT_DATA, MavMessage};
    use mavlink::error::MessageReadError;
    use mavlink::{MavConnection, MavHeader};

    const MESSAGES: usize = 20_000;

    /// `try_recv` must not make a concurrent `send` fail under backpressure (#570).
    #[test]
    fn try_recv_does_not_break_concurrent_send() {
        let server = thread::spawn(|| {
            let server = mavlink::connect::<MavMessage>("tcpin:127.0.0.1:14571")
                .expect("Couldn't create server");
            // Let the client's socket buffers fill up before draining them.
            thread::sleep(Duration::from_millis(500));
            let mut received = 0;
            while received < MESSAGES {
                match server.recv() {
                    Ok((_, MavMessage::FILE_TRANSFER_PROTOCOL(_))) => received += 1,
                    Ok(_) => panic!("unexpected message"),
                    Err(e) => panic!("receive failed after {received} messages: {e:?}"),
                }
            }
            received
        });

        thread::sleep(Duration::from_millis(100));
        let client = Arc::new(
            mavlink::connect::<MavMessage>("tcpout:127.0.0.1:14571")
                .expect("Couldn't create client"),
        );

        let done = Arc::new(AtomicBool::new(false));
        let poller = {
            let client = Arc::clone(&client);
            let done = Arc::clone(&done);
            thread::spawn(move || {
                while !done.load(Ordering::Relaxed) {
                    let _ = client.try_recv();
                }
            })
        };

        let msg = MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
            target_network: 0,
            target_system: 1,
            target_component: 1,
            payload: [0xA5; 251],
        });
        for i in 0..MESSAGES {
            client
                .send_default(&msg)
                .unwrap_or_else(|e| panic!("send {i} failed while try_recv was polling: {e:?}"));
        }

        done.store(true, Ordering::Relaxed);
        poller.join().unwrap();
        assert_eq!(server.join().unwrap(), MESSAGES);
    }

    fn connected_pair() -> (TcpStream, mavlink::Connection<MavMessage>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let client = mavlink::connect::<MavMessage>(&format!("tcpout:{address}"))
            .expect("Couldn't create client");
        let (server, _) = listener.accept().unwrap();
        (server, client)
    }

    fn is_no_message(result: &Result<(MavHeader, MavMessage), MessageReadError>) -> bool {
        matches!(result, Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock)
    }

    /// With nothing to read, `try_recv` returns at once and leaves the socket blocking.
    #[test]
    fn try_recv_returns_promptly_without_data() {
        let (_server, client) = connected_pair();

        let start = Instant::now();
        for _ in 0..100 {
            assert!(is_no_message(&client.try_recv()));
        }
        assert!(
            start.elapsed() < Duration::from_millis(200),
            "100 empty try_recv calls took {:?}",
            start.elapsed()
        );

        // `tcpout` reads time out after 100 ms. A `recv` that returns much sooner
        // would mean the socket was left in nonblocking mode.
        let start = Instant::now();
        assert!(client.recv().is_err());
        assert!(start.elapsed() >= Duration::from_millis(50));
    }

    /// A frame that arrives in two pieces is kept across `try_recv` calls.
    #[test]
    fn try_recv_keeps_frame_split_across_reads() {
        let (mut server, client) = connected_pair();
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        let mut frame = Vec::new();
        mavlink::write_v2_msg(&mut frame, MavHeader::default(), &msg).unwrap();
        let (first, rest) = frame.split_at(7);

        server.write_all(first).unwrap();
        server.flush().unwrap();
        let deadline = Instant::now() + Duration::from_millis(200);
        while Instant::now() < deadline {
            assert!(is_no_message(&client.try_recv()));
        }

        server.write_all(rest).unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let result = client.try_recv();
            if !is_no_message(&result) {
                assert_eq!(result.unwrap().1, msg);
                break;
            }
            assert!(Instant::now() < deadline, "frame never completed");
        }
    }
}
