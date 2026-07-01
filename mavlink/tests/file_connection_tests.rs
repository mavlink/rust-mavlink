mod test_shared;

#[cfg(feature = "dialect-common")]
mod test_file_connections {
    use mavlink::MavConnection;
    use mavlink::dialects::ardupilotmega::MavMessage;

    const ACCEPTED_STREAM_MESSAGES: usize = 878;
    const REAL_MAVLINK_STREAM: &str = "tests/parity/real_mavlink_stream.bin";

    /// Test whether we can send a message via TCP and receive it OK using async_connect.
    /// This also test signing as a property of a MavConnection if the mav2-message-signing feature is enabled.
    #[cfg(feature = "tokio")]
    #[tokio::test]
    pub async fn test_file_async_read_raw() {
        let stream = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join(REAL_MAVLINK_STREAM)
            .canonicalize()
            .unwrap();

        let stream = stream.to_str().unwrap();

        let filename = std::path::Path::new(stream);
        let filename = filename.to_str().unwrap();
        dbg!(filename);

        println!("Processing file: {filename}");
        let connection_string = format!("file:{filename}");

        println!("connection_string - {connection_string}");

        let vehicle = mavlink::connect_async::<MavMessage>(&connection_string)
            .await
            .expect("Couldn't read from file");

        let mut counter = 0;
        loop {
            match vehicle.recv_raw().await {
                Ok(raw_msg) => {
                    println!(
                        "raw_msg.component_id() {} | sequence number {} | message_id {:?}",
                        raw_msg.component_id(),
                        raw_msg.sequence(),
                        raw_msg.message_id()
                    );

                    counter += 1;
                }
                Err(mavlink::error::MessageReadError::Io(e)) => {
                    if e.kind() == tokio::io::ErrorKind::UnexpectedEof {
                        break;
                    }
                }
                _ => {
                    break;
                }
            }
        }

        println!("Number of parsed messages: {counter}");
        assert_eq!(counter, ACCEPTED_STREAM_MESSAGES);
    }

    #[test]
    pub fn test_file_read_raw() {
        let stream = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join(REAL_MAVLINK_STREAM)
            .canonicalize()
            .unwrap();

        let stream = stream.to_str().unwrap();

        let filename = std::path::Path::new(stream);
        let filename = filename.to_str().unwrap();
        dbg!(filename);

        println!("Processing file: {filename}");
        let connection_string = format!("file:{filename}");

        println!("connection_string - {connection_string}");

        let vehicle =
            mavlink::connect::<MavMessage>(&connection_string).expect("Couldn't read from file");

        let mut counter = 0;
        loop {
            match vehicle.recv_raw() {
                Ok(raw_msg) => {
                    println!(
                        "raw_msg.component_id() {} | sequence number {} | message_id {:?}",
                        raw_msg.component_id(),
                        raw_msg.sequence(),
                        raw_msg.message_id()
                    );

                    counter += 1;
                }
                Err(mavlink::error::MessageReadError::Io(e)) => {
                    if e.kind() == tokio::io::ErrorKind::UnexpectedEof {
                        break;
                    }
                }
                _ => {
                    break;
                }
            }
        }

        println!("Number of parsed messages: {counter}");
        assert_eq!(counter, ACCEPTED_STREAM_MESSAGES);
    }
}
