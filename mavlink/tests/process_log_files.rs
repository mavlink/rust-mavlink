#[cfg(all(feature = "default", feature = "dialect-ardupilotmega"))]
mod process_files {
    use mavlink::MavConnection;
    use mavlink::dialects::ardupilotmega::MavMessage;
    use mavlink::error::MessageReadError;

    const ACCEPTED_STREAM_MESSAGES: usize = 878;
    const REAL_MAVLINK_STREAM: &str = "tests/parity/real_mavlink_stream.bin";

    #[test]
    pub fn get_file() {
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

        // Process file
        process_file(&connection_string);
    }

    pub fn process_file(connection_string: &str) {
        let vehicle = mavlink::connect::<MavMessage>(connection_string);
        assert!(vehicle.is_ok(), "Incomplete address should error");

        let vehicle = vehicle.unwrap();
        let mut counter = 0;
        loop {
            match vehicle.recv() {
                Ok((_header, _msg)) => {
                    counter += 1;
                }
                Err(MessageReadError::Io(e)) => {
                    if e.kind() == std::io::ErrorKind::WouldBlock {
                        continue;
                    } else {
                        println!("recv error: {e:?}");
                        break;
                    }
                }
                _ => {}
            }
        }

        println!("Number of parsed messages: {counter}");
        assert_eq!(counter, ACCEPTED_STREAM_MESSAGES);
    }
}
