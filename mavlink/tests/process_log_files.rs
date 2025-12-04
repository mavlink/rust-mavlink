#[cfg(all(feature = "default", feature = "ardupilotmega"))]
mod process_files {
    use mavlink::ardupilotmega::MavMessage;
    use mavlink::error::MessageReadError;
    use mavlink::MavConnection;

    #[test]
    pub fn get_file() {
        // Get path for download script
        let tlog = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/log.tlog")
            .canonicalize()
            .unwrap();

        let tlog = tlog.to_str().unwrap();

        let filename = std::path::Path::new(tlog);
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
        assert!(
            counter == 1426,
            "Unable to hit the necessary amount of matches"
        );
    }
}
