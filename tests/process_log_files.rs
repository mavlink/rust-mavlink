extern crate mavlink;

#[cfg(test)]
#[cfg(all(feature = "default", feature = "ardupilotmega"))]
mod process_files {
    use mavlink::ardupilotmega::MavMessage;
    use mavlink::error::MessageReadError;

    #[test]
    pub fn all() {
        // Get path for download script
        let test_script = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/download_log_files.sh")
            .canonicalize()
            .unwrap();
        let test_script = test_script.to_str().unwrap();

        // Run download script
        let result = std::process::Command::new("sh").arg(test_script).output();
        assert_eq!(result.is_ok(), true);

        // Grab all log files
        let test_path = "/tmp/testlogs/";
        let files = std::fs::read_dir(test_path).unwrap();
        let mut files = files
            .filter_map(Result::ok)
            .filter(|d| d.path().extension().unwrap().to_str() == Some("tlog"));

        while let Some(file) = files.next() {
            let filename = file.file_name();
            let filename = filename.to_str().unwrap();
            let filename = std::path::Path::new(test_path)
                .join(filename)
                .canonicalize()
                .unwrap();
            let filename = filename.to_str().unwrap();

            println!("Processing file: {}", &filename);
            let connection_string = format!("file:{}", &filename);

            // Process file
            process_file(&connection_string);
        }
    }

    pub fn process_file(connection_string: &str) {
        let vehicle = mavlink::connect::<MavMessage>(&connection_string);
        assert!(vehicle.is_ok(), "Incomplete address should error");

        let vehicle = vehicle.unwrap();
        let mut counter = 0;
        loop {
            match vehicle.recv() {
                Ok((_header, msg)) => {
                    counter += 1;
                }
                Err(MessageReadError::Io(e)) => match e.kind() {
                    std::io::ErrorKind::WouldBlock => {
                        continue;
                    }
                    _ => {
                        println!("recv error: {:?}", e);
                        break;
                    }
                },
                _ => {}
            }
        }

        println!("Number of parsed messages: {}", counter);
        assert!(
            counter > 22000,
            "Unable to hit the necessary amount of matches"
        );
    }
}
