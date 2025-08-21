mod test_shared;

#[cfg(all(feature = "tokio-1", feature = "tcp", feature = "common"))]
mod test_file_connections {
    use mavlink::ardupilotmega::MavMessage;

    /// Test whether we can send a message via TCP and receive it OK using async_connect.
    /// This also test signing as a property of a MavConnection if the signing feature is enabled.
    #[tokio::test]
    pub async fn test_file_async_read_raw() {
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

        println!("connection_string - {connection_string}");

        let vehicle = mavlink::connect_async::<MavMessage>(&connection_string)
                .await
                .expect("Couldn't read from file");

        let mut counter = 0;
        loop {
            match vehicle.recv_raw().await {
                Ok(raw_msg) => {
                    
                    println!("raw_msg.component_id() {} | sequence number {} | payload {:?}", raw_msg.component_id(), raw_msg.sequence(), raw_msg.payload());
                    println!("raw_msg.version() {:?}", raw_msg.version());

                    counter+=1;
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
        assert!(
            counter == 1426,
            "Unable to hit the necessary amount of matches"
        );
    }
}
