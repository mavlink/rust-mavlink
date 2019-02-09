extern crate mavlink;

#[cfg(test)]
#[cfg(all(feature = "std", feature = "tcp"))]
mod test_tcp_connections {
    use std::thread;

    /// Create a heartbeat message
    pub fn heartbeat_message() -> mavlink::common::MavMessage {
        mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
            autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: mavlink::common::MavModeFlag::empty(),
            system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
            mavlink_version: 0x3,
        })
    }

    /// Test whether we can send a message via TCP and receive it OK
    #[test]
    pub fn test_tcp_loopback() {
        const RECEIVE_CHECK_COUNT: i32 = 3;

        let server_thread = thread::spawn( {
            move || {
                //TODO consider using get_available_port to use a random port
                let server = mavlink::connect("tcpin:0.0.0.0:14550")
                    .expect("Couldn't create server");

                let mut recv_count = 0;
                for _i in 0..RECEIVE_CHECK_COUNT {
                    if let Ok( (_header, msg) ) = server.recv() {
                        if let mavlink::common::MavMessage::HEARTBEAT(_heartbeat_msg) = msg {
                            recv_count += 1;
                        }
                    } else {
                        // one message parse failure fails the test
                        break;
                    }
                }
                assert_eq!(recv_count, RECEIVE_CHECK_COUNT);
            }
        });

        // have the client send a few hearbeats
        thread::spawn({
            move || {
                let client = mavlink::connect("tcpout:127.0.0.1:14550")
                    .expect("Couldn't create client");
                for _i in 0..RECEIVE_CHECK_COUNT {
                    client.send_default(&heartbeat_message()).ok();
                }
            }
        });

        server_thread.join().unwrap();
    }

}