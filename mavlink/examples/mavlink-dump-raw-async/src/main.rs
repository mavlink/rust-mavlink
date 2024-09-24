use mavlink::error::MessageReadError;
use std::{env, sync::Arc, thread, time::Duration};

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args: Vec<_> = env::args().collect();

    if args.len() < 2 {
        println!(
            "Usage: mavlink-dump-raw (tcpout|tcpin|udpout|udpin|udpbcast|serial|file):(ip|dev|path):(port|baud)"
        );
        return;
    }

    // It's possible to change the mavlink dialect to be used in the connect call
    let mut mavconn = mavlink::connect_async::<mavlink::ardupilotmega::MavMessage>(&args[1])
        .await
        .unwrap();

    let vehicle = Arc::new(mavconn);
    vehicle
        .send(&mavlink::MavHeader::default(), &request_parameters())
        .await
        .unwrap();
    vehicle
        .send(&mavlink::MavHeader::default(), &request_stream())
        .await
        .unwrap();

    let vehicle_clone = vehicle.clone();
    tokio::spawn(async move {
        let res = vehicle_clone.send_default(&heartbeat_message()).await;
        if res.is_ok() {
            tokio::time::sleep(Duration::from_secs(1)).await;
        } else {
            println!("send failed: {res:?}");
        }
    });

    loop {
        match vehicle.recv_raw().await {
            Ok(raw_msg) => match raw_msg {
                mavlink::MAVLinkRawMessage::V1(msg) => {
                    println!("received v1 raw message (id = {})", msg.message_id());
                }
                mavlink::MAVLinkRawMessage::V2(msg) => {
                    println!("received v2 raw message (id = {})", msg.message_id());
                }
            },
            Err(MessageReadError::Io(e)) => {
                if e.kind() == std::io::ErrorKind::WouldBlock {
                    //no messages currently available to receive -- wait a while
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    continue;
                } else {
                    println!("recv error: {e:?}");
                    break;
                }
            }
            // messages that didn't get through due to parser errors are ignored
            _ => {}
        }
    }
}

/// Create a heartbeat message using 'ardupilotmega' dialect
pub fn heartbeat_message() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

/// Create a message requesting the parameters list
pub fn request_parameters() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::PARAM_REQUEST_LIST(
        mavlink::ardupilotmega::PARAM_REQUEST_LIST_DATA {
            target_system: 0,
            target_component: 0,
        },
    )
}

/// Create a message enabling data streaming
pub fn request_stream() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::REQUEST_DATA_STREAM(
        mavlink::ardupilotmega::REQUEST_DATA_STREAM_DATA {
            target_system: 0,
            target_component: 0,
            req_stream_id: 0,
            req_message_rate: 10,
            start_stop: 1,
        },
    )
}
