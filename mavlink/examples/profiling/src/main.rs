use mavlink::MavHeader;
use mavlink::dialects::common::{
    HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
};
use mavlink_core::peek_reader::PeekReader;

use std::env;
use std::hint::black_box;

const DEFAULT_ITERATIONS: usize = 1_000_000;

fn heartbeat_message() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    })
}

fn header() -> MavHeader {
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    }
}

fn encoded_v1_frame() -> Vec<u8> {
    let mut buffer = Vec::with_capacity(280);

    mavlink::write_v1_msg(&mut buffer, header(), &heartbeat_message())
        .expect("failed to encode MAVLink v1 frame");

    buffer
}

fn encoded_v2_frame() -> Vec<u8> {
    let mut buffer = Vec::with_capacity(280);

    mavlink::write_v2_msg(&mut buffer, header(), &heartbeat_message())
        .expect("failed to encode MAVLink v2 frame");

    buffer
}

fn profile_parse_v1(iterations: usize) {
    let frame = encoded_v1_frame();

    for _ in 0..iterations {
        let mut reader = PeekReader::new(frame.as_slice());

        let result = mavlink::read_v1_msg::<MavMessage, _>(&mut reader)
            .expect("failed to parse MAVLink v1 frame");

        black_box(result);
    }
}

fn profile_parse_v2(iterations: usize) {
    let frame = encoded_v2_frame();

    for _ in 0..iterations {
        let mut reader = PeekReader::new(frame.as_slice());

        let result = mavlink::read_v2_msg::<MavMessage, _>(&mut reader)
            .expect("failed to parse MAVLink v2 frame");

        black_box(result);
    }
}

fn profile_serialize_v1(iterations: usize) {
    let message = heartbeat_message();
    let header = header();
    let mut buffer = [0u8; 280];

    for _ in 0..iterations {
        let mut output = &mut buffer[..];

        let len = mavlink::write_v1_msg(&mut output, header, &message)
            .expect("failed to serialize MAVLink v1 frame");

        black_box(len);
        black_box(&buffer);
    }
}

fn profile_serialize_v2(iterations: usize) {
    let message = heartbeat_message();
    let header = header();
    let mut buffer = [0u8; 280];

    for _ in 0..iterations {
        let mut output = &mut buffer[..];

        let len = mavlink::write_v2_msg(&mut output, header, &message)
            .expect("failed to serialize MAVLink v2 frame");

        black_box(len);
        black_box(&buffer);
    }
}

fn usage(program: &str) {
    eprintln!(
        "Usage: {program} <workload> [iterations]\n\
         \n\
         Workloads:\n\
         \tparse-v1\n\
         \tparse-v2\n\
         \tserialize-v1\n\
         \tserialize-v2\n\
         \n\
         Default iterations: {DEFAULT_ITERATIONS}"
    );
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        usage(&args[0]);
        std::process::exit(2);
    }

    let iterations = args
        .get(2)
        .map(|value| {
            value
                .parse::<usize>()
                .expect("iterations must be a positive integer")
        })
        .unwrap_or(DEFAULT_ITERATIONS);

    match args[1].as_str() {
        "parse-v1" => profile_parse_v1(iterations),
        "parse-v2" => profile_parse_v2(iterations),
        "serialize-v1" => profile_serialize_v1(iterations),
        "serialize-v2" => profile_serialize_v2(iterations),
        _ => {
            usage(&args[0]);
            std::process::exit(2);
        }
    }
}
