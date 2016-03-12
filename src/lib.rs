extern crate byteorder;
extern crate time;
extern crate mio;
extern crate bytes;
extern crate crc16;
extern crate eventual;
extern crate bit_vec;
extern crate chan;

pub mod connection;

use connection::{VehicleConnection, DkHandler};
use std::net::ToSocketAddrs;
use mio::tcp::TcpStream;
use std::collections::VecDeque;
use std::thread;
use std::io;

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod common {
    include!(concat!(env!("OUT_DIR"), "/common.rs"));
}

pub fn connect<T: ToSocketAddrs>(address: T) -> io::Result<VehicleConnection> {
    // Create a new event loop, panic if this fails.
    let socket = try!(TcpStream::connect(&address.to_socket_addrs().unwrap().next().unwrap()));

    let mut event_loop = mio::EventLoop::new().unwrap();

    let (tx, rx) = chan::async();
    let vehicle_tx = event_loop.channel();

    thread::spawn(move || {
        println!("running pingpong socket");
        let mut handler = DkHandler {
            socket: socket,
            buf: vec![],
            vehicle_tx: tx,
            watchers: vec![],
        };
        handler.register(&mut event_loop);
        event_loop.run(&mut handler).unwrap();
    });

    Ok(VehicleConnection {
        tx: vehicle_tx,
        rx: rx,
        msg_id: 0,
        started: false,
        buffer: VecDeque::new(),
    })
}

pub fn heartbeat_message() -> common::MavMessage {
    common::MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: 6,
        autopilot: 8,
        base_mode: 0,
        system_status: 0,
        mavlink_version: 0x3,
    })
}

pub fn request_parameters() -> common::MavMessage {
    common::MavMessage::PARAM_REQUEST_LIST(common::PARAM_REQUEST_LIST_DATA {
        target_system: 0,
        target_component: 0,
    })
}

pub fn request_stream() -> common::MavMessage {
    common::MavMessage::REQUEST_DATA_STREAM(common::REQUEST_DATA_STREAM_DATA {
        target_system: 0,
        target_component: 0,
        req_stream_id: 0,
        req_message_rate: 10,
        start_stop: 1,
    })
}
