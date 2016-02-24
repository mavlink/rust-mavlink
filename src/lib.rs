extern crate byteorder;
extern crate time;
extern crate mio;
extern crate bytes;
extern crate crc16;
extern crate eventual;
extern crate bit_vec;

pub mod connection;

use connection::{VehicleConnection, DkHandler};
use std::net::SocketAddr;
use std::sync::mpsc::channel;
use mio::tcp::TcpStream;
use std::collections::VecDeque;
use std::thread;

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_variables)]
#[allow(unused_mut)]
pub mod common {
    include!(concat!(env!("OUT_DIR"), "/common.rs"));
}

pub fn connect(address: SocketAddr) -> VehicleConnection {
    // Create a new event loop, panic if this fails.
    let socket = match TcpStream::connect(&address) {
        Ok(socket) => socket,
        Err(e) => {
            // If the connect fails here, then usually there is something
            // wrong locally. Though, on some operating systems, attempting
            // to connect to a localhost address completes immediately.
            panic!("failed to create socket; err={:?}", e);
        }
    };

    let mut event_loop = mio::EventLoop::new().unwrap();

    let (tx, rx) = channel();
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

    return VehicleConnection {
        tx: vehicle_tx,
        rx: rx,
        msg_id: 0,
        started: false,
        buffer: VecDeque::new(),
    };
}
