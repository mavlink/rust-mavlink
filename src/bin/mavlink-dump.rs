extern crate mavlink;

use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

fn main() {
    let vlock = Arc::new(RwLock::new(mavlink::connect("127.0.0.1:5760").unwrap()));

    {
        let mut vehicle = vlock.write().unwrap();
        vehicle.send(mavlink::heartbeat_message());
        vehicle.send(mavlink::request_parameters());
        vehicle.send(mavlink::request_stream());
    }

    thread::spawn({
        let vlock = vlock.clone();
        move || {
            loop {
                {
                    let mut vehicle = vlock.write().unwrap();
                    vehicle.send(mavlink::heartbeat_message());
                }
                thread::sleep(Duration::from_secs(1));
            }
        }
    });

    while let Ok(msg) = vlock.write().unwrap().recv() {
        println!("{:?}", msg);
    }
}