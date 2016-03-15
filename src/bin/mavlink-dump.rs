extern crate mavlink;

use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;
use std::env;

fn main() {
    let args: Vec<_> = env::args().collect();

    if args.len() < 2 {
        println!("Usage: mavlink-dump (tcp|udpin|udpout):ip:port");
        return;
    }

    let vlock = Arc::new(RwLock::new(mavlink::connect(&args[1]).unwrap()));
    let vehicle_recv = vlock.read().unwrap().wait_recv();

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

    let mut first = true;
    loop {
        vehicle_recv.sleep();

        if first {
            let mut vehicle = vlock.write().unwrap();
            vehicle.send(mavlink::request_parameters());
            vehicle.send(mavlink::request_stream());
            first = false;
        }

        if let Ok(msg) = vlock.write().unwrap().recv() {
            println!("{:?}", msg);
        } else {
            break;
        }
    }
}