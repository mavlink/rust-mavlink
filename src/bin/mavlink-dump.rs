extern crate mavlink;

use std::env;
extern crate env_logger;

fn main() {
    env_logger::init().unwrap();
    let args: Vec<_> = env::args().collect();

    if args.len() < 2 {
        println!("Usage: mavlink-dump (tcp|udpin|udpout):ip:port");
        return;
    }

    let mut vehicle = mavlink::connect(&args[1]).unwrap();

    /*thread::spawn({
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
    });*/

    vehicle.send(&mavlink::request_parameters()).unwrap();
    vehicle.send(&mavlink::request_stream()).unwrap();

    loop {
        if let Ok(msg) = vehicle.recv() {
            println!("{:?}", msg);
        } else {
            break;
        }
    }
}