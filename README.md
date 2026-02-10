# rust-mavlink

[![Crate info](https://img.shields.io/crates/v/mavlink.svg)](https://crates.io/crates/mavlink)
[![Crate downloads](https://img.shields.io/crates/d/mavlink.svg)](https://crates.io/crates/mavlink)
[![Rust 1.80+](https://img.shields.io/badge/rust-1.80%2B-blue.svg)](https://github.com/mavlink/rust-mavlink/blob/master/Cargo.toml)
[![License](https://img.shields.io/crates/l/mavlink.svg)](https://github.com/mavlink/rust-mavlink#license)
[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)
[![Documentation](https://docs.rs/mavlink/badge.svg)](https://docs.rs/mavlink)

Pure Rust implementation of the [MAVLink](https://mavlink.io/en) UAV messaging protocol.
Provides strongly typed message bindings, frame encode/decode and connection APIs for serial,
UDP, TCP, file connection protocols with a rich set of features.

## What rust-mavlink provides

- Rust bindings for MAVLink dialects
- Read/write support for MAVLink v1 and v2
- TCP, UDP, Serial and File connection support. 
- Signing support.
- Blocking and async support.
- std and embedded support.
- Codegen tool mavlink-bindgen for creating Rust bindings from MAVLink XML dialect definitions.

## Workspace crates

| Crate | Purpose |
| --- | --- |
| [`mavlink`](https://crates.io/crates/mavlink) | Main crate with generated dialect modules and high-level APIs |
| [`mavlink-core`](https://crates.io/crates/mavlink-core) | Core protocol types, parser/serializer, and connection traits |
| [`mavlink-bindgen`](https://crates.io/crates/mavlink-bindgen) | XML-to-Rust code generator used by `mavlink` |

## Build requirements

Building the `mavlink` crate runs a build script that initializes
the bundled MAVLink definition submodule, so `git` must be available.

## Quick start

Add the crate:

```toml
[dependencies]
mavlink = "0.17.1"
```

Simple code example:

```rust
use mavlink::{dialects::ardupilotmega, MavConnection};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let conn = mavlink::connect::<ardupilotmega::MavMessage>("udpin:0.0.0.0:14550")?;

    let heartbeat = ardupilotmega::MavMessage::HEARTBEAT(ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
        autopilot: ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: ardupilotmega::MavModeFlag::empty(),
        system_status: ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    });

    conn.send(&mavlink::MavHeader::default(), &heartbeat)?;
    let (header, message) = conn.recv()?;

    println!(
        "received from sys={}, comp={}: {message:?}",
        header.system_id, header.component_id
    );

    Ok(())
}
```

## Supported address formats

- `tcpin:<addr>:<port>`: TCP server
- `tcpout:<addr>:<port>`: TCP client
- `udpin:<addr>:<port>`: UDP listener
- `udpout:<addr>:<port>`: UDP sender
- `udpbcast:<addr>:<port>`: UDP broadcast sender
- `serial:<port>:<baudrate>`: serial port connection
- `file:<path>`: read MAVLink frames from a file

## Feature flags

Transport and runtime:

- `embedded`
- `std`
- `tokio`
- `transport-direct-serial`
- `transport-tcp`
- `transport-udp`

Protocol and code generation:

- `arbitrary`
- `format-generated-code`
- `mav2-message-extensions`
- `mav2-message-signing`
- `serde`
- `ts-rs`

Dialects:

- `dialect-all`
- `dialect-ardupilotmega`
- `dialect-asluav`
- `dialect-avssuas`
- `dialect-common`
- `dialect-csairlink`
- `dialect-cubepilot`
- `dialect-development`
- `dialect-icarous`
- `dialect-loweheiser`
- `dialect-marsh`
- `dialect-matrixpilot`
- `dialect-minimal`
- `dialect-paparazzi`
- `dialect-python_array_test`
- `dialect-standard`
- `dialect-stemstudios`
- `dialect-storm32`
- `dialect-test`
- `dialect-ualberta`
- `dialect-uavionix`

Note that `std` and `embedded` features are mutually exclusive.

## Examples

See [`mavlink/examples/`](mavlink/examples/) for runnable examples.

## Maintainers

See [MAINTAINERS.md](MAINTAINERS.md) for active maintainers, release managers and contact details.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT) at your option.
