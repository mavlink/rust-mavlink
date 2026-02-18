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

- Read/write support for MAVLink v1 and v2
- TCP, UDP, Serial and File connection support. 
- Signing support.
- Blocking and async runtime support.
- `std` and embedded targets. (`embedded-io` / `embedded-hal` compatibility)
- Codegen tool mavlink-bindgen for creating Rust bindings from MAVLink XML dialect definitions.

## Workspace crates

| Crate | Purpose |
| --- | --- |
| [`mavlink`](https://crates.io/crates/mavlink) | Main crate with generated dialect modules and high-level APIs |
| [`mavlink-core`](https://crates.io/crates/mavlink-core) | Core protocol types, parser/serializer, and connection traits |
| [`mavlink-bindgen`](https://crates.io/crates/mavlink-bindgen) | XML-to-Rust code generator used by `mavlink` |

## Quick start

Add to `Cargo.toml`:

```toml
[dependencies]
mavlink = "0.17"
```

```rust
use mavlink::{MavConnection, ardupilotmega};

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

Note that building `mavlink` requires `git`, because the build script initializes and updates
MAVLink definition submodules before generating dialect code.

## Supported address formats

- `tcpin:<addr>:<port>`: TCP server
- `tcpout:<addr>:<port>`: TCP client
- `udpin:<addr>:<port>`: UDP listener
- `udpout:<addr>:<port>`: UDP sender
- `udpcast:<addr>:<port>`: UDP broadcast sender
- `serial:<port>:<baudrate>`: serial port connection
- `file:<path>`: read MAVLink frames from a file

## Feature flags

- Transport/runtime:
  - `std` (default)
  - `udp`, `tcp`, `direct-serial` (default)
  - `tokio-1` (async APIs)
- Protocol/data:
  - `serde` (default)
  - `signing` (MAVLink 2 message signing)
  - `emit-extensions`
  - `format-generated-code` (default)
- Embedded:
  - `embedded` (`embedded-io` based)
  - `embedded-hal-02` (`embedded-hal` 0.2 compatibility)
- Dialects:
  - `all`
  - `ardupilotmega`
  - `asluav`
  - `avssuas`
  - `common`
  - `csairlink`
  - `cubepilot`
  - `development`
  - `icarous`
  - `loweheiser`
  - `marsh`
  - `matrixpilot`
  - `minimal`
  - `paparazzi`
  - `python_array_test`
  - `standard`
  - `stemstudios`
  - `storm32`
  - `test`
  - `ualberta`
  - `uavionix`
  - `all-dialects` (enables all dialect feature flags)

## Examples

See [`mavlink/examples/`](mavlink/examples/) for all examples and run instructions.

## Maintainers
See [MAINTAINERS.md](MAINTAINERS.md) for active maintainers, release managers and their contact details.

## Maintainers
See [MAINTAINERS.md](MAINTAINERS.md) for active maintainers, release managers and their contact details.

## License

Licensed under either of
 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.
