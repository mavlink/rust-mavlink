# rust-mavlink

## Avalor Fork Modifications

This branch splits from rust-mavlink, continuing from version 0.32.2. It addresses the following:
- Contains fixes which are in their github, but for which they didn't create a new crate.
- Some fields that should be bitmasks are recognized as enums. This leads to messages with bitmasks that have multiple bits set being discarded wrongfully.

Two additional MavCmds namely:
- `AVALOR_CUSTOM_AUTERION_FLAP_CHECK` is added to support flap checks on vehicles which have a version lower than 3.0.0
- `MAV_CMD_EXTERNAL_POSITION_ESTIMATE` is added to be able to correct the vehicle position when GPS is not available.

## Info
[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)
[![Crate info](https://img.shields.io/crates/v/mavlink.svg)](https://crates.io/crates/mavlink)
[![Documentation](https://docs.rs/mavlink/badge.svg)](https://docs.rs/mavlink)

Rust implementation of the [MAVLink](https://mavlink.io/en) UAV messaging protocol,
with bindings for all message sets.

Add to your Cargo.toml:

```
mavlink = "0.12.2"
```

## Examples
See [examples/](mavlink/examples/mavlink-dump/src/main.rs) for different usage examples.

### mavlink-dump
[examples/mavlink-dump](mavlink/examples/mavlink-dump/src/main.rs) contains an executable example that can be used to test message reception.

It can be executed directly by running:
```
cargo run --example mavlink-dump [options]
```

It's also possible to install the working example via `cargo` command line:
```sh
cargo install --path examples/mavlink-dump
```

It can then be executed by running:
```
mavlink-dump [options]
```

Execution call example:
```sh
mavlink-dump udpin:127.0.0.1:14540
```

### Community projects
Check some projects built by the community:
- [mavlink2rest](https://github.com/patrickelectric/mavlink2rest): A REST server that provides easy and friendly access to mavlink messages.
- [mavlink-camera-manager](https://github.com/mavlink/mavlink-camera-manager): Extensible cross-platform camera server.

## License

Licensed under either of
 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.

