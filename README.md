# rust-mavlink

[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)
[![Crate info](https://img.shields.io/crates/v/mavlink.svg)](https://crates.io/crates/mavlink)
[![Documentation](https://docs.rs/mavlink/badge.svg)](https://docs.rs/mavlink)

Rust implementation of the [MAVLink](http://qgroundcontrol.org/mavlink/start) UAV messaging protocol,
with bindings for all message sets.

Add to your Cargo.toml:

```
mavlink = "0.10.1"
```

## Examples
See [src/bin/mavlink-dump.rs](src/bin/mavlink-dump.rs) for a usage example.

It's also possible to install the working example via `cargo` command line:
```sh
cargo install mavlink
```

### Community projects
Check some projects built by the community:
- [mavlink2rest](https://github.com/patrickelectric/mavlink2rest): A REST server that provides easy and friendly access to mavlink messages.
- [mavlink-camera-manager](https://github.com/patrickelectric/mavlink-camera-manager): Extensible cross-platform camera server.

## License

Licensed under either of
 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.

