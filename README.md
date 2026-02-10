# rust-mavlink

[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)
[![Crate info](https://img.shields.io/crates/v/mavlink.svg)](https://crates.io/crates/mavlink)
[![Documentation](https://docs.rs/mavlink/badge.svg)](https://docs.rs/mavlink)

Rust implementation of the [MAVLink](https://mavlink.io/en) UAV messaging protocol,
with bindings for all message sets.

Add to your Cargo.toml:

```
mavlink = "0.16"
```

Building this crate requires `git`.

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

## Maintainers
See [MAINTAINERS.md](MAINTAINERS.md) for active maintainers, release managers and their contact details.

## License

Licensed under either of
 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.
