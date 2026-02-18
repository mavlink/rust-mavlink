# Examples

This directory contains runnable examples for different targets.

## Available examples

- [`mavlink-dump`](mavlink-dump/src/main.rs): desktop example for receiving and printing MAVLink frames.
- [`embedded`](embedded): blocking embedded example.
- [`embedded-async-read`](embedded-async-read): async embedded read loop example.

## Running `mavlink-dump`

From the workspace root:

```bash
cargo run --example mavlink-dump -- udpin:127.0.0.1:14540
```

The first argument is a MAVLink connection string such as:

- `udpin:0.0.0.0:14550`
- `udpout:127.0.0.1:14550`
- `tcpin:0.0.0.0:5760`
- `tcpout:127.0.0.1:5760`
- `serial:/dev/ttyUSB0:115200`
- `file:./mavlink/tests/log.tlog`

## Running embedded examples

See the per-example docs:

- [`embedded/README.md`](embedded/README.md)
- [`embedded-async-read/README.md`](embedded-async-read/README.md)
