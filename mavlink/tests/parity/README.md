# Mavlink Parser Parity Fixtures

This directory contains tools and data used to compare the Rust Mavlink parser
with the C parser.

`real_mavlink_stream.bin` is a raw Mavlink byte stream captured from an actual
Holybro Pixhawk 4 Mini flight controller over a serial connection.

The file is intentionally stored as `.bin` because it is not a Mavlink log
container. It does not contain `.tlog` timestamps, QGroundControl metadata or
any other wrapper format. It is the byte stream as read from the flight
controller serial device.

The captured file is used by the parity test as a real-world stream fixture. The
test parses it with both implementations and fails if the Rust parser misses a
frame that the C parser accepts.
