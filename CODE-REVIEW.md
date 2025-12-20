# Code Review

## Findings (ordered by severity)

- [High] Signed 24-bit encode/decode is incorrect: decode zero-extends and encode rejects negative values.
  - mavlink-core/src/bytes.rs:145
  - mavlink-core/src/bytes_mut.rs:121
  - Impact: negative `int24` values are parsed as large positives and serialization panics for valid negative values.
  - Fix: sign-extend on read and use a negative MIN (-(1 << 23)) on write; consider unit tests for `i24` round-trips.

- [High] Build script requires `git`, mutates the submodule, and ignores non-zero exit statuses.
  - mavlink/build/main.rs:13
  - mavlink/build/main.rs:19
  - mavlink/build/main.rs:35
  - Impact: builds can fail in offline/crates.io environments or produce inconsistent definitions; patch failures are silently ignored because only spawn errors are checked.
  - Fix: ship patched XML definitions (or pre-generated Rust), avoid VCS operations in `build.rs`, and check `status.success()` for `git` calls if they remain.

- [Medium] `tokio-1` and `embedded` are documented as incompatible but not enforced, and they define duplicate async APIs.
  - mavlink-core/src/lib.rs:875
  - mavlink-core/src/lib.rs:899
  - Impact: enabling both features produces duplicate symbol errors and confusing APIs.
  - Fix: add a `compile_error!` guard for `cfg(all(feature = "tokio-1", feature = "embedded"))` or gate one set of functions with `not(feature = "embedded")`.

- [Medium] UDP (sync/async) `recv` loops swallow all errors, and file `recv` ignores non-EOF I/O errors.
  - mavlink-core/src/connection/udp.rs:90
  - mavlink-core/src/async_connection/udp.rs:92
  - mavlink-core/src/connection/file.rs:45
  - Impact: persistent I/O errors turn into infinite loops or busy-spins; callers cannot observe disconnects or socket failures.
  - Fix: only ignore parse/CRC errors; propagate `MessageReadError::Io` (except maybe `WouldBlock`/`UnexpectedEof` where appropriate).

- [Low] `PeekReader`/`AsyncPeekReader` rejects exact buffer-size reads even though docs say “more than BUFFER_SIZE”.
  - mavlink-core/src/peek_reader.rs:141
  - mavlink-core/src/async_peek_reader.rs:139
  - Impact: `peek_exact(BUFFER_SIZE)` panics; API behavior does not match documentation.
  - Fix: change `< BUFFER_SIZE` to `<= BUFFER_SIZE` and update docs/tests.

- [Low] Address format docs and examples use `udpbcast`, but the parser accepts `udpcast`.
  - mavlink-core/src/connectable.rs:78
  - mavlink-core/src/async_connection/mod.rs:91
  - mavlink/examples/mavlink-dump/src/main.rs:10
  - Impact: user confusion and copy/paste failures.
  - Fix: accept both or standardize documentation and CLI help on one string.

## Simplification / existing-crate opportunities

- Replace custom `bytes`/`bytes_mut` helpers with `byteorder::ByteOrder` and/or `bytes::Buf`/`BufMut` for most primitives, keeping a small custom helper only for `u24/i24`. This reduces bespoke parsing code and risk (the current i24 bug is a good example).
  - mavlink-core/src/bytes.rs
  - mavlink-core/src/bytes_mut.rs
  - mavlink-bindgen/src/parser.rs:806

- Drop `utils::RustDefault` in favor of `Default` (arrays implement `Default` on Rust 1.80). Update codegen to use `#[serde(default)]` instead of a custom default function.
  - mavlink-core/src/utils.rs
  - mavlink-bindgen/src/parser.rs:752
  - mavlink/src/lib.rs:68

- Consider splitting the 2300-line `mavlink-core/src/lib.rs` into focused modules (frame structs, parsing, write APIs, connection glue) or using small macros for the repeated v1/v2 + sync/async variants to reduce duplication and review surface.
  - mavlink-core/src/lib.rs
