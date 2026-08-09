# MAVLink Profiling

This example provides deterministic workloads for profiling core MAVLink
parsing and serialization paths.

## Workloads

- `parse-v1`
- `parse-v2`
- `serialize-v1`
- `serialize-v2`

The optional second argument specifies the iteration count. The default is
1,000,000.

Examples:

```bash
cargo run --release -- parse-v1 1000000
cargo run --release -- parse-v2 1000000
cargo run --release -- serialize-v1 1000000
cargo run --release -- serialize-v2 1000000
```

## Linux perf

Build the profiling binary:

```bash
cargo build --release
```

Then profile the binary directly:

```bash
perf record -g ./target/release/mavlink-profiling parse-v2 1000000
perf report
```

Running the built binary directly avoids including Cargo startup work in the
profile.

The release profile retains debug information to provide useful symbols in
profiling output.

## Scope

The initial workloads focus on MAVLink v1 and v2 parsing and serialization
without network or filesystem I/O in the measured path.
