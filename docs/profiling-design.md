# Profiling Support Design

Related issue: #486

## Problem

rust-mavlink currently does not provide a reproducible profiling workflow for
analyzing the runtime characteristics of core MAVLink operations.

Without a common profiling workload, performance-related changes are harder to
evaluate because parsing and serialization costs have to be estimated or tested
using ad-hoc applications.

## Goal

Provide a small and reproducible profiling setup that allows contributors to
analyze rust-mavlink using Linux `perf`.

The initial implementation will focus on the core MAVLink parsing and
serialization paths for MAVLink v1 and v2.

## Non-goals

The first iteration will not:

- introduce performance regression thresholds;
- run profiling automatically in CI;
- change the public rust-mavlink API;
- introduce a complete benchmarking framework;
- optimize existing code as part of the same change.

Performance optimizations can be proposed separately once profiling data is
available.

## Proposed Architecture

Add a dedicated profiling binary containing deterministic workloads.

The workloads should initially cover:

- MAVLink v1 parsing;
- MAVLink v2 parsing;
- MAVLink v1 serialization;
- MAVLink v2 serialization.

Parsing and serialization should be separate workloads so they can be profiled
independently.

The profiling binary should use representative MAVLink messages, including
small and larger payloads where practical.

Example usage:

```bash
cargo run --release --example profiling -- parse-v1
cargo run --release --example profiling -- parse-v2
cargo run --release --example profiling -- serialize-v1
cargo run --release --example profiling -- serialize-v2
```

An iteration count may also be supported:

```bash
cargo run --release --example profiling -- parse-v2 1000000
```

## Workload Design

The profiling workloads should minimize unrelated work inside the measured hot
loop.

## Parsing

The parsing workload should:
1. Construct or serialize a valid MAVLink frame before entering the hot loop.
2. Reuse the encoded frame bytes.
3. Parse the frame repeatedly.
4. Ensure the result is consumed so the compiler cannot optimize the work away.

This keeps frame construction and unrelated allocation outside the section
being profiled.

## Serialization

The serialization workload should:
1. Construct the MAVLink message before entering the hot loop.
2. Serialize the message repeatedly.
3. Reuse buffers where practical.
4. Ensure the serialized output is consumed so the compiler cannot optimize the
operation away.

Allocation should only be part of the workload when it is intentionally being
measured.

## Profiling Workflow

Profiling should use optimized builds while retaining enough debug information
to produce useful call stacks.

The intended Linux workflow is:

```bash
perf record -g <profiling-command>
perf report
```

For example:

```bash
perf record -g cargo run --release --example profiling -- parse-v2 1000000
perf report
```

Flamegraphs may also be generated from the captured profiling data.

The documentation should describe the required commands without requiring
profiling-specific runtime dependencies in rust-mavlink itself.

## Design Requirements

The initial implementation should:
1. Be deterministic.
2. Exercise real rust-mavlink parsing and serialization code paths.
3. Keep parsing and serialization independently profileable.
4. Avoid network and filesystem I/O in the hot loop.
5. Avoid public API changes.
6. Avoid additional runtime dependencies where possible.
7. Be straightforward for contributors to reproduce locally.
8. Support both MAVLink v1 and MAVLink v2 workloads.

## Initial Scope

The first implementation is expected to include:

- a dedicated profiling workload;
- MAVLink v1 parsing;
- MAVLink v2 parsing;
- MAVLink v1 serialization;
- MAVLink v2 serialization;
- configurable iteration count;
- documentation for running the workload with Linux perf.

## Future Work

Possible follow-up work includes:

- additional MAVLink message types;
- different payload sizes;
- transport-level profiling;
- allocation profiling;
- memory profiling;
- Criterion benchmarks;
- automated comparison between revisions;
- CI performance regression monitoring.

These are intentionally outside the scope of the initial implementation.

## Acceptance Criteria

The initial profiling support is complete when:

- the profiling workload builds successfully;
- MAVLink v1 parsing can be profiled independently;
- MAVLink v2 parsing can be profiled independently;
- MAVLink v1 serialization can be profiled independently;
- MAVLink v2 serialization can be profiled independently;
- the workload is deterministic;
- no public API changes are required;
- documentation explains how to run the workload using Linux perf.

