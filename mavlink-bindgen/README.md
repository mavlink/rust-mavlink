# mavlink-bindgen

[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)
[![Crate info](https://img.shields.io/crates/v/mavlink-bindgen.svg)](https://crates.io/crates/mavlink-bindgen)
[![Documentation](https://docs.rs/mavlink-bindgen/badge.svg)](https://docs.rs/mavlink-bindgen)

Library and CLI for generating code for the Rust implementation of the [MAVLink](https://mavlink.io/en) UAV messaging protocol.

`mavlink-bindgen` can be used to create MAVLink bindings for Rust. This is used from `build.rs` in the [mavlink](https://crates.io/crates/mavlink) crate to create bindings from the standard MAVLink dialects in <https://github.com/mavlink/mavlink>.

## Usage

`mavlink-bindgen` can be used as a code generator from `build.rs` as done is the `mavlink` crate for a custom MAVLink dialect or as a CLI tool to generate rust binding from XML dialect definitions. The generated code will depend on the [mavlink-core](https://crates.io/crates/mavlink-core) crate in both use cases. Each dialect generated will be locked behind a feature flag of the same name, that must be enabled when using the generated code.

Furthermore the following feature gates will be present in the generated code: 

- `serde`: enable support for the [serde](https://crates.io/crates/serde) crate
- `arbitrary`: enable support for the [arbitrary](https://crates.io/crates/arbitrary) crate

### CLI

Build the binary using cargo with `cli` feature enabled:

```shell
cd mavlink-bindgen
cargo build --features cli
```

Alternatively you can build and install `mavlink-bindgen` to your locally installed crates:

```shell
cargo install mavlink-bindgen --features cli
```  

To generate code using the resulting binary:

```shell
mavlink-bindgen --format-generated-code message_definitions mavlink_dialects
```

The full command line options are shown below.

```shell
Usage: mavlink-bindgen [OPTIONS] <DEFINITIONS_DIR> <DESTINATION_DIR>

Arguments:
  <DEFINITIONS_DIR>  Path to the directory containing the MAVLink dialect definitions
  <DESTINATION_DIR>  Path to the directory where the code is generated into, must already exist

Options:
      --format-generated-code      format code generated code, requires rustfmt to be installed
      --emit-cargo-build-messages  prints cargo build message indicating when the code has to be rebuild
  -h, --help                       Print help
```

The output dir will contain a `mod.rs` file with each dialect in its own file locked behind a feature flag.

### Library as build dependency

Add to your Cargo.toml:

```toml
mavlink-bindgen = "0.13.1"
```

Add a `build/main.rs` or `build.rs` to your project if it does not already exist. Then add the following to the `main` function to generate the code:

```rs
let out_dir = env::var("OUT_DIR").unwrap();
let result = match mavlink_bindgen::generate(XmlDefinitions::Directory(definitions_dir), out_dir) {
    Ok(r) => r,
    Err(e) => {
        eprintln!("{e}");
        return ExitCode::FAILURE;
    }
};
```

If the generated code should be formatted use

```rs
    mavlink_bindgen::format_generated_code(&result);
```

To tell cargo when to regenerate code from the definitions use:

```rs
    mavlink_bindgen::emit_cargo_build_messages(&result);
```

Finally include the generated code into the `lib.rs` or `main.rs` :

```rs
#![cfg_attr(not(feature = "std"), no_std)]
// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub use mavlink_core::*;
```

Since each dialect is locked behind a feature flag these need to be enabled for the dialects to become available when using the generated code.

This approach is used by the `mavlink` crate see its build script for an example.

## Contributing

### Snapshot tests

This crate uses snapshot tests to guard against subtle changes to the generated code.
The tests generate the formatted MAVLink Rust code from small MAVLink definition files under `tests/definitions/`
and assert that the output hasn't changed. If the output changed the tests will fail. Snapshots can easily be updated
with the new changes if they are valid.

Performing snapshot testing has two main purposes, the first is to avoid having changes to the generated code
go unnoticed. Any change to the generated code will be very obvious with the snapshot tests. The second
purpose is that by checking in the snapshots we have examples of generated code and changes to generated
code are included in the review diff, making reviews easier.

#### Layout

- `tests/definitions/`: Definition files including MAVLink messages. Keeping the amount of messages per definition file
  to a minimum will make it easier to review the output in the snapshots. 
- `tests/e2e_snapshots.rs`: This is where the tests live. It invokes the generator for each XML definition file and snapshots all emitted `.rs` files.
- `tests/snapshots/`: The committed snapshot files created by `insta` are located here. Only the `.snap` files should be commited, `.snap.new` files 
  are created automatically when the tests detect diverging output. They have to be reviewed and either the generator has to be fixed or the 
  snapshots have to be updated.

#### Running tests

Run all tests (including snapshots):

```bash
cargo test
```

On the first run or after codegen changes, new snapshots will be created with a `.snap.new` suffix. Review and accept them if the changes are intended.

#### Reviewing and accepting snapshots

For a better workflow, install `cargo-insta` and use it to collect and review updates interactively. See the Insta quickstart for details: [Insta quickstart](https://insta.rs/docs/quickstart/).

```bash
# Install cargo-insta from crates.io
cargo install cargo-insta
```

After running `cargo test` and snapshot tests have failed:
```bash
# interactively review and accept/reject changes
cargo insta review
```

Alternatively, if you don't want to install `cargo-insta` you can control updates via the `INSTA_UPDATE` environment variable:

```bash
# do not update snapshots (useful for CI-like checks)
INSTA_UPDATE=no cargo test -p mavlink-bindgen

# overwrite all existing snapshots with new results
INSTA_UPDATE=always cargo test -p mavlink-bindgen

# write only new snapshots (existing ones stay unchanged)
INSTA_UPDATE=new cargo test -p mavlink-bindgen
```

#### Adding a new snapshot test

1. Create a minimal XML in `tests/definitions/`, e.g. `foo.xml`. Prefer one message or a tiny set of enums/messages to keep snapshots small and stable.
2. Run the tests: `cargo test -p mavlink-bindgen` (or `cargo insta test`).
3. Review and accept the new snapshots: `cargo insta review`.

The test harness automatically discovers generated `.rs` files for each XML and creates one snapshot per file (e.g. `e2e_snapshots__foo.xml@foo.rs.snap`).

#### Determinism and formatting

The generator strives to emit items in a deterministic order.
If you change code generation intentionally, expect corresponding snapshot updates. Review the diffs carefully for unintended regressions.
