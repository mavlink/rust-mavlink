# mavlink-bindgen

[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)
[![Crate info](https://img.shields.io/crates/v/mavlink-bindgen.svg)](https://crates.io/crates/mavlink-bindgen)
[![Documentation](https://docs.rs/mavlink-bindgen/badge.svg)](https://docs.rs/mavlink-bindgen)

Library and CLI for generating code for the Rust implementation of the [MAVLink](https://mavlink.io/en) UAV messaging protocol.

`mavlink-bindgen` can be used to create MAVLink bindings for Rust. This is used from `build.rs` in the [mavlink](https://crates.io/crates/mavlink) crate to create bindings from the standard MAVLink dialects in <https://github.com/mavlink/mavlink>.

## Usage

`mavlink-bindgen` can be used as a code generator from `build.rs` as done is the `mavlink` crate for a custom MAVLink dialect or as a CLI tool to generate rust binding from XML dialect definitions. The generated code will depend on the [mavlink-core](https://crates.io/crates/mavlink-core) crate in both use cases. Each dialect generated will be locked behind a feature flag of the same name, that must be enabled when using the generated code.

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
      --format-generated-code      format code generated code
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
let result = match mavlink_bindgen::generate(definitions_dir, out_dir) {
    Ok(r) => r,
    Err(e) => {
        eprintln!("{e}");
        return ExitCode::FAILURE;
    }
};
```

If the generated code should be formated use

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
