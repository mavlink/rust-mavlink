# rust-mavlink

[![Build status](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml/badge.svg)](https://github.com/mavlink/rust-mavlink/actions/workflows/test.yml)

[![Crate info](https://img.shields.io/crates/v/mavlink-bindgen.svg)](https://crates.io/crates/mavlink-bindgen)
[![Documentation](https://docs.rs/mavlink-bindgen/badge.svg)](https://docs.rs/mavlink-bindgen)

Library and CLI for code generator of the Rust implementation of the [MAVLink](https://mavlink.io/en) UAV messaging protocol.

`mavlink-bindgen` can be used to create MAVLink bindings for Rust. This is used from `build.rs` in the [mavlink](https://crates.io/crates/mavlink) crate to create bindings from the standard MAVLink dialects in https://github.com/mavlink/mavlink. 

## Usage

`mavlink-bindgen` can be used as a code generator from `build.rs` as done is the `mavlink` crate for a custom MAVLink dialect or as a CLI tool to generate rust binding from XML dialect definitions. The generated code will depend on the [mavlink-core](https://crates.io/crates/mavlink-core) crate in both use cases.

### CLI

Build using cargo with `cli` feature enabled:

```shell
cargo build --features cli
```

Alternatively you can build and install `mavlink-bindgen` to you locally installed crates:

```shell
cargo install mavlink-bindgen --features cli
```  

Generate code using the resulting binary:

```shell
mavlink-bindgen --format-generated-code message_definitions mavlink_dialects
```

The full options are shown below.

```shell
Usage: mavlink-bindgen [OPTIONS] <DEFINITIONS_DIR> <DESTINATION_DIR>

Arguments:
  <DEFINITIONS_DIR>  Path to the directory containing the MAVLink dialect definitions
  <DESTINATION_DIR>  Path to the directory where the code is generated into

Options:
      --format-generated-code      format code generated code
      --emit-cargo-build-messages  prints cargo build message indicating when the code has to be rebuild
  -h, --help                       Print help
```

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

This approach is used by the mavlink crate see its build script for an example.
