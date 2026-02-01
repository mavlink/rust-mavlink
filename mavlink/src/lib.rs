#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

//! Rust implementation of the MAVLink UAV messaging protocol, with bindings for all dialects.
//! This crate provides message set code generation, packet building, parsing and connection handling for blocking and asynchronous I/O.
//!
//! # Feature flags
//! The `mavlink` crate uses a number of [feature flags] to reduce the amount of compiled code by making certain functions and MAVLink message sets (dialects) optional.
//! These feature flags are available to control the provided functionalities:
//!
//! - `std`: Enables the usage of `std` in `mavlink`, enabled by default, this can be disabled for embedded applications.
//! - `direct-serial`: Enable serial MAVLink connections, enabled by default.
//! - `udp`: Enables UDP based MAVLink connections, enabled by default.
//! - `tcp`: Enables TCP based MAVLink connections, enabled by default.
//! - `signing`: Enable support for [MAVLink 2 message signing]
//! - `embedded`: Enables embedded support using the [embedded-io] crate, incompatible with `embedded-hal-02` and `tokio-1`.
//! - `embedded-hal-02`: Enables embedded support using version 0.2 of the [embedded-hal] crate, incompatible with `embedded`.
//! - `tokio-1`: Enable support for asynchronous I/O using [tokio], incompatible with `embedded`.
//! - `serde`: Enables [serde] support in generated message sets, enabled by default.
//! - `format-generated-code`: Generated MAVLink message set code will be formatted, requires `rustfmt` to be installed, enabled by default.
//! - `emit-extensions`: Generated MAVLink message set code will include [MAVLink 2 message extensions].
//! - `arbitrary`: Enable support for the [arbitrary] crate.
//! - `ts`: Enable support for [ts-rs] typescript generation.
//!
//! Either `std`, `embedded` or `embedded-hal-02` must be enabled.
//!
//! Each MAVlink message set (dialect) can be enabled using its feature flag. The following message set feature flags are available:
//! - `ardupilotmega`, enabled by default
//! - `common`, enabled by default
//! - `all`, this includes all other sets in the same message set
//! - `asluav`
//! - `avssuas`
//! - `cubepilot`
//! - `csairlink`
//! - `development`
//! - `icarous`
//! - `loweheiser`
//! - `marsh`
//! - `matrixpilot`
//! - `minimal`
//! - `paparazzi`
//! - `python_array_test`
//! - `slugs`
//! - `standard`
//! - `stemstudios`
//! - `storm32`
//! - `test`
//! - `ualberta`
//! - `uavionix`
//!
//! The `all-dialects` feature enables all message sets except `all`.
//!
//! [feature flags]: https://doc.rust-lang.org/cargo/reference/manifest.html#the-features-section
//! [MAVLink 2 message signing]: https://mavlink.io/en/guide/message_signing.html
//! [MAVLink 2 message extensions]: https://mavlink.io/en/guide/define_xml_element.html#message_extensions
//! [embedded-io]: https://crates.io/crates/embedded-io
//! [embedded-hal]: https://crates.io/crates/embedded-hal
//! [tokio]: https://crates.io/crates/tokio
//! [serde]: https://crates.io/crates/serde
//! [arbitrary]: https://crates.io/crates/arbitrary
//! [ts-rs]: https://crates.io/crates/ts-rs

// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub use mavlink_core::*;

#[cfg(feature = "emit-extensions")]
#[allow(unused_imports)]
pub(crate) use mavlink_core::utils::RustDefault;

#[cfg(feature = "serde")]
#[allow(unused_imports)]
pub(crate) use mavlink_core::utils::nulstr;
