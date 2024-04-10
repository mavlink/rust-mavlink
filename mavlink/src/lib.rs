#![cfg_attr(not(feature = "std"), no_std)]
// include generate definitions
include!(concat!(env!("OUT_DIR"), "/mod.rs"));

pub use mavlink_core::*;

#[cfg(feature = "ardu_json")]
mod ardu_json;
#[cfg(feature = "ardu_json")]
pub use ardu_json::{mavlink_to_json_value,
    mavlink_to_json_str,
    json_value_to_mavlink,
    json_str_to_mavlink,
};