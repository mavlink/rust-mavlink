use crc_any::CRCu16;
use std::cmp::Ordering;
use std::collections::btree_map::Entry;
use std::collections::{BTreeMap, HashSet};
use std::default::Default;
use std::fmt::Display;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::str::FromStr;
use std::sync::LazyLock;

use regex::Regex;

use quick_xml::{events::Event, Reader};

use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::error::BindGenError;
use crate::util;

static URL_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(concat!(
        r"(https?://",                                               // url scheme
        r"([-a-zA-Z0-9@:%._\+~#=]{2,256}\.)+",                       // one or more subdomains
        r"[a-zA-Z]{2,63}",                                           // root domain
        r"\b([-a-zA-Z0-9@:%_\+.~#?&/=]*[-a-zA-Z0-9@:%_\+~#?&/=])?)", // optional query or url fragments
    ))
    .expect("failed to build regex")
});

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavProfile {
    pub messages: BTreeMap<String, MavMessage>,
    pub enums: BTreeMap<String, MavEnum>,
    pub version: Option<u8>,
    pub dialect: Option<u8>,
}

impl MavProfile {
    fn add_message(&mut self, message: &MavMessage) {
        match self.messages.entry(message.name.clone()) {
            Entry::Occupied(entry) => {
                assert!(
                    entry.get() == message,
                    "Message '{}' defined twice but definitions are different",
                    message.name
                );
            }
            Entry::Vacant(entry) => {
                entry.insert(message.clone());
            }
        }
    }

    fn add_enum(&mut self, enm: &MavEnum) {
        match self.enums.entry(enm.name.clone()) {
            Entry::Occupied(entry) => {
                entry.into_mut().try_combine(enm);
            }
            Entry::Vacant(entry) => {
                entry.insert(enm.clone());
            }
        }
    }

    /// Go over all fields in the messages, and if you encounter an enum,
    /// which is a bitmask, set the bitmask size based on field size
    fn update_enums(mut self) -> Self {
        for msg in self.messages.values_mut() {
            for field in &mut msg.fields {
                if let Some(enum_name) = &field.enumtype {
                    // find the corresponding enum
                    if let Some(enm) = self.enums.get_mut(enum_name) {
                        // Handle legacy definition where bitmask is defined as display="bitmask"
                        if field.display == Some("bitmask".to_string()) {
                            enm.bitmask = true;
                        }

                        // it is a bitmask
                        if enm.bitmask {
                            enm.primitive = Some(field.mavtype.rust_primitive_type());

                            // check if all enum values can be stored in the fields
                            for entry in &enm.entries {
                                assert!(
                                    entry.value.unwrap_or_default() <= field.mavtype.max_int_value(),
                                    "bitflag enum field {} of {} must be able to fit all possible values for {}",
                                    field.name,
                                    msg.name,
                                    enum_name,
                                );
                            }

                            // Fix fields in backwards manner
                            if field.display.is_none() {
                                field.display = Some("bitmask".to_string());
                            }
                        }
                    }
                }
            }
        }
        self
    }

    //TODO verify this is no longer necessary since we're supporting both mavlink1 and mavlink2
    //    ///If we are not using Mavlink v2, remove messages with id's > 254
    //    fn update_messages(mut self) -> Self {
    //        //println!("Updating messages");
    //        let msgs = self.messages.into_iter().filter(
    //            |x| x.id <= 254).collect::<Vec<MavMessage>>();
    //        self.messages = msgs;
    //        self
    //    }

    /// Simple header comment
    #[inline(always)]
    fn emit_comments(&self, dialect_name: &str) -> TokenStream {
        let message = format!("MAVLink {dialect_name} dialect.");
        quote!(
            #![doc = #message]
            #![doc = ""]
            #![doc = "This file was automatically generated, do not edit."]
        )
    }

    /// Emit rust messages
    #[inline(always)]
    fn emit_msgs(&self) -> Vec<TokenStream> {
        self.messages
            .values()
            .map(|d| d.emit_rust(self.version.is_some()))
            .collect()
    }

    /// Emit rust enums
    #[inline(always)]
    fn emit_enums(&self) -> Vec<TokenStream> {
        self.enums.values().map(|d| d.emit_rust()).collect()
    }

    #[inline(always)]
    fn emit_deprecations(&self) -> Vec<TokenStream> {
        self.messages
            .values()
            .map(|msg| {
                msg.deprecated
                    .as_ref()
                    .map(|d| d.emit_tokens())
                    .unwrap_or_default()
            })
            .collect()
    }

    /// Get list of original message names
    #[inline(always)]
    fn emit_enum_names(&self) -> Vec<TokenStream> {
        self.messages
            .values()
            .map(|msg| {
                let name = format_ident!("{}", msg.name);
                quote!(#name)
            })
            .collect()
    }

    /// Emit message names with "_DATA" at the end
    #[inline(always)]
    fn emit_struct_names(&self) -> Vec<TokenStream> {
        self.messages
            .values()
            .map(|msg| msg.emit_struct_name())
            .collect()
    }

    fn emit_rust(&self, dialect_name: &str) -> TokenStream {
        //TODO verify that id_width of u8 is OK even in mavlink v1
        let id_width = format_ident!("u32");

        let comment = self.emit_comments(dialect_name);
        let mav_minor_version = self.emit_minor_version();
        let mav_dialect_number = self.emit_dialect_number();
        let msgs = self.emit_msgs();
        let deprecations = self.emit_deprecations();
        let enum_names = self.emit_enum_names();
        let struct_names = self.emit_struct_names();
        let enums = self.emit_enums();

        let variant_docs = self.emit_variant_description();

        let mav_message =
            self.emit_mav_message(&variant_docs, &deprecations, &enum_names, &struct_names);
        let mav_message_all_ids = self.emit_mav_message_all_ids();
        let mav_message_all_messages = self.emit_mav_message_all_messages();
        let mav_message_parse = self.emit_mav_message_parse(&enum_names, &struct_names);
        let mav_message_crc = self.emit_mav_message_crc(&id_width, &struct_names);
        let mav_message_name = self.emit_mav_message_name(&enum_names, &struct_names);
        let mav_message_id = self.emit_mav_message_id(&enum_names, &struct_names);
        let mav_message_id_from_name = self.emit_mav_message_id_from_name(&struct_names);
        let mav_message_default_from_id =
            self.emit_mav_message_default_from_id(&enum_names, &struct_names);
        let mav_message_random_from_id =
            self.emit_mav_message_random_from_id(&enum_names, &struct_names);
        let mav_message_serialize = self.emit_mav_message_serialize(&enum_names);
        let mav_message_target_system_id = self.emit_mav_message_target_system_id();
        let mav_message_target_component_id = self.emit_mav_message_target_component_id();

        quote! {
            #comment
            #![allow(deprecated)]
            #![allow(clippy::match_single_binding)]
            #[allow(unused_imports)]
            use num_derive::{FromPrimitive, ToPrimitive};
            #[allow(unused_imports)]
            use num_traits::{FromPrimitive, ToPrimitive};
            #[allow(unused_imports)]
            use bitflags::{bitflags, Flags};
            #[allow(unused_imports)]
            use mavlink_core::{MavlinkVersion, Message, MessageData, bytes::Bytes, bytes_mut::BytesMut, types::CharArray};

            #[cfg(feature = "serde")]
            use serde::{Serialize, Deserialize};

            #[cfg(feature = "arbitrary")]
            use arbitrary::Arbitrary;

            #[cfg(feature = "ts")]
            use ts_rs::TS;

            #mav_minor_version
            #mav_dialect_number

            #(#enums)*

            #(#msgs)*

            #[derive(Clone, PartialEq, Debug)]
            #mav_message

            impl MavMessage {
                #mav_message_all_ids
                #mav_message_all_messages
            }

            impl Message for MavMessage {
                #mav_message_parse
                #mav_message_name
                #mav_message_id
                #mav_message_id_from_name
                #mav_message_default_from_id
                #mav_message_random_from_id
                #mav_message_serialize
                #mav_message_crc
                #mav_message_target_system_id
                #mav_message_target_component_id
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message(
        &self,
        docs: &[TokenStream],
        deprecations: &[TokenStream],
        enums: &[TokenStream],
        structs: &[TokenStream],
    ) -> TokenStream {
        quote! {
            #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
            #[cfg_attr(feature = "serde", serde(tag = "type"))]
            #[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
            #[cfg_attr(feature = "ts", derive(TS))]
            #[cfg_attr(feature = "ts", ts(export))]
            #[repr(u32)]
            pub enum MavMessage {
                #(#docs #deprecations #enums(#structs),)*
            }
        }
    }

    fn emit_variant_description(&self) -> Vec<TokenStream> {
        self.messages
            .values()
            .map(|msg| {
                let mut ts = TokenStream::new();

                if let Some(doc) = msg.description.as_ref() {
                    let doc = format!("{doc}{}", if doc.ends_with('.') { "" } else { "." });
                    let doc = URL_REGEX.replace_all(&doc, "<$1>");
                    ts.extend(quote!(#[doc = #doc]));

                    // Leave a blank line before the message ID for readability.
                    ts.extend(quote!(#[doc = ""]));
                }

                let id = format!("ID: {}", msg.id);
                ts.extend(quote!(#[doc = #id]));

                ts
            })
            .collect()
    }

    #[inline(always)]
    fn emit_mav_message_all_ids(&self) -> TokenStream {
        let mut message_ids = self.messages.values().map(|m| m.id).collect::<Vec<u32>>();
        message_ids.sort();

        quote!(
            pub const fn all_ids() -> &'static [u32] {
                &[#(#message_ids),*]
            }
        )
    }

    #[inline(always)]
    fn emit_minor_version(&self) -> TokenStream {
        if let Some(version) = self.version {
            quote! (pub const MINOR_MAVLINK_VERSION: u8 = #version;)
        } else {
            TokenStream::default()
        }
    }

    #[inline(always)]
    fn emit_dialect_number(&self) -> TokenStream {
        if let Some(dialect) = self.dialect {
            quote! (pub const DIALECT_NUMBER: u8 = #dialect;)
        } else {
            TokenStream::default()
        }
    }

    #[inline(always)]
    fn emit_mav_message_parse(
        &self,
        enums: &[TokenStream],
        structs: &[TokenStream],
    ) -> TokenStream {
        let id_width = format_ident!("u32");

        quote! {
            fn parse(version: MavlinkVersion, id: #id_width, payload: &[u8]) -> Result<Self, ::mavlink_core::error::ParserError> {
                match id {
                    #(#structs::ID => #structs::deser(version, payload).map(Self::#enums),)*
                    _ => {
                        Err(::mavlink_core::error::ParserError::UnknownMessage { id })
                    },
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_crc(&self, id_width: &Ident, structs: &[TokenStream]) -> TokenStream {
        quote! {
            fn extra_crc(id: #id_width) -> u8 {
                match id {
                    #(#structs::ID => #structs::EXTRA_CRC,)*
                    _ => {
                        0
                    },
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_name(&self, enums: &[TokenStream], structs: &[TokenStream]) -> TokenStream {
        quote! {
            fn message_name(&self) -> &'static str {
                match self {
                    #(Self::#enums(..) => #structs::NAME,)*
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_id(&self, enums: &[TokenStream], structs: &[TokenStream]) -> TokenStream {
        let id_width = format_ident!("u32");
        quote! {
            fn message_id(&self) -> #id_width {
                match self {
                    #(Self::#enums(..) => #structs::ID,)*
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_id_from_name(&self, structs: &[TokenStream]) -> TokenStream {
        quote! {
            fn message_id_from_name(name: &str) -> Option<u32> {
                match name {
                    #(#structs::NAME => Some(#structs::ID),)*
                    _ => {
                        None
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_default_from_id(
        &self,
        enums: &[TokenStream],
        structs: &[TokenStream],
    ) -> TokenStream {
        quote! {
            fn default_message_from_id(id: u32) -> Option<Self> {
                match id {
                    #(#structs::ID => Some(Self::#enums(#structs::default())),)*
                    _ => {
                        None
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_random_from_id(
        &self,
        enums: &[TokenStream],
        structs: &[TokenStream],
    ) -> TokenStream {
        quote! {
            #[cfg(feature = "arbitrary")]
            fn random_message_from_id<R: rand::RngCore>(id: u32, rng: &mut R) -> Option<Self> {
                match id {
                    #(#structs::ID => Some(Self::#enums(#structs::random(rng))),)*
                    _ => None,
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_serialize(&self, enums: &Vec<TokenStream>) -> TokenStream {
        quote! {
            fn ser(&self, version: MavlinkVersion, bytes: &mut [u8]) -> usize {
                match self {
                    #(Self::#enums(body) => body.ser(version, bytes),)*
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_target_system_id(&self) -> TokenStream {
        let arms: Vec<TokenStream> = self
            .messages
            .values()
            .filter(|msg| msg.fields.iter().any(|f| f.name == "target_system"))
            .map(|msg| {
                let variant = format_ident!("{}", msg.name);
                quote!(Self::#variant(inner) => Some(inner.target_system),)
            })
            .collect();

        quote! {
            fn target_system_id(&self) -> Option<u8> {
                match self {
                    #(#arms)*
                    _ => None,
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_target_component_id(&self) -> TokenStream {
        let arms: Vec<TokenStream> = self
            .messages
            .values()
            .filter(|msg| msg.fields.iter().any(|f| f.name == "target_component"))
            .map(|msg| {
                let variant = format_ident!("{}", msg.name);
                quote!(Self::#variant(inner) => Some(inner.target_component),)
            })
            .collect();

        quote! {
            fn target_component_id(&self) -> Option<u8> {
                match self {
                    #(#arms)*
                    _ => None,
                }
            }
        }
    }

    #[inline(always)]
    fn emit_mav_message_all_messages(&self) -> TokenStream {
        let mut entries = self
            .messages
            .values()
            .map(|msg| (msg.id, msg.emit_struct_name()))
            .collect::<Vec<_>>();

        entries.sort_by_key(|(id, _)| *id);

        let pairs = entries
            .into_iter()
            .map(|(_, struct_name)| quote!((#struct_name::NAME, #struct_name::ID)))
            .collect::<Vec<_>>();

        quote! {
            pub const fn all_messages() -> &'static [(&'static str, u32)] {
                &[#(#pairs),*]
            }
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavEnum {
    pub name: String,
    pub description: Option<String>,
    pub entries: Vec<MavEnumEntry>,
    /// If contains Some, the string represents the primitive type (size) for bitflags.
    /// If no fields use this enum, the bitmask is true, but primitive is None. In this case
    /// regular enum is generated as primitive is unknown.
    pub primitive: Option<String>,
    pub bitmask: bool,
    pub deprecated: Option<MavDeprecation>,
}

impl MavEnum {
    /// Returns true when this enum will be emitted as a `bitflags` struct.
    fn is_generated_as_bitflags(&self) -> bool {
        self.primitive.is_some()
    }

    fn try_combine(&mut self, enm: &Self) {
        if self.name == enm.name {
            for enum_entry in &enm.entries {
                let found_entry = self.entries.iter().find(|elem| {
                    elem.name == enum_entry.name && elem.value.unwrap() == enum_entry.value.unwrap()
                });
                match found_entry {
                    Some(entry) => panic!("Enum entry {} already exists", entry.name),
                    None => self.entries.push(enum_entry.clone()),
                }
            }
        }
    }

    fn emit_defs(&self) -> Vec<TokenStream> {
        let mut cnt = 0u64;
        self.entries
            .iter()
            .map(|enum_entry| {
                let name = format_ident!("{}", enum_entry.name.clone());
                let value;

                let deprecation = enum_entry.emit_deprecation();

                let description = if let Some(description) = enum_entry.description.as_ref() {
                    let description = URL_REGEX.replace_all(description, "<$1>");
                    quote!(#[doc = #description])
                } else {
                    quote!()
                };

                let params_doc = enum_entry.emit_params();

                if let Some(tmp_value) = enum_entry.value {
                    cnt = cnt.max(tmp_value);
                    let tmp = TokenStream::from_str(&tmp_value.to_string()).unwrap();
                    value = quote!(#tmp);
                } else {
                    cnt += 1;
                    value = quote!(#cnt);
                }

                if self.is_generated_as_bitflags() {
                    quote! {
                        #deprecation
                        #description
                        #params_doc
                        const #name = #value;
                    }
                } else {
                    quote! {
                        #deprecation
                        #description
                        #params_doc
                        #name = #value,
                    }
                }
            })
            .collect()
    }

    #[inline(always)]
    fn emit_name(&self) -> TokenStream {
        let name = format_ident!("{}", self.name);
        quote!(#name)
    }

    #[inline(always)]
    fn emit_const_default(&self) -> TokenStream {
        let default = format_ident!("{}", self.entries[0].name);
        quote!(pub const DEFAULT: Self = Self::#default;)
    }

    #[inline(always)]
    fn emit_deprecation(&self) -> TokenStream {
        self.deprecated
            .as_ref()
            .map(|d| d.emit_tokens())
            .unwrap_or_default()
    }

    fn emit_rust(&self) -> TokenStream {
        let defs = self.emit_defs();
        let enum_name = self.emit_name();
        let const_default = self.emit_const_default();

        let deprecated = self.emit_deprecation();

        let description = if let Some(description) = self.description.as_ref() {
            let desc = URL_REGEX.replace_all(description, "<$1>");
            quote!(#[doc = #desc])
        } else {
            quote!()
        };

        let mav_bool_impl = if self.name == "MavBool"
            && self
                .entries
                .iter()
                .any(|entry| entry.name == "MAV_BOOL_TRUE")
        {
            if self.is_generated_as_bitflags() {
                quote!(
                    pub fn as_bool(&self) -> bool {
                        self.contains(Self::MAV_BOOL_TRUE)
                    }
                )
            } else {
                quote!(
                    pub fn as_bool(&self) -> bool {
                        *self == Self::MAV_BOOL_TRUE
                    }
                )
            }
        } else {
            quote!()
        };

        let enum_def;
        if let Some(primitive) = self.primitive.clone() {
            let primitive = format_ident!("{}", primitive);
            enum_def = quote! {
                bitflags!{
                    #[cfg_attr(feature = "ts", derive(TS))]
                    #[cfg_attr(feature = "ts", ts(export, type = "number"))]
                    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
                    #[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
                    #[derive(Debug, Copy, Clone, PartialEq)]
                    #deprecated
                    #description
                    pub struct #enum_name: #primitive {
                        #(#defs)*
                    }
                }
            };
        } else {
            enum_def = quote! {
                #[cfg_attr(feature = "ts", derive(TS))]
                #[cfg_attr(feature = "ts", ts(export))]
                #[derive(Debug, Copy, Clone, PartialEq, FromPrimitive, ToPrimitive)]
                #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
                #[cfg_attr(feature = "serde", serde(tag = "type"))]
                #[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
                #[repr(u32)]
                #deprecated
                #description
                pub enum #enum_name {
                    #(#defs)*
                }
            };
        }

        quote! {
            #enum_def

            impl #enum_name {
                #const_default
                #mav_bool_impl
            }

            impl Default for #enum_name {
                fn default() -> Self {
                    Self::DEFAULT
                }
            }
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavEnumEntry {
    pub value: Option<u64>,
    pub name: String,
    pub description: Option<String>,
    pub params: Option<Vec<MavParam>>,
    pub deprecated: Option<MavDeprecation>,
}

impl MavEnumEntry {
    #[inline(always)]
    fn emit_deprecation(&self) -> TokenStream {
        self.deprecated
            .as_ref()
            .map(|d| d.emit_tokens())
            .unwrap_or_default()
    }

    #[inline(always)]
    fn emit_params(&self) -> TokenStream {
        let Some(params) = &self.params else {
            return quote!();
        };
        let any_value_range = params.iter().any(|p| {
            p.min_value.is_some()
                || p.max_value.is_some()
                || p.increment.is_some()
                || p.enum_used.is_some()
                || (p.reserved && p.default.is_some())
        });
        let any_units = params.iter().any(|p| p.units.is_some());
        let lines = params
            .iter()
            .map(|param| param.emit_doc_row(any_value_range, any_units));
        let mut table_header = "| Parameter | Description |".to_string();
        let mut table_hl = "| --------- | ----------- |".to_string();
        if any_value_range {
            table_header += " Values |";
            table_hl += " ------ |";
        }
        if any_units {
            table_header += " Units |";
            table_hl += " ----- |";
        }
        quote! {
            #[doc = ""]
            #[doc = "# Parameters"]
            #[doc = ""]
            #[doc = #table_header]
            #[doc = #table_hl]
            #(#lines)*
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavParam {
    pub index: usize,
    pub description: Option<String>,
    pub label: Option<String>,
    pub units: Option<String>,
    pub enum_used: Option<String>,
    pub increment: Option<f32>,
    pub min_value: Option<f32>,
    pub max_value: Option<f32>,
    pub reserved: bool,
    pub default: Option<f32>,
}

fn format_number_range(min: Option<f32>, max: Option<f32>, inc: Option<f32>) -> String {
    match (min, max, inc) {
        (Some(min), Some(max), Some(inc)) => {
            if min + inc == max {
                format!("{min}, {max}")
            } else if min + 2. * inc == max {
                format!("{}, {}, {}", min, min + inc, max)
            } else {
                format!("{}, {}, .. , {}", min, min + inc, max)
            }
        }
        (Some(min), Some(max), None) => format!("{min} .. {max}"),
        (Some(min), None, Some(inc)) => format!("{}, {}, ..", min, min + inc),
        (None, Some(max), Some(inc)) => format!(".., {}, {}", max - inc, max),
        (Some(min), None, None) => format!("&ge; {min}"),
        (None, Some(max), None) => format!("&le; {max}"),
        (None, None, Some(inc)) => format!("Multiples of {inc}"),
        (None, None, None) => String::new(),
    }
}

impl MavParam {
    fn format_valid_values(&self) -> String {
        if let (true, Some(default)) = (self.reserved, self.default) {
            format!("Reserved (use {default})")
        } else if let Some(enum_used) = &self.enum_used {
            format!("[`{enum_used}`]")
        } else {
            format_number_range(self.min_value, self.max_value, self.increment)
        }
    }

    fn emit_doc_row(&self, value_range_col: bool, units_col: bool) -> TokenStream {
        let label = if let Some(label) = &self.label {
            format!("{} ({})", self.index, label)
        } else {
            format!("{}", self.index)
        };
        let mut line = format!(
            "| {label:10}| {:12}|",
            self.description.as_deref().unwrap_or_default()
        );
        if value_range_col {
            let range = self.format_valid_values();
            line += &format!(" {range} |");
        }
        if units_col {
            let units = self.units.clone().unwrap_or_default();
            line += &format!(" {units} |");
        }
        quote! {#[doc = #line]}
    }
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavMessage {
    pub id: u32,
    pub name: String,
    pub description: Option<String>,
    pub fields: Vec<MavField>,
    pub deprecated: Option<MavDeprecation>,
}

impl MavMessage {
    /// Return Token of "MESSAGE_NAME_DATA
    /// for mavlink struct data
    fn emit_struct_name(&self) -> TokenStream {
        let name = format_ident!("{}", format!("{}_DATA", self.name));
        quote!(#name)
    }

    #[inline(always)]
    fn emit_name_types(&self) -> (Vec<TokenStream>, usize) {
        let mut encoded_payload_len: usize = 0;
        let field_toks = self
            .fields
            .iter()
            .map(|field| {
                let nametype = field.emit_name_type();
                encoded_payload_len += field.mavtype.len();

                let description = field.emit_description();

                // From MAVLink specification:
                // If sent by an implementation that doesn't have the extensions fields
                // then the recipient will see zero values for the extensions fields.
                let serde_default = if field.is_extension {
                    if field.enumtype.is_some() {
                        quote!(#[cfg_attr(feature = "serde", serde(default))])
                    } else {
                        quote!(#[cfg_attr(feature = "serde", serde(default = "crate::RustDefault::rust_default"))])
                    }
                } else {
                    quote!()
                };

                let serde_with_attr = if matches!(field.mavtype, MavType::Array(_, _)) {
                    quote!(
                        #[cfg_attr(feature = "serde", serde(with = "serde_arrays"))]
                        #[cfg_attr(feature = "ts", ts(type = "Array<number>"))]
                    )
                } else if matches!(field.mavtype, MavType::CharArray(_)) {
                    quote!(
                        #[cfg_attr(feature = "ts", ts(type = "string"))]
                    )
                } else {
                    quote!()
                };

                quote! {
                    #description
                    #serde_default
                    #serde_with_attr
                    #nametype
                }
            })
            .collect::<Vec<TokenStream>>();
        (field_toks, encoded_payload_len)
    }

    /// Generate description for the given message
    #[inline(always)]
    fn emit_description(&self) -> TokenStream {
        let mut ts = TokenStream::new();
        if let Some(doc) = self.description.as_ref() {
            let doc = format!("{doc}{}", if doc.ends_with('.') { "" } else { "." });
            // create hyperlinks
            let doc = URL_REGEX.replace_all(&doc, "<$1>");
            ts.extend(quote!(#[doc = #doc]));
            // Leave a blank line before the message ID for readability.
            ts.extend(quote!(#[doc = ""]));
        }
        let id = format!("ID: {}", self.id);
        ts.extend(quote!(#[doc = #id]));
        ts
    }

    #[inline(always)]
    fn emit_serialize_vars(&self) -> TokenStream {
        let (base_fields, ext_fields): (Vec<_>, Vec<_>) =
            self.fields.iter().partition(|f| !f.is_extension);
        let ser_vars = base_fields.iter().map(|f| f.rust_writer());
        let ser_ext_vars = ext_fields.iter().map(|f| f.rust_writer());
        quote! {
            let mut __tmp = BytesMut::new(bytes);

            // TODO: these lints are produced on a couple of cubepilot messages
            // because they are generated as empty structs with no fields and
            // therefore Self::ENCODED_LEN is 0. This itself is a bug because
            // cubepilot.xml has unclosed tags in fields, which the parser has
            // bad time handling. It should probably be fixed in both the parser
            // and mavlink message definitions. However, until it's done, let's
            // skip the lints.
            #[allow(clippy::absurd_extreme_comparisons)]
            #[allow(unused_comparisons)]
            if __tmp.remaining() < Self::ENCODED_LEN {
                panic!(
                    "buffer is too small (need {} bytes, but got {})",
                    Self::ENCODED_LEN,
                    __tmp.remaining(),
                )
            }

            #(#ser_vars)*
            if matches!(version, MavlinkVersion::V2) {
                #(#ser_ext_vars)*
                let len = __tmp.len();
                ::mavlink_core::utils::remove_trailing_zeroes(&bytes[..len])
            } else {
                __tmp.len()
            }
        }
    }

    #[inline(always)]
    fn emit_deserialize_vars(&self) -> TokenStream {
        let deser_vars = self
            .fields
            .iter()
            .map(|f| f.rust_reader())
            .collect::<Vec<TokenStream>>();

        if deser_vars.is_empty() {
            // struct has no fields
            quote! {
                Ok(Self::default())
            }
        } else {
            quote! {
                let avail_len = __input.len();

                let mut payload_buf  = [0; Self::ENCODED_LEN];
                let mut buf = if avail_len < Self::ENCODED_LEN {
                    //copy available bytes into an oversized buffer filled with zeros
                    payload_buf[0..avail_len].copy_from_slice(__input);
                    Bytes::new(&payload_buf)
                } else {
                    // fast zero copy
                    Bytes::new(__input)
                };

                let mut __struct = Self::default();
                #(#deser_vars)*
                Ok(__struct)
            }
        }
    }

    #[inline(always)]
    fn emit_default_impl(&self) -> TokenStream {
        let msg_name = self.emit_struct_name();
        quote! {
            impl Default for #msg_name {
                fn default() -> Self {
                    Self::DEFAULT.clone()
                }
            }
        }
    }

    #[inline(always)]
    fn emit_deprecation(&self) -> TokenStream {
        self.deprecated
            .as_ref()
            .map(|d| d.emit_tokens())
            .unwrap_or_default()
    }

    #[inline(always)]
    fn emit_const_default(&self, dialect_has_version: bool) -> TokenStream {
        let initializers = self
            .fields
            .iter()
            .map(|field| field.emit_default_initializer(dialect_has_version));
        quote!(pub const DEFAULT: Self = Self { #(#initializers)* };)
    }

    fn emit_rust(&self, dialect_has_version: bool) -> TokenStream {
        let msg_name = self.emit_struct_name();
        let id = self.id;
        let name = self.name.clone();
        let extra_crc = extra_crc(self);
        let (name_types, payload_encoded_len) = self.emit_name_types();
        assert!(
            payload_encoded_len <= 255,
            "maximum payload length is 255 bytes"
        );

        let deser_vars = self.emit_deserialize_vars();
        let serialize_vars = self.emit_serialize_vars();
        let const_default = self.emit_const_default(dialect_has_version);
        let default_impl = self.emit_default_impl();

        let deprecation = self.emit_deprecation();

        let description = self.emit_description();

        quote! {
            #deprecation
            #description
            #[derive(Debug, Clone, PartialEq)]
            #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
            #[cfg_attr(feature = "arbitrary", derive(Arbitrary))]
            #[cfg_attr(feature = "ts", derive(TS))]
            #[cfg_attr(feature = "ts", ts(export))]
            pub struct #msg_name {
                #(#name_types)*
            }

            impl #msg_name {
                pub const ENCODED_LEN: usize = #payload_encoded_len;
                #const_default

                #[cfg(feature = "arbitrary")]
                pub fn random<R: rand::RngCore>(rng: &mut R) -> Self {
                    use arbitrary::{Unstructured, Arbitrary};
                    let mut buf = [0u8; 1024];
                    rng.fill_bytes(&mut buf);
                    let mut unstructured = Unstructured::new(&buf);
                    Self::arbitrary(&mut unstructured).unwrap_or_default()
                }
            }

            #default_impl

            impl MessageData for #msg_name {
                type Message = MavMessage;

                const ID: u32 = #id;
                const NAME: &'static str = #name;
                const EXTRA_CRC: u8 = #extra_crc;
                const ENCODED_LEN: usize = #payload_encoded_len;

                fn deser(_version: MavlinkVersion, __input: &[u8]) -> Result<Self, ::mavlink_core::error::ParserError> {
                    #deser_vars
                }

                fn ser(&self, version: MavlinkVersion, bytes: &mut [u8]) -> usize {
                    #serialize_vars
                }
            }
        }
    }

    /// Ensures that a message does not contain duplicate field names.
    ///
    /// Duplicate field names would generate invalid Rust structs.
    fn validate_unique_fields(&self) {
        let mut seen: HashSet<&str> = HashSet::new();
        for f in &self.fields {
            let name: &str = &f.name;
            assert!(
                seen.insert(name),
                "Duplicate field '{}' found in message '{}' while generating bindings",
                name,
                self.name
            );
        }
    }

    /// Ensure that the fields count is at least one and no more than 64
    fn validate_field_count(&self) {
        assert!(
            !self.fields.is_empty(),
            "Message '{}' does not any fields",
            self.name
        );
        assert!(
            self.fields.len() <= 64,
            "Message '{}' has more then 64 fields",
            self.name
        );
    }
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavField {
    pub mavtype: MavType,
    pub name: String,
    pub description: Option<String>,
    pub enumtype: Option<String>,
    pub display: Option<String>,
    pub is_extension: bool,
}

impl MavField {
    /// Emit rust name of a given field
    #[inline(always)]
    fn emit_name(&self) -> TokenStream {
        let name = format_ident!("{}", self.name);
        quote!(#name)
    }

    /// Emit rust type of the field
    #[inline(always)]
    fn emit_type(&self) -> TokenStream {
        let mavtype;
        if matches!(self.mavtype, MavType::Array(_, _)) {
            let rt = TokenStream::from_str(&self.mavtype.rust_type()).unwrap();
            mavtype = quote!(#rt);
        } else if let Some(enumname) = &self.enumtype {
            let en = TokenStream::from_str(enumname).unwrap();
            mavtype = quote!(#en);
        } else {
            let rt = TokenStream::from_str(&self.mavtype.rust_type()).unwrap();
            mavtype = quote!(#rt);
        }
        mavtype
    }

    /// Generate description for the given field
    #[inline(always)]
    fn emit_description(&self) -> TokenStream {
        let mut ts = TokenStream::new();
        if let Some(val) = self.description.as_ref() {
            let desc = URL_REGEX.replace_all(val, "<$1>");
            ts.extend(quote!(#[doc = #desc]));
        }
        ts
    }

    /// Combine rust name and type of a given field
    #[inline(always)]
    fn emit_name_type(&self) -> TokenStream {
        let name = self.emit_name();
        let fieldtype = self.emit_type();
        quote!(pub #name: #fieldtype,)
    }

    /// Emit writer
    fn rust_writer(&self) -> TokenStream {
        let mut name = "self.".to_string() + &self.name.clone();
        if self.enumtype.is_some() {
            // casts are not necessary for arrays, because they are currently
            // generated as primitive arrays
            if !matches!(self.mavtype, MavType::Array(_, _)) {
                if let Some(dsp) = &self.display {
                    // potentially a bitflag
                    if dsp == "bitmask" {
                        // it is a bitflag
                        name += ".bits() as ";
                        name += &self.mavtype.rust_type();
                    } else {
                        panic!("Display option not implemented");
                    }
                } else {
                    // an enum, have to use "*foo as u8" cast
                    name += " as ";
                    name += &self.mavtype.rust_type();
                }
            }
        }
        let ts = TokenStream::from_str(&name).unwrap();
        let name = quote!(#ts);
        let buf = format_ident!("__tmp");
        self.mavtype.rust_writer(&name, buf)
    }

    /// Emit reader
    fn rust_reader(&self) -> TokenStream {
        let _name = TokenStream::from_str(&self.name).unwrap();

        let name = quote!(__struct.#_name);
        let buf = format_ident!("buf");
        if let Some(enum_name) = &self.enumtype {
            // TODO: handle enum arrays properly, rather than just generating
            // primitive arrays
            if let MavType::Array(_t, _size) = &self.mavtype {
                return self.mavtype.rust_reader(&name, buf);
            }
            if let Some(dsp) = &self.display {
                if dsp == "bitmask" {
                    // bitflags
                    let tmp = self.mavtype.rust_reader(&quote!(let tmp), buf);
                    let enum_name_ident = format_ident!("{}", enum_name);
                    quote! {
                        #tmp
                        #name = #enum_name_ident::from_bits(tmp as <#enum_name_ident as Flags>::Bits)
                            .ok_or(::mavlink_core::error::ParserError::InvalidFlag { flag_type: #enum_name, value: tmp as u64 })?;
                    }
                } else {
                    panic!("Display option not implemented");
                }
            } else {
                // handle enum by FromPrimitive
                let tmp = self.mavtype.rust_reader(&quote!(let tmp), buf);
                let val = format_ident!("from_{}", &self.mavtype.rust_type());
                quote!(
                    #tmp
                    #name = FromPrimitive::#val(tmp)
                        .ok_or(::mavlink_core::error::ParserError::InvalidEnum { enum_type: #enum_name, value: tmp as u64 })?;
                )
            }
        } else {
            self.mavtype.rust_reader(&name, buf)
        }
    }

    #[inline(always)]
    fn emit_default_initializer(&self, dialect_has_version: bool) -> TokenStream {
        let field = self.emit_name();
        // FIXME: Is this actually expected behaviour??
        if matches!(self.mavtype, MavType::Array(_, _)) {
            let default_value = self.mavtype.emit_default_value(dialect_has_version);
            quote!(#field: #default_value,)
        } else if let Some(enumname) = &self.enumtype {
            let ty = TokenStream::from_str(enumname).unwrap();
            quote!(#field: #ty::DEFAULT,)
        } else {
            let default_value = self.mavtype.emit_default_value(dialect_has_version);
            quote!(#field: #default_value,)
        }
    }
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MavType {
    UInt8MavlinkVersion,
    #[default]
    UInt8,
    UInt16,
    UInt32,
    UInt64,
    Int8,
    Int16,
    Int32,
    Int64,
    Char,
    Float,
    Double,
    CharArray(usize),
    Array(Box<Self>, usize),
}

impl MavType {
    fn parse_type(s: &str) -> Option<Self> {
        use self::MavType::*;
        match s {
            "uint8_t_mavlink_version" => Some(UInt8MavlinkVersion),
            "uint8_t" => Some(UInt8),
            "uint16_t" => Some(UInt16),
            "uint32_t" => Some(UInt32),
            "uint64_t" => Some(UInt64),
            "int8_t" => Some(Int8),
            "int16_t" => Some(Int16),
            "int32_t" => Some(Int32),
            "int64_t" => Some(Int64),
            "char" => Some(Char),
            "float" => Some(Float),
            "Double" => Some(Double),
            "double" => Some(Double),
            _ if s.starts_with("char[") => {
                let start = 4;
                let size = s[start + 1..(s.len() - 1)].parse::<usize>().ok()?;
                Some(CharArray(size))
            }
            _ if s.ends_with(']') => {
                let start = s.find('[')?;
                let size = s[start + 1..(s.len() - 1)].parse::<usize>().ok()?;
                let mtype = Self::parse_type(&s[0..start])?;
                Some(Array(Box::new(mtype), size))
            }
            _ => None,
        }
    }

    /// Emit reader of a given type
    pub fn rust_reader(&self, val: &TokenStream, buf: Ident) -> TokenStream {
        use self::MavType::*;
        match self {
            Char => quote! {#val = #buf.get_u8()?;},
            UInt8 => quote! {#val = #buf.get_u8()?;},
            UInt16 => quote! {#val = #buf.get_u16_le()?;},
            UInt32 => quote! {#val = #buf.get_u32_le()?;},
            UInt64 => quote! {#val = #buf.get_u64_le()?;},
            UInt8MavlinkVersion => quote! {#val = #buf.get_u8()?;},
            Int8 => quote! {#val = #buf.get_i8()?;},
            Int16 => quote! {#val = #buf.get_i16_le()?;},
            Int32 => quote! {#val = #buf.get_i32_le()?;},
            Int64 => quote! {#val = #buf.get_i64_le()?;},
            Float => quote! {#val = #buf.get_f32_le()?;},
            Double => quote! {#val = #buf.get_f64_le()?;},
            CharArray(size) => {
                quote! {
                    let mut tmp = [0_u8; #size];
                    for v in &mut tmp {
                        *v = #buf.get_u8()?;
                    }
                    #val = CharArray::new(tmp);
                }
            }
            Array(t, _) => {
                let r = t.rust_reader(&quote!(let val), buf);
                quote! {
                    for v in &mut #val {
                        #r
                        *v = val;
                    }
                }
            }
        }
    }

    /// Emit writer of a given type
    pub fn rust_writer(&self, val: &TokenStream, buf: Ident) -> TokenStream {
        use self::MavType::*;
        match self {
            UInt8MavlinkVersion => quote! {#buf.put_u8(#val);},
            UInt8 => quote! {#buf.put_u8(#val);},
            Char => quote! {#buf.put_u8(#val);},
            UInt16 => quote! {#buf.put_u16_le(#val);},
            UInt32 => quote! {#buf.put_u32_le(#val);},
            Int8 => quote! {#buf.put_i8(#val);},
            Int16 => quote! {#buf.put_i16_le(#val);},
            Int32 => quote! {#buf.put_i32_le(#val);},
            Float => quote! {#buf.put_f32_le(#val);},
            UInt64 => quote! {#buf.put_u64_le(#val);},
            Int64 => quote! {#buf.put_i64_le(#val);},
            Double => quote! {#buf.put_f64_le(#val);},
            CharArray(_) => {
                let w = Char.rust_writer(&quote!(*val), buf);
                quote! {
                    for val in &#val {
                        #w
                    }
                }
            }
            Array(t, _size) => {
                let w = t.rust_writer(&quote!(*val), buf);
                quote! {
                    for val in &#val {
                        #w
                    }
                }
            }
        }
    }

    /// Size of a given Mavtype
    fn len(&self) -> usize {
        use self::MavType::*;
        match self {
            UInt8MavlinkVersion | UInt8 | Int8 | Char => 1,
            UInt16 | Int16 => 2,
            UInt32 | Int32 | Float => 4,
            UInt64 | Int64 | Double => 8,
            CharArray(size) => *size,
            Array(t, size) => t.len() * size,
        }
    }

    fn max_int_value(&self) -> u64 {
        match self {
            Self::UInt8MavlinkVersion | Self::UInt8 => u8::MAX as u64,
            Self::UInt16 => u16::MAX as u64,
            Self::UInt32 => u32::MAX as u64,
            Self::UInt64 => u64::MAX,
            Self::Int8 | Self::Char | Self::CharArray(_) => i8::MAX as u64,
            Self::Int16 => i16::MAX as u64,
            Self::Int32 => i32::MAX as u64,
            Self::Int64 => i64::MAX as u64,
            // maximum precisly representable value minus 1 for float types
            Self::Float => (1 << f32::MANTISSA_DIGITS) - 1,
            Self::Double => (1 << f64::MANTISSA_DIGITS) - 1,
            Self::Array(mav_type, _) => mav_type.max_int_value(),
        }
    }

    /// Used for ordering of types
    fn order_len(&self) -> usize {
        use self::MavType::*;
        match self {
            UInt8MavlinkVersion | UInt8 | Int8 | Char | CharArray(_) => 1,
            UInt16 | Int16 => 2,
            UInt32 | Int32 | Float => 4,
            UInt64 | Int64 | Double => 8,
            Array(t, _) => t.len(),
        }
    }

    /// Used for crc calculation
    pub fn primitive_type(&self) -> String {
        use self::MavType::*;
        match self {
            UInt8MavlinkVersion => "uint8_t".into(),
            UInt8 => "uint8_t".into(),
            Int8 => "int8_t".into(),
            Char => "char".into(),
            UInt16 => "uint16_t".into(),
            Int16 => "int16_t".into(),
            UInt32 => "uint32_t".into(),
            Int32 => "int32_t".into(),
            Float => "float".into(),
            UInt64 => "uint64_t".into(),
            Int64 => "int64_t".into(),
            Double => "double".into(),
            CharArray(_) => "char".into(),
            Array(t, _) => t.primitive_type(),
        }
    }

    /// Return rust equivalent of a given Mavtype
    /// Used for generating struct fields.
    pub fn rust_type(&self) -> String {
        use self::MavType::*;
        match self {
            UInt8 | UInt8MavlinkVersion => "u8".into(),
            Int8 => "i8".into(),
            Char => "u8".into(),
            UInt16 => "u16".into(),
            Int16 => "i16".into(),
            UInt32 => "u32".into(),
            Int32 => "i32".into(),
            Float => "f32".into(),
            UInt64 => "u64".into(),
            Int64 => "i64".into(),
            Double => "f64".into(),
            CharArray(size) => format!("CharArray<{size}>"),
            Array(t, size) => format!("[{};{}]", t.rust_type(), size),
        }
    }

    pub fn emit_default_value(&self, dialect_has_version: bool) -> TokenStream {
        use self::MavType::*;
        match self {
            UInt8 => quote!(0_u8),
            UInt8MavlinkVersion => {
                if dialect_has_version {
                    quote!(MINOR_MAVLINK_VERSION)
                } else {
                    quote!(0_u8)
                }
            }
            Int8 => quote!(0_i8),
            Char => quote!(0_u8),
            UInt16 => quote!(0_u16),
            Int16 => quote!(0_i16),
            UInt32 => quote!(0_u32),
            Int32 => quote!(0_i32),
            Float => quote!(0.0_f32),
            UInt64 => quote!(0_u64),
            Int64 => quote!(0_i64),
            Double => quote!(0.0_f64),
            CharArray(size) => quote!(CharArray::new([0_u8; #size])),
            Array(ty, size) => {
                let default_value = ty.emit_default_value(dialect_has_version);
                quote!([#default_value; #size])
            }
        }
    }

    /// Return rust equivalent of the primitive type of a MavType. The primitive
    /// type is the type itself for all except arrays, in which case it is the
    /// element type.
    pub fn rust_primitive_type(&self) -> String {
        use self::MavType::*;
        match self {
            Array(t, _) => t.rust_primitive_type(),
            _ => self.rust_type(),
        }
    }

    /// Compare two MavTypes
    pub fn compare(&self, other: &Self) -> Ordering {
        let len = self.order_len();
        (-(len as isize)).cmp(&(-(other.order_len() as isize)))
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Default)]
pub enum MavDeprecationType {
    #[default]
    Deprecated,
    Superseded,
}

impl Display for MavDeprecationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Deprecated => f.write_str("Deprecated"),
            Self::Superseded => f.write_str("Superseded"),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavDeprecation {
    // YYYY-MM
    pub since: String,
    pub replaced_by: Option<String>,
    pub deprecation_type: MavDeprecationType,
    pub note: Option<String>,
}

impl MavDeprecation {
    pub fn emit_tokens(&self) -> TokenStream {
        let since = &self.since;
        let note = match &self.note {
            Some(str) if str.is_empty() || str.ends_with(".") => str.clone(),
            Some(str) => format!("{str}."),
            None => String::new(),
        };
        let replaced_by = match &self.replaced_by {
            Some(str) if str.starts_with('`') => format!("See {str}"),
            Some(str) => format!("See `{str}`"),
            None => String::new(),
        };
        let message = format!(
            "{note} {replaced_by} ({} since {since})",
            self.deprecation_type
        );
        quote!(#[deprecated = #message])
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavSuperseded {
    // YYYY-MM
    pub since: String,
    // maybe empty, may be encapuslated in `` and contain a wildcard
    pub replaced_by: String,
    pub note: Option<String>,
}

impl MavSuperseded {
    pub fn emit_tokens(&self) -> TokenStream {
        let since = &self.since;
        let note = match &self.note {
            Some(str) if str.is_empty() || str.ends_with(".") => str.clone(),
            Some(str) => format!("{str}."),
            None => String::new(),
        };
        let replaced_by = if self.replaced_by.starts_with("`") {
            format!("See {}", self.replaced_by)
        } else if self.replaced_by.is_empty() {
            String::new()
        } else {
            format!("See `{}`", self.replaced_by)
        };
        let message = format!("{note} {replaced_by} (Superseded since {since})");
        quote!(#[superseded = #message])
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
pub enum MavXmlElement {
    Version,
    Mavlink,
    Dialect,
    Include,
    Enums,
    Enum,
    Entry,
    Description,
    Param,
    Messages,
    Message,
    Field,
    Deprecated,
    Wip,
    Extensions,
    Superseded,
}

const fn identify_element(s: &[u8]) -> Option<MavXmlElement> {
    use self::MavXmlElement::*;
    match s {
        b"version" => Some(Version),
        b"mavlink" => Some(Mavlink),
        b"dialect" => Some(Dialect),
        b"include" => Some(Include),
        b"enums" => Some(Enums),
        b"enum" => Some(Enum),
        b"entry" => Some(Entry),
        b"description" => Some(Description),
        b"param" => Some(Param),
        b"messages" => Some(Messages),
        b"message" => Some(Message),
        b"field" => Some(Field),
        b"deprecated" => Some(Deprecated),
        b"wip" => Some(Wip),
        b"extensions" => Some(Extensions),
        b"superseded" => Some(Superseded),
        _ => None,
    }
}

fn is_valid_parent(p: Option<MavXmlElement>, s: MavXmlElement) -> bool {
    use self::MavXmlElement::*;
    match s {
        Version => p == Some(Mavlink),
        Mavlink => p.is_none(),
        Dialect => p == Some(Mavlink),
        Include => p == Some(Mavlink),
        Enums => p == Some(Mavlink),
        Enum => p == Some(Enums),
        Entry => p == Some(Enum),
        Description => p == Some(Entry) || p == Some(Message) || p == Some(Enum),
        Param => p == Some(Entry),
        Messages => p == Some(Mavlink),
        Message => p == Some(Messages),
        Field => p == Some(Message),
        Deprecated => p == Some(Entry) || p == Some(Message) || p == Some(Enum),
        Wip => p == Some(Entry) || p == Some(Message) || p == Some(Enum),
        Extensions => p == Some(Message),
        Superseded => p == Some(Entry) || p == Some(Message) || p == Some(Enum),
    }
}

pub fn parse_profile(
    definitions_dir: &Path,
    definition_file: &Path,
    parsed_files: &mut HashSet<PathBuf>,
) -> Result<MavProfile, BindGenError> {
    let in_path = Path::new(&definitions_dir).join(definition_file);
    parsed_files.insert(in_path.clone()); // Keep track of which files have been parsed

    let mut stack: Vec<MavXmlElement> = vec![];

    let mut text = None;

    let mut profile = MavProfile::default();
    let mut field = MavField::default();
    let mut message = MavMessage::default();
    let mut mavenum = MavEnum::default();
    let mut entry = MavEnumEntry::default();
    let mut param_index: Option<usize> = None;
    let mut param_label: Option<String> = None;
    let mut param_units: Option<String> = None;
    let mut param_enum: Option<String> = None;
    let mut param_increment: Option<f32> = None;
    let mut param_min_value: Option<f32> = None;
    let mut param_max_value: Option<f32> = None;
    let mut param_reserved = false;
    let mut param_default: Option<f32> = None;
    let mut deprecated: Option<MavDeprecation> = None;

    let mut xml_filter = MavXmlFilter::default();
    let mut events: Vec<Result<Event, quick_xml::Error>> = Vec::new();
    let xml = std::fs::read_to_string(&in_path).map_err(|e| {
        BindGenError::CouldNotReadDefinitionFile {
            source: e,
            path: in_path.clone(),
        }
    })?;
    let mut reader = Reader::from_str(&xml);
    reader.config_mut().trim_text(true);
    reader.config_mut().expand_empty_elements = true;

    loop {
        match reader.read_event() {
            Ok(Event::Eof) => {
                events.push(Ok(Event::Eof));
                break;
            }
            Ok(event) => events.push(Ok(event.into_owned())),
            Err(why) => events.push(Err(why)),
        }
    }
    xml_filter.filter(&mut events);
    let mut is_in_extension = false;
    for e in events {
        match e {
            Ok(Event::Start(bytes)) => {
                let Some(id) = identify_element(bytes.name().into_inner()) else {
                    panic!(
                        "unexpected element {:?}",
                        String::from_utf8_lossy(bytes.name().into_inner())
                    );
                };

                assert!(
                    is_valid_parent(stack.last().copied(), id),
                    "not valid parent {:?} of {id:?}",
                    stack.last(),
                );

                match id {
                    MavXmlElement::Extensions => {
                        is_in_extension = true;
                    }
                    MavXmlElement::Message => {
                        message = MavMessage::default();
                    }
                    MavXmlElement::Field => {
                        field = MavField {
                            is_extension: is_in_extension,
                            ..Default::default()
                        };
                    }
                    MavXmlElement::Enum => {
                        mavenum = MavEnum::default();
                    }
                    MavXmlElement::Entry => {
                        if mavenum.entries.is_empty() {
                            mavenum.deprecated = deprecated;
                        }
                        deprecated = None;
                        entry = MavEnumEntry::default();
                    }
                    MavXmlElement::Param => {
                        param_index = None;
                        param_increment = None;
                        param_min_value = None;
                        param_max_value = None;
                        param_reserved = false;
                        param_default = None;
                    }
                    MavXmlElement::Deprecated => {
                        deprecated = Some(MavDeprecation {
                            replaced_by: None,
                            since: String::new(),
                            deprecation_type: MavDeprecationType::Deprecated,
                            note: None,
                        });
                    }
                    MavXmlElement::Superseded => {
                        deprecated = Some(MavDeprecation {
                            replaced_by: Some(String::new()),
                            since: String::new(),
                            deprecation_type: MavDeprecationType::Superseded,
                            note: None,
                        });
                    }
                    _ => (),
                }
                stack.push(id);

                for attr in bytes.attributes() {
                    let attr = attr.unwrap();
                    match stack.last() {
                        Some(&MavXmlElement::Enum) => {
                            if attr.key.into_inner() == b"name" {
                                mavenum.name = to_pascal_case(attr.value);
                                //mavenum.name = attr.value.clone();
                            } else if attr.key.into_inner() == b"bitmask" {
                                mavenum.bitmask = true;
                            }
                        }
                        Some(&MavXmlElement::Entry) => {
                            match attr.key.into_inner() {
                                b"name" => {
                                    entry.name = String::from_utf8_lossy(&attr.value).to_string();
                                }
                                b"value" => {
                                    let value = String::from_utf8_lossy(&attr.value);
                                    // Deal with hexadecimal numbers
                                    let (src, radix) = value
                                        .strip_prefix("0x")
                                        .map(|value| (value, 16))
                                        .unwrap_or((value.as_ref(), 10));
                                    entry.value = u64::from_str_radix(src, radix).ok();
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Message) => {
                            match attr.key.into_inner() {
                                b"name" => {
                                    /*message.name = attr
                                    .value
                                    .clone()
                                    .split("_")
                                    .map(|x| x.to_lowercase())
                                    .map(|x| {
                                        let mut v: Vec<char> = x.chars().collect();
                                        v[0] = v[0].to_uppercase().nth(0).unwrap();
                                        v.into_iter().collect()
                                    })
                                    .collect::<Vec<String>>()
                                    .join("");
                                    */
                                    message.name = String::from_utf8_lossy(&attr.value).to_string();
                                }
                                b"id" => {
                                    message.id =
                                        String::from_utf8_lossy(&attr.value).parse().unwrap();
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Field) => {
                            match attr.key.into_inner() {
                                b"name" => {
                                    let name = String::from_utf8_lossy(&attr.value);
                                    field.name = if name == "type" {
                                        "mavtype".to_string()
                                    } else {
                                        name.to_string()
                                    };
                                }
                                b"type" => {
                                    let r#type = String::from_utf8_lossy(&attr.value);
                                    field.mavtype = MavType::parse_type(&r#type).unwrap();
                                }
                                b"enum" => {
                                    field.enumtype = Some(to_pascal_case(&attr.value));

                                    // Update field display if enum is a bitmask
                                    if let Some(e) =
                                        profile.enums.get(field.enumtype.as_ref().unwrap())
                                    {
                                        if e.bitmask {
                                            field.display = Some("bitmask".to_string());
                                        }
                                    }
                                }
                                b"display" => {
                                    field.display =
                                        Some(String::from_utf8_lossy(&attr.value).to_string());
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Param) => {
                            if entry.params.is_none() {
                                entry.params = Some(vec![]);
                            }
                            match attr.key.into_inner() {
                                b"index" => {
                                    let value = String::from_utf8_lossy(&attr.value)
                                        .parse()
                                        .expect("failed to parse param index");
                                    assert!(
                                        (1..=7).contains(&value),
                                        "param index must be between 1 and 7"
                                    );
                                    param_index = Some(value);
                                }
                                b"label" => {
                                    param_label =
                                        std::str::from_utf8(&attr.value).ok().map(str::to_owned);
                                }
                                b"increment" => {
                                    param_increment = Some(
                                        String::from_utf8_lossy(&attr.value)
                                            .parse()
                                            .expect("failed to parse param increment"),
                                    );
                                }
                                b"minValue" => {
                                    param_min_value = Some(
                                        String::from_utf8_lossy(&attr.value)
                                            .parse()
                                            .expect("failed to parse param minValue"),
                                    );
                                }
                                b"maxValue" => {
                                    param_max_value = Some(
                                        String::from_utf8_lossy(&attr.value)
                                            .parse()
                                            .expect("failed to parse param maxValue"),
                                    );
                                }
                                b"units" => {
                                    param_units =
                                        std::str::from_utf8(&attr.value).ok().map(str::to_owned);
                                }
                                b"enum" => {
                                    param_enum =
                                        std::str::from_utf8(&attr.value).ok().map(to_pascal_case);
                                }
                                b"reserved" => {
                                    param_reserved = attr.value.as_ref() == b"true";
                                }
                                b"default" => {
                                    param_default = Some(
                                        String::from_utf8_lossy(&attr.value)
                                            .parse()
                                            .expect("failed to parse param maxValue"),
                                    );
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Deprecated) => match attr.key.into_inner() {
                            b"since" => {
                                deprecated.as_mut().unwrap().since =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                            b"replaced_by" => {
                                let value = String::from_utf8_lossy(&attr.value);
                                deprecated.as_mut().unwrap().replaced_by = if value.is_empty() {
                                    None
                                } else {
                                    Some(value.to_string())
                                };
                            }
                            _ => (),
                        },
                        Some(&MavXmlElement::Superseded) => match attr.key.into_inner() {
                            b"since" => {
                                deprecated.as_mut().unwrap().since =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                            b"replaced_by" => {
                                deprecated.as_mut().unwrap().replaced_by =
                                    Some(String::from_utf8_lossy(&attr.value).to_string());
                            }
                            _ => (),
                        },
                        _ => (),
                    }
                }
            }
            Ok(Event::Text(bytes)) => {
                let s = String::from_utf8_lossy(&bytes);

                use self::MavXmlElement::*;
                match (stack.last(), stack.get(stack.len() - 2)) {
                    (Some(&Description), Some(&Message))
                    | (Some(&Field), Some(&Message))
                    | (Some(&Description), Some(&Enum))
                    | (Some(&Description), Some(&Entry))
                    | (Some(&Include), Some(&Mavlink))
                    | (Some(&Version), Some(&Mavlink))
                    | (Some(&Dialect), Some(&Mavlink))
                    | (Some(&Param), Some(&Entry))
                    | (Some(Deprecated), _)
                    | (Some(Superseded), _) => {
                        text = Some(text.map(|t| t + s.as_ref()).unwrap_or(s.to_string()));
                    }
                    data => {
                        panic!("unexpected text data {data:?} reading {s:?}");
                    }
                }
            }
            Ok(Event::GeneralRef(bytes)) => {
                let entity = String::from_utf8_lossy(&bytes);
                text = Some(
                    text.map(|t| format!("{t}&{entity};"))
                        .unwrap_or(format!("&{entity};")),
                );
            }
            Ok(Event::End(_)) => {
                match stack.last() {
                    Some(&MavXmlElement::Field) => {
                        field.description = text.map(|t| t.replace('\n', " "));
                        message.fields.push(field.clone());
                    }
                    Some(&MavXmlElement::Entry) => {
                        entry.deprecated = deprecated;
                        deprecated = None;
                        mavenum.entries.push(entry.clone());
                    }
                    Some(&MavXmlElement::Message) => {
                        message.deprecated = deprecated;

                        deprecated = None;
                        is_in_extension = false;
                        // Follow mavlink ordering specification: https://mavlink.io/en/guide/serialization.html#field_reordering
                        let mut not_extension_fields = message.fields.clone();
                        let mut extension_fields = message.fields.clone();

                        not_extension_fields.retain(|field| !field.is_extension);
                        extension_fields.retain(|field| field.is_extension);

                        // Only not mavlink 1 fields need to be sorted
                        not_extension_fields.sort_by(|a, b| a.mavtype.compare(&b.mavtype));

                        // Update msg fields and add the new message
                        let mut msg = message.clone();
                        msg.fields.clear();
                        msg.fields.extend(not_extension_fields);
                        msg.fields.extend(extension_fields);

                        // Validate there are no duplicate field names
                        msg.validate_unique_fields();
                        // Validate field count must be between 1 and 64
                        msg.validate_field_count();

                        profile.add_message(&msg);
                    }
                    Some(&MavXmlElement::Enum) => {
                        profile.add_enum(&mavenum);
                    }
                    Some(&MavXmlElement::Include) => {
                        let include =
                            PathBuf::from(text.map(|t| t.replace('\n', "")).unwrap_or_default());
                        let include_file = Path::new(&definitions_dir).join(include.clone());
                        if !parsed_files.contains(&include_file) {
                            let included_profile =
                                parse_profile(definitions_dir, &include, parsed_files)?;
                            for message in included_profile.messages.values() {
                                profile.add_message(message);
                            }
                            for enm in included_profile.enums.values() {
                                profile.add_enum(enm);
                            }
                            if profile.version.is_none() {
                                profile.version = included_profile.version;
                            }
                        }
                    }
                    Some(&MavXmlElement::Description) => match stack.get(stack.len() - 2) {
                        Some(&MavXmlElement::Message) => {
                            message.description = text.map(|t| t.replace('\n', " "));
                        }
                        Some(&MavXmlElement::Enum) => {
                            mavenum.description = text.map(|t| t.replace('\n', " "));
                        }
                        Some(&MavXmlElement::Entry) => {
                            entry.description = text.map(|t| t.replace('\n', " "));
                        }
                        _ => (),
                    },
                    Some(&MavXmlElement::Version) => {
                        if let Some(t) = text {
                            profile.version =
                                Some(t.parse().expect("Invalid minor version number format"));
                        }
                    }
                    Some(&MavXmlElement::Dialect) => {
                        if let Some(t) = text {
                            profile.dialect =
                                Some(t.parse().expect("Invalid dialect number format"));
                        }
                    }
                    Some(&MavXmlElement::Deprecated) => {
                        if let Some(t) = text {
                            deprecated.as_mut().unwrap().note = Some(t);
                        }
                    }
                    Some(&MavXmlElement::Param) => {
                        if let Some(params) = entry.params.as_mut() {
                            // Some messages can jump between values, like: 1, 2, 7
                            let param_index = param_index.expect("entry params must have an index");
                            while params.len() < param_index {
                                params.push(MavParam {
                                    index: params.len() + 1,
                                    description: None,
                                    ..Default::default()
                                });
                            }
                            if let Some((min, max)) = param_min_value.zip(param_max_value) {
                                assert!(
                                    min <= max,
                                    "param minValue must not be greater than maxValue"
                                );
                            }
                            params[param_index - 1] = MavParam {
                                index: param_index,
                                description: text.map(|t| t.replace('\n', " ")),
                                label: param_label,
                                units: param_units,
                                enum_used: param_enum,
                                increment: param_increment,
                                max_value: param_max_value,
                                min_value: param_min_value,
                                reserved: param_reserved,
                                default: param_default,
                            };
                            param_label = None;
                            param_units = None;
                            param_enum = None;
                        }
                    }
                    _ => (),
                }
                text = None;
                stack.pop();
                // println!("{}-{}", indent(depth), name);
            }
            Err(e) => {
                eprintln!("Error: {e}");
                break;
            }
            _ => {}
        }
    }

    //let profile = profile.update_messages(); //TODO verify no longer needed
    Ok(profile.update_enums())
}

/// Generate protobuf represenation of mavlink message set
/// Generate rust representation of mavlink message set with appropriate conversion methods
pub fn generate<W: Write>(
    definitions_dir: &Path,
    definition_file: &Path,
    output_rust: &mut W,
) -> Result<(), BindGenError> {
    let mut parsed_files: HashSet<PathBuf> = HashSet::new();
    let profile = parse_profile(definitions_dir, definition_file, &mut parsed_files)?;

    let dialect_name = util::to_dialect_name(definition_file);

    // rust file
    let rust_tokens = profile.emit_rust(&dialect_name);
    writeln!(output_rust, "{rust_tokens}").unwrap();

    Ok(())
}

/// CRC operates over names of the message and names of its fields
/// Hence we have to preserve the original uppercase names delimited with an underscore
/// For field names, we replace "type" with "mavtype" to make it rust compatible (this is
/// needed for generating sensible rust code), but for calculating crc function we have to
/// use the original name "type"
pub fn extra_crc(msg: &MavMessage) -> u8 {
    // calculate a 8-bit checksum of the key fields of a message, so we
    // can detect incompatible XML changes
    let mut crc = CRCu16::crc16mcrf4cc();

    crc.digest(msg.name.as_bytes());
    crc.digest(b" ");

    let mut f = msg.fields.clone();
    // only mavlink 1 fields should be part of the extra_crc
    f.retain(|f| !f.is_extension);
    f.sort_by(|a, b| a.mavtype.compare(&b.mavtype));
    for field in &f {
        crc.digest(field.mavtype.primitive_type().as_bytes());
        crc.digest(b" ");
        if field.name == "mavtype" {
            crc.digest(b"type");
        } else {
            crc.digest(field.name.as_bytes());
        }
        crc.digest(b" ");
        if let MavType::Array(_, size) | MavType::CharArray(size) = field.mavtype {
            crc.digest(&[size as u8]);
        }
    }

    let crcval = crc.get_crc();
    ((crcval & 0xFF) ^ (crcval >> 8)) as u8
}

#[cfg(not(feature = "emit-extensions"))]
struct ExtensionFilter {
    pub is_in: bool,
}

struct MessageFilter {
    pub is_in: bool,
    pub messages: Vec<String>,
}

impl MessageFilter {
    pub fn new() -> Self {
        Self {
            is_in: false,
            messages: vec![
                // device_cap_flags is u32, when enum is u16, which is not handled by the parser yet
                "STORM32_GIMBAL_MANAGER_INFORMATION".to_string(),
            ],
        }
    }
}

struct MavXmlFilter {
    #[cfg(not(feature = "emit-extensions"))]
    extension_filter: ExtensionFilter,
    message_filter: MessageFilter,
}

impl Default for MavXmlFilter {
    fn default() -> Self {
        Self {
            #[cfg(not(feature = "emit-extensions"))]
            extension_filter: ExtensionFilter { is_in: false },
            message_filter: MessageFilter::new(),
        }
    }
}

impl MavXmlFilter {
    pub fn filter(&mut self, elements: &mut Vec<Result<Event, quick_xml::Error>>) {
        elements.retain(|x| self.filter_extension(x) && self.filter_messages(x));
    }

    #[cfg(feature = "emit-extensions")]
    pub fn filter_extension(&mut self, _element: &Result<Event, quick_xml::Error>) -> bool {
        true
    }

    /// Ignore extension fields
    #[cfg(not(feature = "emit-extensions"))]
    pub fn filter_extension(&mut self, element: &Result<Event, quick_xml::Error>) -> bool {
        match element {
            Ok(content) => {
                match content {
                    Event::Start(bytes) | Event::Empty(bytes) => {
                        let Some(id) = identify_element(bytes.name().into_inner()) else {
                            panic!(
                                "unexpected element {:?}",
                                String::from_utf8_lossy(bytes.name().into_inner())
                            );
                        };
                        if id == MavXmlElement::Extensions {
                            self.extension_filter.is_in = true;
                        }
                    }
                    Event::End(bytes) => {
                        let Some(id) = identify_element(bytes.name().into_inner()) else {
                            panic!(
                                "unexpected element {:?}",
                                String::from_utf8_lossy(bytes.name().into_inner())
                            );
                        };

                        if id == MavXmlElement::Message {
                            self.extension_filter.is_in = false;
                        }
                    }
                    _ => {}
                }
                !self.extension_filter.is_in
            }
            Err(error) => panic!("Failed to filter XML: {error}"),
        }
    }

    /// Filters messages by their name
    pub fn filter_messages(&mut self, element: &Result<Event, quick_xml::Error>) -> bool {
        match element {
            Ok(content) => {
                match content {
                    Event::Start(bytes) | Event::Empty(bytes) => {
                        let Some(id) = identify_element(bytes.name().into_inner()) else {
                            panic!(
                                "unexpected element {:?}",
                                String::from_utf8_lossy(bytes.name().into_inner())
                            );
                        };
                        if id == MavXmlElement::Message {
                            for attr in bytes.attributes() {
                                let attr = attr.unwrap();
                                if attr.key.into_inner() == b"name" {
                                    let value = String::from_utf8_lossy(&attr.value).into_owned();
                                    if self.message_filter.messages.contains(&value) {
                                        self.message_filter.is_in = true;
                                        return false;
                                    }
                                }
                            }
                        }
                    }
                    Event::End(bytes) => {
                        let Some(id) = identify_element(bytes.name().into_inner()) else {
                            panic!(
                                "unexpected element {:?}",
                                String::from_utf8_lossy(bytes.name().into_inner())
                            );
                        };

                        if id == MavXmlElement::Message && self.message_filter.is_in {
                            self.message_filter.is_in = false;
                            return false;
                        }
                    }
                    _ => {}
                }
                !self.message_filter.is_in
            }
            Err(error) => panic!("Failed to filter XML: {error}"),
        }
    }
}

#[inline(always)]
fn to_pascal_case(text: impl AsRef<[u8]>) -> String {
    let input = text.as_ref();
    let mut result = String::with_capacity(input.len());
    let mut capitalize = true;

    for &b in input {
        if b == b'_' {
            capitalize = true;
            continue;
        }

        if capitalize {
            result.push((b as char).to_ascii_uppercase());
            capitalize = false;
        } else {
            result.push((b as char).to_ascii_lowercase());
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emits_target_id_match_arms() {
        // Build a minimal profile containing one message with target fields and one without
        let mut profile = MavProfile::default();

        let msg_with_targets = MavMessage {
            id: 300,
            name: "COMMAND_INT".to_string(),
            description: None,
            fields: vec![
                MavField {
                    mavtype: MavType::UInt8,
                    name: "target_system".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
                MavField {
                    mavtype: MavType::UInt8,
                    name: "target_component".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
            ],
            deprecated: None,
        };

        let msg_without_targets = MavMessage {
            id: 0,
            name: "HEARTBEAT".to_string(),
            description: None,
            fields: vec![MavField {
                mavtype: MavType::UInt32,
                name: "custom_mode".to_string(),
                description: None,
                enumtype: None,
                display: None,
                is_extension: false,
            }],
            deprecated: None,
        };

        profile.add_message(&msg_with_targets);
        profile.add_message(&msg_without_targets);

        let tokens = profile.emit_rust("common");
        let mut code = tokens.to_string();
        code.retain(|c| !c.is_whitespace());

        // Check the code contains the target_system/component_id functions
        assert!(code.contains("fntarget_system_id(&self)->Option<u8>"));
        assert!(code.contains("fntarget_component_id(&self)->Option<u8>"));

        // Check the generated impl contains arms referencing COMMAND_INT(inner).target_system/component
        assert!(code.contains("Self::COMMAND_INT(inner)=>Some(inner.target_system)"));
        assert!(code.contains("Self::COMMAND_INT(inner)=>Some(inner.target_component)"));

        // Ensure a message without target fields returns None
        assert!(!code.contains("Self::HEARTBEAT(inner)=>Some(inner.target_system)"));
        assert!(!code.contains("Self::HEARTBEAT(inner)=>Some(inner.target_component)"));
    }

    #[test]
    fn validate_unique_fields_allows_unique() {
        let msg = MavMessage {
            id: 1,
            name: "FOO".to_string(),
            description: None,
            fields: vec![
                MavField {
                    mavtype: MavType::UInt8,
                    name: "a".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
                MavField {
                    mavtype: MavType::UInt16,
                    name: "b".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
            ],
            deprecated: None,
        };
        // Should not panic
        msg.validate_unique_fields();
    }

    #[test]
    #[should_panic(expected = "Duplicate field")]
    fn validate_unique_fields_panics_on_duplicate() {
        let msg = MavMessage {
            id: 2,
            name: "BAR".to_string(),
            description: None,
            fields: vec![
                MavField {
                    mavtype: MavType::UInt8,
                    name: "target_system".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
                MavField {
                    mavtype: MavType::UInt8,
                    name: "target_system".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
            ],
            deprecated: None,
        };
        // Should panic due to duplicate field names
        msg.validate_unique_fields();
    }

    #[test]
    fn validate_field_count_ok() {
        let msg = MavMessage {
            id: 2,
            name: "FOO".to_string(),
            description: None,
            fields: vec![
                MavField {
                    mavtype: MavType::UInt8,
                    name: "a".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
                MavField {
                    mavtype: MavType::UInt8,
                    name: "b".to_string(),
                    description: None,
                    enumtype: None,
                    display: None,
                    is_extension: false,
                },
            ],
            deprecated: None,
        };
        // Should not panic
        msg.validate_field_count();
    }

    #[test]
    #[should_panic]
    fn validate_field_count_too_many() {
        let mut fields = vec![];
        for i in 0..65 {
            let field = MavField {
                mavtype: MavType::UInt8,
                name: format!("field_{i}"),
                description: None,
                enumtype: None,
                display: None,
                is_extension: false,
            };
            fields.push(field);
        }
        let msg = MavMessage {
            id: 2,
            name: "BAZ".to_string(),
            description: None,
            fields,
            deprecated: None,
        };
        // Should panic due to 65 fields
        msg.validate_field_count();
    }

    #[test]
    #[should_panic]
    fn validate_field_count_empty() {
        let msg = MavMessage {
            id: 2,
            name: "BAM".to_string(),
            description: None,
            fields: vec![],
            deprecated: None,
        };
        // Should panic due to no fields
        msg.validate_field_count();
    }

    #[test]
    fn test_fmt_mav_param_values() {
        let enum_param = MavParam {
            enum_used: Some("ENUM_NAME".to_string()),
            ..Default::default()
        };
        assert_eq!(enum_param.format_valid_values(), "[`ENUM_NAME`]");

        let reserved_param = MavParam {
            reserved: true,
            default: Some(f32::NAN),
            ..Default::default()
        };
        assert_eq!(reserved_param.format_valid_values(), "Reserved (use NaN)");

        let unrestricted_param = MavParam::default();
        assert_eq!(unrestricted_param.format_valid_values(), "");

        let int_param = MavParam {
            increment: Some(1.0),
            ..Default::default()
        };
        assert_eq!(int_param.format_valid_values(), "Multiples of 1");

        let pos_param = MavParam {
            min_value: Some(0.0),
            ..Default::default()
        };
        assert_eq!(pos_param.format_valid_values(), "&ge; 0");

        let max_param = MavParam {
            max_value: Some(5.5),
            ..Default::default()
        };
        assert_eq!(max_param.format_valid_values(), "&le; 5.5");

        let pos_int_param = MavParam {
            min_value: Some(0.0),
            increment: Some(1.0),
            ..Default::default()
        };
        assert_eq!(pos_int_param.format_valid_values(), "0, 1, ..");

        let max_inc_param = MavParam {
            increment: Some(1.0),
            max_value: Some(360.0),
            ..Default::default()
        };
        assert_eq!(max_inc_param.format_valid_values(), ".., 359, 360");

        let range_param = MavParam {
            min_value: Some(0.0),
            max_value: Some(10.0),
            ..Default::default()
        };
        assert_eq!(range_param.format_valid_values(), "0 .. 10");

        let int_range_param = MavParam {
            min_value: Some(0.0),
            max_value: Some(10.0),
            increment: Some(1.0),
            ..Default::default()
        };
        assert_eq!(int_range_param.format_valid_values(), "0, 1, .. , 10");

        let close_inc_range_param = MavParam {
            min_value: Some(-2.0),
            max_value: Some(2.0),
            increment: Some(2.0),
            ..Default::default()
        };
        assert_eq!(close_inc_range_param.format_valid_values(), "-2, 0, 2");

        let bin_range_param = MavParam {
            min_value: Some(0.0),
            max_value: Some(1.0),
            increment: Some(1.0),
            ..Default::default()
        };
        assert_eq!(bin_range_param.format_valid_values(), "0, 1");
    }

    #[test]
    fn test_emit_doc_row() {
        let param = MavParam {
            index: 3,
            label: Some("test param".to_string()),
            min_value: Some(0.0),
            units: Some("m/s".to_string()),
            ..Default::default()
        };
        // test with all variations of columns
        assert_eq!(
            param.emit_doc_row(false, false).to_string(),
            quote! {#[doc = "| 3 (test param)|             |"]}.to_string()
        );
        assert_eq!(
            param.emit_doc_row(false, true).to_string(),
            quote! {#[doc = "| 3 (test param)|             | m/s |"]}.to_string()
        );
        assert_eq!(
            param.emit_doc_row(true, false).to_string(),
            quote! {#[doc = "| 3 (test param)|             | &ge; 0 |"]}.to_string()
        );
        assert_eq!(
            param.emit_doc_row(true, true).to_string(),
            quote! {#[doc = "| 3 (test param)|             | &ge; 0 | m/s |"]}.to_string()
        );

        let unlabeled_param = MavParam {
            index: 2,
            ..Default::default()
        };
        assert_eq!(
            unlabeled_param.emit_doc_row(false, false).to_string(),
            quote! {#[doc = "| 2         |             |"]}.to_string()
        );
    }
}
