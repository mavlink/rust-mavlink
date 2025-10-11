use crc_any::CRCu16;
use std::cmp::Ordering;
use std::collections::btree_map::Entry;
use std::collections::{BTreeMap, HashSet};
use std::default::Default;
use std::fs::File;
use std::io::{BufReader, Write};
use std::path::{Path, PathBuf};
use std::str::FromStr;

use lazy_static::lazy_static;
use regex::Regex;

use quick_xml::{events::Event, Reader};

use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::error::BindGenError;
use crate::util;

lazy_static! {
    static ref URL_REGEX: Regex = {
        Regex::new(concat!(
            r"(https?://",                          // url scheme
            r"([-a-zA-Z0-9@:%._\+~#=]{2,256}\.)+", // one or more subdomains
            r"[a-zA-Z]{2,63}",                     // root domain
            r"\b([-a-zA-Z0-9@:%_\+.~#?&/=]*[-a-zA-Z0-9@:%_\+~#?&/=])?)"      // optional query or url fragments

        ))
        .expect("failed to build regex")
    };
}

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
            #[allow(unused_imports)]
            use num_derive::FromPrimitive;
            #[allow(unused_imports)]
            use num_traits::FromPrimitive;
            #[allow(unused_imports)]
            use num_derive::ToPrimitive;
            #[allow(unused_imports)]
            use num_traits::ToPrimitive;
            #[allow(unused_imports)]
            use bitflags::bitflags;

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
}

#[derive(Debug, PartialEq, Eq, Clone, Default)]
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
        let mut cnt = 0u32;
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

                if enum_entry.value.is_none() {
                    cnt += 1;
                    value = quote!(#cnt);
                } else {
                    let tmp_value = enum_entry.value.unwrap();
                    cnt = cnt.max(tmp_value);
                    let tmp = TokenStream::from_str(&tmp_value.to_string()).unwrap();
                    value = quote!(#tmp);
                }
                if self.primitive.is_some() {
                    quote! {
                        #deprecation
                        #description
                        const #name = #value;
                    }
                } else {
                    quote! {
                        #deprecation
                        #description
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
            }

            impl Default for #enum_name {
                fn default() -> Self {
                    Self::DEFAULT
                }
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavEnumEntry {
    pub value: Option<u32>,
    pub name: String,
    pub description: Option<String>,
    pub params: Option<Vec<String>>,
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
                        name += ".bits()";
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
                        #name = #enum_name_ident::from_bits(tmp & #enum_name_ident::all().bits())
                            .ok_or(::mavlink_core::error::ParserError::InvalidFlag { flag_type: #enum_name, value: tmp as u32 })?;
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
                        .ok_or(::mavlink_core::error::ParserError::InvalidEnum { enum_type: #enum_name, value: tmp as u32 })?;
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
    Array(Box<MavType>, usize),
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
                let start = s.find('[')?;
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
            Char => quote! {#val = #buf.get_u8();},
            UInt8 => quote! {#val = #buf.get_u8();},
            UInt16 => quote! {#val = #buf.get_u16_le();},
            UInt32 => quote! {#val = #buf.get_u32_le();},
            UInt64 => quote! {#val = #buf.get_u64_le();},
            UInt8MavlinkVersion => quote! {#val = #buf.get_u8();},
            Int8 => quote! {#val = #buf.get_i8();},
            Int16 => quote! {#val = #buf.get_i16_le();},
            Int32 => quote! {#val = #buf.get_i32_le();},
            Int64 => quote! {#val = #buf.get_i64_le();},
            Float => quote! {#val = #buf.get_f32_le();},
            Double => quote! {#val = #buf.get_f64_le();},
            CharArray(size) => {
                quote! {
                    let mut tmp = [0_u8; #size];
                    for v in &mut tmp {
                        *v = #buf.get_u8();
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
            CharArray(size) => format!("CharArray<{}>", size),
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavDeprecation {
    // YYYY-MM
    pub since: String,
    // maybe empty, may be encapuslated in `` and contain a wildcard
    pub replaced_by: String,
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
        let replaced_by = if self.replaced_by.starts_with("`") {
            format!("See {}", self.replaced_by)
        } else if self.replaced_by.is_empty() {
            String::new()
        } else {
            format!("See `{}`", self.replaced_by)
        };
        let message = format!("{note} {replaced_by} (Deprecated since {since})");
        quote!(#[deprecated = #message])
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

    let mut profile = MavProfile::default();
    let mut field = MavField::default();
    let mut message = MavMessage::default();
    let mut mavenum = MavEnum::default();
    let mut entry = MavEnumEntry::default();
    let mut include = PathBuf::new();
    let mut paramid: Option<usize> = None;
    let mut deprecated: Option<MavDeprecation> = None;

    let mut xml_filter = MavXmlFilter::default();
    let mut events: Vec<Result<Event, quick_xml::Error>> = Vec::new();
    let file = File::open(&in_path).map_err(|e| BindGenError::CouldNotReadDefinitionFile {
        source: e,
        path: in_path.clone(),
    })?;
    let mut reader = Reader::from_reader(BufReader::new(file));
    reader.config_mut().trim_text(true);

    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => {
                events.push(Ok(Event::Eof));
                break;
            }
            Ok(event) => events.push(Ok(event.into_owned())),
            Err(why) => events.push(Err(why)),
        }
        buf.clear();
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
                    MavXmlElement::Include => {
                        include = PathBuf::default();
                    }
                    MavXmlElement::Param => {
                        paramid = None;
                    }
                    MavXmlElement::Deprecated => {
                        deprecated = Some(MavDeprecation {
                            replaced_by: String::new(),
                            since: String::new(),
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
                                    entry.value = u32::from_str_radix(src, radix).ok();
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
                            if attr.key.into_inner() == b"index" {
                                paramid =
                                    Some(String::from_utf8_lossy(&attr.value).parse().unwrap());
                            }
                        }
                        Some(&MavXmlElement::Deprecated) => match attr.key.into_inner() {
                            b"since" => {
                                deprecated.as_mut().unwrap().since =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                            b"replaced_by" => {
                                deprecated.as_mut().unwrap().replaced_by =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                            _ => (),
                        },
                        _ => (),
                    }
                }
            }
            Ok(Event::Empty(bytes)) => match bytes.name().into_inner() {
                b"extensions" => {
                    is_in_extension = true;
                }
                b"entry" => {
                    if mavenum.entries.is_empty() {
                        mavenum.deprecated = deprecated;
                    }
                    deprecated = None;
                    entry = MavEnumEntry::default();
                    for attr in bytes.attributes() {
                        let attr = attr.unwrap();
                        match attr.key.into_inner() {
                            b"name" => {
                                entry.name = String::from_utf8_lossy(&attr.value).to_string();
                            }
                            b"value" => {
                                entry.value =
                                    Some(String::from_utf8_lossy(&attr.value).parse().unwrap());
                            }
                            _ => (),
                        }
                    }
                    mavenum.entries.push(entry.clone());
                }
                b"deprecated" => {
                    deprecated = Some(MavDeprecation {
                        since: String::new(),
                        replaced_by: String::new(),
                        note: None,
                    });
                    for attr in bytes.attributes() {
                        let attr = attr.unwrap();
                        match attr.key.into_inner() {
                            b"since" => {
                                deprecated.as_mut().unwrap().since =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                            b"replaced_by" => {
                                deprecated.as_mut().unwrap().replaced_by =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                            _ => (),
                        }
                    }
                }
                b"field" => {
                    let mut field = MavField {
                        is_extension: is_in_extension,
                        ..Default::default()
                    };
                    for attr in bytes.attributes() {
                        let attr = attr.unwrap();
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
                                if let Some(e) = profile.enums.get(field.enumtype.as_ref().unwrap())
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
                    message.fields.push(field);
                }
                _ => (),
            },
            Ok(Event::Text(bytes)) => {
                let s = String::from_utf8_lossy(&bytes).to_string();

                use self::MavXmlElement::*;
                match (stack.last(), stack.get(stack.len() - 2)) {
                    (Some(&Description), Some(&Message)) => {
                        message.description = Some(s.replace('\n', " "));
                    }
                    (Some(&Field), Some(&Message)) => {
                        field.description = Some(s.replace('\n', " "));
                    }
                    (Some(&Description), Some(&Enum)) => {
                        mavenum.description = Some(s.replace('\n', " "));
                    }
                    (Some(&Description), Some(&Entry)) => {
                        entry.description = Some(s.replace('\n', " "));
                    }
                    (Some(&Param), Some(&Entry)) => {
                        if let Some(params) = entry.params.as_mut() {
                            // Some messages can jump between values, like:
                            // 0, 1, 2, 7
                            let paramid = paramid.unwrap();
                            if params.len() < paramid {
                                for index in params.len()..paramid {
                                    params.insert(index, String::from("The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0)."));
                                }
                            }
                            params[paramid - 1] = s;
                        }
                    }
                    (Some(&Include), Some(&Mavlink)) => {
                        include = PathBuf::from(s.replace('\n', ""));
                    }
                    (Some(&Version), Some(&Mavlink)) => {
                        profile.version =
                            Some(s.parse().expect("Invalid minor version number format"));
                    }
                    (Some(&Dialect), Some(&Mavlink)) => {
                        profile.dialect = Some(s.parse().expect("Invalid dialect number format"));
                    }
                    (Some(Deprecated), _) => {
                        deprecated.as_mut().unwrap().note = Some(s);
                    }
                    data => {
                        panic!("unexpected text data {data:?} reading {s:?}");
                    }
                }
            }
            Ok(Event::End(_)) => {
                match stack.last() {
                    Some(&MavXmlElement::Field) => message.fields.push(field.clone()),
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
                    _ => (),
                }
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
}
