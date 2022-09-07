use crc_any::CRCu16;
use std::cmp::Ordering;
use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet};
use std::default::Default;
use std::fs::File;
use std::io::{BufReader, Write};
use std::path::{Path, PathBuf};
use std::str::FromStr;
use std::u32;

use quick_xml::{events::Event, Reader};

use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavProfile {
    pub messages: HashMap<String, MavMessage>,
    pub enums: HashMap<String, MavEnum>,
}

impl MavProfile {
    fn add_message(&mut self, message: &MavMessage) {
        match self.messages.entry(message.name.clone()) {
            Entry::Occupied(entry) => {
                if entry.get() != message {
                    panic!(
                        "Message '{}' defined twice but definitions are different",
                        message.name
                    );
                }
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
    /// update this enum with information about whether it is a bitmask, and what
    /// is the desired width of such.
    fn update_enums(mut self) -> Self {
        for msg in self.messages.values() {
            for field in &msg.fields {
                if let Some(ref enum_name) = field.enumtype {
                    // it is an enum
                    if let Some(ref dsp) = field.display {
                        // it is a bitmask
                        if dsp == "bitmask" {
                            // find the corresponding enum
                            for mut enm in self.enums.values_mut() {
                                if enm.name == *enum_name {
                                    // this is the right enum
                                    enm.bitfield = Some(field.mavtype.rust_type());
                                }
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
    fn emit_comments(&self) -> TokenStream {
        quote!(#![doc = "This file was automatically generated, do not edit"])
    }

    /// Emit rust messages
    fn emit_msgs(&self) -> Vec<TokenStream> {
        self.messages
            .iter()
            .map(|(_, d)| d.emit_rust())
            .collect::<Vec<TokenStream>>()
    }

    /// Emit rust enums
    fn emit_enums(&self) -> Vec<TokenStream> {
        self.enums
            .iter()
            .map(|(_, d)| d.emit_rust())
            .collect::<Vec<TokenStream>>()
    }

    /// Get list of original message names
    fn emit_enum_names(&self) -> Vec<TokenStream> {
        self.messages
            .iter()
            .map(|(_, msg)| {
                let name = format_ident!("{}", msg.name);
                quote!(#name)
            })
            .collect::<Vec<TokenStream>>()
    }

    /// Emit message names with "_DATA" at the end
    fn emit_struct_names(&self) -> Vec<TokenStream> {
        self.messages
            .iter()
            .map(|(_, msg)| msg.emit_struct_name())
            .collect::<Vec<TokenStream>>()
    }

    /// A list of message IDs
    fn emit_msg_ids(&self) -> Vec<TokenStream> {
        self.messages
            .iter()
            .map(|(_, msg)| {
                let msg_id = msg.id;
                quote!(#msg_id)
            })
            .collect::<Vec<TokenStream>>()
    }

    /// CRC values needed for mavlink parsing
    fn emit_msg_crc(&self) -> Vec<TokenStream> {
        self.messages
            .iter()
            .map(|(_, msg)| {
                let ec = extra_crc(msg);
                quote!(#ec)
            })
            .collect::<Vec<TokenStream>>()
    }

    fn emit_rust(&self) -> TokenStream {
        //TODO verify that id_width of u8 is OK even in mavlink v1
        let id_width = format_ident!("u32");

        let comment = self.emit_comments();
        let msgs = self.emit_msgs();
        let enum_names = self.emit_enum_names();
        let struct_names = self.emit_struct_names();
        let enums = self.emit_enums();
        let msg_ids = self.emit_msg_ids();
        let msg_crc = self.emit_msg_crc();

        let mav_message = self.emit_mav_message(&enum_names, &struct_names);
        let mav_message_parse = self.emit_mav_message_parse(&enum_names, &struct_names, &msg_ids);
        let mav_message_crc = self.emit_mav_message_crc(&id_width, &msg_ids, &msg_crc);
        let mav_message_name = self.emit_mav_message_name(&enum_names);
        let mav_message_id = self.emit_mav_message_id(&enum_names, &msg_ids);
        let mav_message_id_from_name = self.emit_mav_message_id_from_name(&enum_names, &msg_ids);
        let mav_message_default_from_id =
            self.emit_mav_message_default_from_id(&enum_names, &msg_ids);
        let mav_message_serialize = self.emit_mav_message_serialize(&enum_names);

        quote! {
            #comment
            use crate::MavlinkVersion;
            #[allow(unused_imports)]
            use bytes::{Buf, BufMut, Bytes, BytesMut};
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

            use crate::{Message, error::*};

            #[cfg(feature = "serde")]
            use serde::{Serialize, Deserialize};

            #[cfg(not(feature = "std"))]
            use alloc::vec::Vec;

            #[cfg(not(feature = "std"))]
            use alloc::string::ToString;

            #(#enums)*

            #(#msgs)*

            #[derive(Clone, PartialEq, Debug)]
            #mav_message

            impl Message for MavMessage {
                #mav_message_parse
                #mav_message_name
                #mav_message_id
                #mav_message_id_from_name
                #mav_message_default_from_id
                #mav_message_serialize
                #mav_message_crc
            }
        }
    }

    fn emit_mav_message(
        &self,
        enums: &Vec<TokenStream>,
        structs: &Vec<TokenStream>,
    ) -> TokenStream {
        quote! {
            #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
            #[cfg_attr(feature = "serde", serde(tag = "type"))]
            pub enum MavMessage {
                #(#enums(#structs),)*
            }
        }
    }

    fn emit_mav_message_parse(
        &self,
        enums: &[TokenStream],
        structs: &[TokenStream],
        ids: &[TokenStream],
    ) -> TokenStream {
        let id_width = format_ident!("u32");

        quote! {
            fn parse(version: MavlinkVersion, id: #id_width, payload: &[u8]) -> Result<MavMessage, ParserError> {
                match id {
                    #(#ids => #structs::deser(version, payload).map(|s| MavMessage::#enums(s)),)*
                    _ => {
                        Err(ParserError::UnknownMessage { id })
                    },
                }
            }
        }
    }

    fn emit_mav_message_crc(
        &self,
        id_width: &Ident,
        ids: &[TokenStream],
        crc: &[TokenStream],
    ) -> TokenStream {
        quote! {
            fn extra_crc(id: #id_width) -> u8 {
                match id {
                    #(#ids => #crc,)*
                    _ => {
                        0
                    },
                }
            }
        }
    }

    fn emit_mav_message_name(&self, enums: &Vec<TokenStream>) -> TokenStream {
        let enum_names = enums
            .iter()
            .map(|enum_name| {
                let name = enum_name.to_string();
                quote!(#name)
            })
            .collect::<Vec<TokenStream>>();

        quote! {
            fn message_name(&self) -> &'static str {
                match self {
                    #(MavMessage::#enums(..) => #enum_names,)*
                }
            }
        }
    }

    fn emit_mav_message_id(&self, enums: &Vec<TokenStream>, ids: &Vec<TokenStream>) -> TokenStream {
        let id_width = format_ident!("u32");
        quote! {
            fn message_id(&self) -> #id_width {
                match self {
                    #(MavMessage::#enums(..) => #ids,)*
                }
            }
        }
    }

    fn emit_mav_message_id_from_name(
        &self,
        enums: &[TokenStream],
        ids: &[TokenStream],
    ) -> TokenStream {
        let enum_names = enums
            .iter()
            .map(|enum_name| {
                let name = enum_name.to_string();
                quote!(#name)
            })
            .collect::<Vec<TokenStream>>();

        quote! {
            fn message_id_from_name(name: &str) -> Result<u32, &'static str> {
                match name {
                    #(#enum_names => Ok(#ids),)*
                    _ => {
                        Err("Invalid message name.")
                    }
                }
            }
        }
    }

    fn emit_mav_message_default_from_id(
        &self,
        enums: &[TokenStream],
        ids: &[TokenStream],
    ) -> TokenStream {
        let data_name = enums
            .iter()
            .map(|enum_name| {
                let name = format_ident!("{}_DATA", enum_name.to_string());
                quote!(#name)
            })
            .collect::<Vec<TokenStream>>();

        quote! {
            fn default_message_from_id(id: u32) -> Result<MavMessage, &'static str> {
                match id {
                    #(#ids => Ok(MavMessage::#enums(#data_name::default())),)*
                    _ => {
                        return Err("Invalid message id.");
                    }
                }
            }
        }
    }

    fn emit_mav_message_serialize(&self, enums: &Vec<TokenStream>) -> TokenStream {
        quote! {
            fn ser(&self, version: MavlinkVersion) -> Vec<u8> {
                match self {
                    #(&MavMessage::#enums(ref body) => body.ser(version),)*
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
    /// If contains Some, the string represents the type witdh for bitflags
    pub bitfield: Option<String>,
}

impl MavEnum {
    fn try_combine(&mut self, enm: &MavEnum) {
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

    fn has_enum_values(&self) -> bool {
        self.entries.iter().all(|x| x.value.is_some())
    }

    fn emit_defs(&self) -> Vec<TokenStream> {
        let mut cnt = 0isize;
        self.entries
            .iter()
            .map(|enum_entry| {
                let name = format_ident!("{}", enum_entry.name.clone());
                let value;

                #[cfg(feature = "emit-description")]
                let description = if let Some(description) = enum_entry.description.as_ref() {
                    quote!(#[doc = #description])
                } else {
                    quote!()
                };

                #[cfg(not(feature = "emit-description"))]
                let description = quote!();

                if !self.has_enum_values() {
                    value = quote!(#cnt);
                    cnt += 1;
                } else {
                    let tmp =
                        TokenStream::from_str(&enum_entry.value.unwrap().to_string()).unwrap();
                    value = quote!(#tmp);
                };
                if self.bitfield.is_some() {
                    quote! {
                        #description
                        const #name = #value;
                    }
                } else {
                    quote! {
                        #description
                        #name = #value,
                    }
                }
            })
            .collect::<Vec<TokenStream>>()
    }

    fn emit_name(&self) -> TokenStream {
        let name = format_ident!("{}", self.name);
        quote!(#name)
    }

    fn emit_rust(&self) -> TokenStream {
        let defs = self.emit_defs();
        let default = format_ident!("{}", self.entries[0].name);
        let enum_name = self.emit_name();

        #[cfg(feature = "emit-description")]
        let description = if let Some(description) = self.description.as_ref() {
            let desc = format!("{}", description);
            quote!(#[doc = #desc])
        } else {
            quote!()
        };

        #[cfg(not(feature = "emit-description"))]
        let description = quote!();

        let enum_def;
        if let Some(width) = self.bitfield.clone() {
            let width = format_ident!("{}", width);
            enum_def = quote! {
                bitflags!{
                    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
                    #description
                    pub struct #enum_name: #width {
                        #(#defs)*
                    }
                }
            };
        } else {
            enum_def = quote! {
                #[derive(Debug, Copy, Clone, PartialEq, FromPrimitive, ToPrimitive)]
                #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
                #[cfg_attr(feature = "serde", serde(tag = "type"))]
                #description
                pub enum #enum_name {
                    #(#defs)*
                }
            };
        }

        dbg!(quote! {
            #enum_def

            impl Default for #enum_name {
                fn default() -> Self {
                    #enum_name::#default
                }
            }
        })
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavEnumEntry {
    pub value: Option<u32>,
    pub name: String,
    pub description: Option<String>,
    pub params: Option<Vec<String>>,
}

#[derive(Debug, PartialEq, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MavMessage {
    pub id: u32,
    pub name: String,
    pub description: Option<String>,
    pub fields: Vec<MavField>,
}

impl MavMessage {
    /// Return Token of "MESSAGE_NAME_DATA
    /// for mavlink struct data
    fn emit_struct_name(&self) -> TokenStream {
        let name = format_ident!("{}", format!("{}_DATA", self.name));
        quote!(#name)
    }

    fn emit_name_types(&self) -> (Vec<TokenStream>, usize) {
        let mut encoded_payload_len: usize = 0;
        let field_toks = self
            .fields
            .iter()
            .map(|field| {
                let nametype = field.emit_name_type();
                encoded_payload_len += field.mavtype.len();

                #[cfg(feature = "emit-description")]
                let description = field.emit_description();

                #[cfg(not(feature = "emit-description"))]
                let description = quote!();

                // From MAVLink specification:
                // If sent by an implementation that doesn't have the extensions fields
                // then the recipient will see zero values for the extensions fields.
                let serde_default = if field.is_extension {
                    quote!(#[cfg_attr(feature = "serde", serde(default))])
                } else {
                    quote!()
                };

                quote! {
                    #description
                    #serde_default
                    #nametype
                }
            })
            .collect::<Vec<TokenStream>>();
        (field_toks, encoded_payload_len)
    }

    /// Generate description for the given message
    #[cfg(feature = "emit-description")]
    fn emit_description(&self) -> TokenStream {
        let mut ts = TokenStream::new();
        let desc = format!("id: {}", self.id);
        ts.extend(quote!(#[doc = #desc]));
        if let Some(val) = self.description.clone() {
            let doc = &format!("{}.", val);
            ts.extend(quote!(#[doc = #doc]));
        }
        ts
    }

    fn emit_serialize_vars(&self) -> TokenStream {
        let ser_vars = self
            .fields
            .iter()
            .map(|f| f.rust_writer())
            .collect::<Vec<TokenStream>>();
        quote! {
            let mut _tmp = Vec::new();
            #(#ser_vars)*
            if matches!(version, MavlinkVersion::V2) {
                crate::remove_trailing_zeroes(&mut _tmp);
            }
            _tmp
        }
    }

    fn emit_deserialize_vars(&self) -> TokenStream {
        let deser_vars = self
            .fields
            .iter()
            .map(|f| f.rust_reader())
            .collect::<Vec<TokenStream>>();

        let i = format_ident!("{}_DATA", self.name);
        let encoded_len_name = quote!(#i::ENCODED_LEN);

        if deser_vars.is_empty() {
            // struct has no fields
            quote! {
                Ok(Self::default())
            }
        } else {
            quote! {
                let avail_len = _input.len();

                // fast zero copy
                let mut buf = BytesMut::from(_input);

                // handle payload length truncuation due to empty fields
                if avail_len < #encoded_len_name {
                    //copy available bytes into an oversized buffer filled with zeros
                    let mut payload_buf  = [0; #encoded_len_name];
                    payload_buf[0..avail_len].copy_from_slice(_input);
                    buf = BytesMut::from(&payload_buf[..]);
                }

                let mut _struct = Self::default();
                #(#deser_vars)*
                Ok(_struct)
            }
        }
    }

    fn emit_rust(&self) -> TokenStream {
        let msg_name = self.emit_struct_name();
        let (name_types, msg_encoded_len) = self.emit_name_types();

        let deser_vars = self.emit_deserialize_vars();
        let serialize_vars = self.emit_serialize_vars();

        #[cfg(feature = "emit-description")]
        let description = self.emit_description();

        #[cfg(not(feature = "emit-description"))]
        let description = quote!();

        quote! {
            #description
            #[derive(Debug, Clone, PartialEq, Default)]
            #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
            pub struct #msg_name {
                #(#name_types)*
            }

            impl #msg_name {
                pub const ENCODED_LEN: usize = #msg_encoded_len;

                pub fn deser(version: MavlinkVersion, _input: &[u8]) -> Result<Self, ParserError> {
                    #deser_vars
                }

                pub fn ser(&self, version: MavlinkVersion) -> Vec<u8> {
                    #serialize_vars
                }
            }
        }
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
    fn emit_name(&self) -> TokenStream {
        let name = format_ident!("{}", self.name);
        quote!(#name)
    }

    /// Emit rust type of the field
    fn emit_type(&self) -> TokenStream {
        let mavtype;
        match self.mavtype {
            MavType::Array(_, _) => {
                let rt = TokenStream::from_str(&self.mavtype.rust_type()).unwrap();
                mavtype = quote!(#rt);
            }
            _ => match self.enumtype {
                Some(ref enumname) => {
                    let en = TokenStream::from_str(enumname).unwrap();
                    mavtype = quote!(#en);
                }
                _ => {
                    let rt = TokenStream::from_str(&self.mavtype.rust_type()).unwrap();
                    mavtype = quote!(#rt);
                }
            },
        }
        mavtype
    }

    /// Generate description for the given field
    #[cfg(feature = "emit-description")]
    fn emit_description(&self) -> TokenStream {
        let mut ts = TokenStream::new();
        if let Some(val) = self.description.clone() {
            let desc = format!("{}.", val);
            ts.extend(quote!(#[doc = #desc]));
        }
        ts
    }

    /// Combine rust name and type of a given field
    fn emit_name_type(&self) -> TokenStream {
        let name = self.emit_name();
        let fieldtype = self.emit_type();
        quote!(pub #name: #fieldtype,)
    }

    /// Emit writer
    fn rust_writer(&self) -> TokenStream {
        let mut name = "self.".to_string() + &self.name.clone();
        if self.enumtype.is_some() {
            if let Some(dsp) = &self.display {
                // potentially a bitflag
                if dsp == "bitmask" {
                    // it is a bitflag
                    name += ".bits()";
                } else {
                    panic!("Display option not implemented");
                }
            } else {
                match self.mavtype {
                    MavType::Array(_, _) => {} // cast are not necessary for arrays
                    _ => {
                        // an enum, have to use "*foo as u8" cast
                        name += " as ";
                        name += &self.mavtype.rust_type();
                    }
                }
            }
        }
        let ts = TokenStream::from_str(&name).unwrap();
        let name = quote!(#ts);
        let buf = format_ident!("_tmp");
        self.mavtype.rust_writer(name, buf)
    }

    /// Emit reader
    fn rust_reader(&self) -> TokenStream {
        let _name = TokenStream::from_str(&self.name).unwrap();

        let name = quote!(_struct.#_name);
        let buf = format_ident!("buf");
        if let Some(enum_name) = &self.enumtype {
            if let Some(dsp) = &self.display {
                if dsp == "bitmask" {
                    // bitflags
                    let tmp = self.mavtype.rust_reader(quote!(let tmp), buf);
                    let enum_name_ident = format_ident!("{}", enum_name);
                    quote! {
                        #tmp
                        #name = #enum_name_ident::from_bits(tmp & #enum_name_ident::all().bits())
                            .ok_or(ParserError::InvalidFlag { flag_type: #enum_name.to_string(), value: tmp as u32 })?;
                    }
                } else {
                    panic!("Display option not implemented");
                }
            } else {
                if let MavType::Array(_t, _size) = &self.mavtype {
                    return self.mavtype.rust_reader(name, buf);
                }
                // handle enum by FromPrimitive
                let tmp = self.mavtype.rust_reader(quote!(let tmp), buf);
                let val = format_ident!("from_{}", &self.mavtype.rust_type());
                quote!(
                    #tmp
                    #name = FromPrimitive::#val(tmp)
                        .ok_or(ParserError::InvalidEnum { enum_type: #enum_name.to_string(), value: tmp as u32 })?;
                )
            }
        } else {
            self.mavtype.rust_reader(name, buf)
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MavType {
    UInt8MavlinkVersion,
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
    Array(Box<MavType>, usize),
}

impl Default for MavType {
    fn default() -> MavType {
        MavType::UInt8
    }
}

impl MavType {
    fn parse_type(s: &str) -> Option<MavType> {
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
            _ => {
                if s.ends_with(']') {
                    let start = s.find('[')?;
                    let size = s[start + 1..(s.len() - 1)].parse::<usize>().ok()?;
                    let mtype = MavType::parse_type(&s[0..start])?;
                    Some(Array(Box::new(mtype), size))
                } else {
                    None
                }
            }
        }
    }

    /// Emit reader of a given type
    pub fn rust_reader(&self, val: TokenStream, buf: Ident) -> TokenStream {
        use self::MavType::*;
        match self.clone() {
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
            Array(t, size) => {
                if size > 32 {
                    // it is a vector
                    let r = t.rust_reader(quote!(let val), buf);
                    quote! {
                        for _ in 0..#size {
                            #r
                            #val.push(val);
                        }
                    }
                } else {
                    // handle as a slice
                    let r = t.rust_reader(quote!(let val), buf);
                    quote! {
                        for idx in 0..#size {
                            #r
                            #val[idx] = val;
                        }
                    }
                }
            }
        }
    }

    /// Emit writer of a given type
    pub fn rust_writer(&self, val: TokenStream, buf: Ident) -> TokenStream {
        use self::MavType::*;
        match self.clone() {
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
            Array(t, _size) => {
                let w = t.rust_writer(quote!(*val), buf);
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
        match self.clone() {
            UInt8MavlinkVersion | UInt8 | Int8 | Char => 1,
            UInt16 | Int16 => 2,
            UInt32 | Int32 | Float => 4,
            UInt64 | Int64 | Double => 8,
            Array(t, size) => t.len() * size,
        }
    }

    /// Used for ordering of types
    fn order_len(&self) -> usize {
        use self::MavType::*;
        match self.clone() {
            UInt8MavlinkVersion | UInt8 | Int8 | Char => 1,
            UInt16 | Int16 => 2,
            UInt32 | Int32 | Float => 4,
            UInt64 | Int64 | Double => 8,
            Array(t, _) => t.len(),
        }
    }

    /// Used for crc calculation
    pub fn primitive_type(&self) -> String {
        use self::MavType::*;
        match self.clone() {
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
            Array(t, _) => t.primitive_type(),
        }
    }

    /// Return rust equivalent of a given Mavtype
    /// Used for generating struct fields.
    pub fn rust_type(&self) -> String {
        use self::MavType::*;
        match self.clone() {
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
            Array(t, size) => {
                if size > 32 {
                    // we have to use a vector to make our lives easier
                    format!("Vec<{}> /* {} elements */", t.rust_type(), size)
                } else {
                    // we can use a slice, as Rust derives lot of thinsg for slices <= 32 elements
                    format!("[{};{}]", t.rust_type(), size)
                }
            }
        }
    }

    /// Compare two MavTypes
    pub fn compare(&self, other: &Self) -> Ordering {
        let len = self.order_len();
        (-(len as isize)).cmp(&(-(other.order_len() as isize)))
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

fn identify_element(s: &[u8]) -> Option<MavXmlElement> {
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
    definition_file: &String,
    parsed_files: &mut HashSet<PathBuf>,
) -> MavProfile {
    let in_path = Path::new(&definitions_dir).join(&definition_file);
    parsed_files.insert(in_path.clone()); // Keep track of which files have been parsed

    let mut stack: Vec<MavXmlElement> = vec![];

    let mut profile = MavProfile::default();
    let mut field = MavField::default();
    let mut message = MavMessage::default();
    let mut mavenum = MavEnum::default();
    let mut entry = MavEnumEntry::default();
    let mut include = String::new();
    let mut paramid: Option<usize> = None;

    let mut xml_filter = MavXmlFilter::default();
    let mut events: Vec<Result<Event, quick_xml::Error>> = Vec::new();
    let mut reader = Reader::from_reader(BufReader::new(File::open(in_path).unwrap()));
    reader.trim_text(true);
    reader.trim_text_end(true);

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
                let id = match identify_element(&bytes.name().into_inner()) {
                    None => {
                        panic!(
                            "unexpected element {:?}",
                            String::from_utf8_lossy(bytes.name().into_inner())
                        );
                    }
                    Some(kind) => kind,
                };

                if !is_valid_parent(stack.last().copied(), id) {
                    panic!("not valid parent {:?} of {:?}", stack.last(), id);
                }

                match id {
                    MavXmlElement::Extensions => {
                        is_in_extension = true;
                    }
                    MavXmlElement::Message => {
                        message = Default::default();
                    }
                    MavXmlElement::Field => {
                        field = Default::default();
                        field.is_extension = is_in_extension;
                    }
                    MavXmlElement::Enum => {
                        mavenum = Default::default();
                    }
                    MavXmlElement::Entry => {
                        entry = Default::default();
                    }
                    MavXmlElement::Include => {
                        include = Default::default();
                    }
                    MavXmlElement::Param => {
                        paramid = None;
                    }
                    _ => (),
                }

                stack.push(id);

                for attr in bytes.attributes() {
                    let attr = attr.unwrap();
                    match stack.last() {
                        Some(&MavXmlElement::Enum) => {
                            if let b"name" = attr.key.into_inner() {
                                mavenum.name = attr
                                    .value
                                    .clone()
                                    .split(|b| *b == b'_')
                                    .map(|x| x.to_ascii_lowercase())
                                    .map(|mut v| {
                                        v[0] = v[0].to_ascii_uppercase();
                                        String::from_utf8(v).unwrap()
                                    })
                                    .collect::<Vec<String>>()
                                    .join("");
                                //mavenum.name = attr.value.clone();
                            }
                        }
                        Some(&MavXmlElement::Entry) => {
                            match attr.key.into_inner() {
                                b"name" => {
                                    let name = String::from_utf8(attr.value.to_vec()).unwrap();
                                    entry.name = name;
                                }
                                b"value" => {
                                    // Deal with hexadecimal numbers
                                    if attr.value.starts_with(b"0x") {
                                        entry.value = Some(
                                            u32::from_str_radix(
                                                std::str::from_utf8(&attr.value[2..]).unwrap(),
                                                16,
                                            )
                                            .unwrap(),
                                        );
                                    } else {
                                        let s = std::str::from_utf8(&attr.value[..]).unwrap();
                                        entry.value = Some(s.parse::<u32>().unwrap());
                                    }
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
                                    message.name = String::from_utf8(attr.value.to_vec()).unwrap();
                                }
                                b"id" => {
                                    let s = std::str::from_utf8(&attr.value).unwrap();
                                    message.id = s.parse::<u32>().unwrap();
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Field) => {
                            match attr.key.into_inner() {
                                b"name" => {
                                    let name = String::from_utf8(attr.value.to_vec()).unwrap();
                                    field.name = name;
                                    if field.name == "type" {
                                        field.name = "mavtype".to_string();
                                    }
                                }
                                b"type" => {
                                    let s = std::str::from_utf8(&attr.value).unwrap();
                                    field.mavtype = MavType::parse_type(s).unwrap();
                                }
                                b"enum" => {
                                    field.enumtype = Some(
                                        attr.value
                                            .clone()
                                            .split(|b| *b == b'_')
                                            .map(|x| x.to_ascii_lowercase())
                                            .map(|mut v| {
                                                v[0] = v[0].to_ascii_uppercase();
                                                String::from_utf8(v).unwrap()
                                            })
                                            .collect::<Vec<String>>()
                                            .join(""),
                                    );
                                    //field.enumtype = Some(attr.value.clone());
                                }
                                b"display" => {
                                    field.display =
                                        Some(String::from_utf8(attr.value.to_vec()).unwrap());
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Param) => {
                            if entry.params.is_none() {
                                entry.params = Some(vec![]);
                            }
                            if let b"index" = attr.key.into_inner() {
                                let s = std::str::from_utf8(&attr.value).unwrap();
                                paramid = Some(s.parse::<usize>().unwrap());
                            }
                        }
                        _ => (),
                    }
                }
            }
            Ok(Event::Empty(bytes)) => match bytes.name().into_inner() {
                b"extensions" => {
                    is_in_extension = true;
                }
                b"entry" => {
                    entry = Default::default();
                    for attr in bytes.attributes() {
                        let attr = attr.unwrap();
                        match attr.key.into_inner() {
                            b"name" => {
                                entry.name = String::from_utf8(attr.value.to_vec()).unwrap();
                            }
                            b"value" => {
                                let s = std::str::from_utf8(&attr.value).unwrap();
                                entry.value = Some(s.parse().unwrap());
                            }
                            _ => (),
                        }
                    }
                    mavenum.entries.push(entry.clone());
                }
                _ => (),
            },
            Ok(Event::Text(bytes)) => {
                let s = String::from_utf8(bytes.to_vec()).unwrap();

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
                        if let Some(ref mut params) = entry.params {
                            // Some messages can jump between values, like:
                            // 0, 1, 2, 7
                            if params.len() < paramid.unwrap() {
                                for index in params.len()..paramid.unwrap() {
                                    params.insert(index, String::from("The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0)."));
                                }
                            }
                            params[paramid.unwrap() - 1] = s;
                        }
                    }
                    (Some(&Include), Some(&Mavlink)) => {
                        include = s.replace('\n', "");
                    }
                    (Some(&Version), Some(&Mavlink)) => {
                        eprintln!("TODO: version {:?}", s);
                    }
                    (Some(&Dialect), Some(&Mavlink)) => {
                        eprintln!("TODO: dialect {:?}", s);
                    }
                    (Some(Deprecated), _) => {
                        eprintln!("TODO: deprecated {:?}", s);
                    }
                    data => {
                        panic!("unexpected text data {:?} reading {:?}", data, s);
                    }
                }
            }
            Ok(Event::End(_)) => {
                match stack.last() {
                    Some(&MavXmlElement::Field) => message.fields.push(field.clone()),
                    Some(&MavXmlElement::Entry) => {
                        mavenum.entries.push(entry.clone());
                    }
                    Some(&MavXmlElement::Message) => {
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

                        profile.add_message(&msg);
                    }
                    Some(&MavXmlElement::Enum) => {
                        profile.add_enum(&mavenum);
                    }
                    Some(&MavXmlElement::Include) => {
                        let include_file = Path::new(&definitions_dir).join(include.clone());
                        if !parsed_files.contains(&include_file) {
                            let included_profile =
                                parse_profile(definitions_dir, &include, parsed_files);
                            for message in included_profile.messages.values() {
                                profile.add_message(message);
                            }
                            for enm in included_profile.enums.values() {
                                profile.add_enum(enm);
                            }
                        }
                    }
                    _ => (),
                }
                stack.pop();
                // println!("{}-{}", indent(depth), name);
            }
            Err(e) => {
                eprintln!("Error: {}", e);
                break;
            }
            _ => {}
        }
    }

    //let profile = profile.update_messages(); //TODO verify no longer needed
    profile.update_enums()
}

/// Generate protobuf represenation of mavlink message set
/// Generate rust representation of mavlink message set with appropriate conversion methods
pub fn generate<W: Write>(definitions_dir: &Path, definition_file: &String, output_rust: &mut W) {
    let mut parsed_files: HashSet<PathBuf> = HashSet::new();
    let profile = parse_profile(definitions_dir, definition_file, &mut parsed_files);

    // rust file
    let rust_tokens = profile.emit_rust();
    writeln!(output_rust, "{}", rust_tokens).unwrap();
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
    crc.digest(" ".as_bytes());

    let mut f = msg.fields.clone();
    // only mavlink 1 fields should be part of the extra_crc
    f.retain(|f| !f.is_extension);
    f.sort_by(|a, b| a.mavtype.compare(&b.mavtype));
    for field in &f {
        crc.digest(field.mavtype.primitive_type().as_bytes());
        crc.digest(" ".as_bytes());
        if field.name == "mavtype" {
            crc.digest("type".as_bytes());
        } else {
            crc.digest(field.name.as_bytes());
        }
        crc.digest(" ".as_bytes());
        if let MavType::Array(_, size) = field.mavtype {
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

struct MavXmlFilter {
    #[cfg(not(feature = "emit-extensions"))]
    extension_filter: ExtensionFilter,
}

impl Default for MavXmlFilter {
    fn default() -> MavXmlFilter {
        MavXmlFilter {
            #[cfg(not(feature = "emit-extensions"))]
            extension_filter: ExtensionFilter { is_in: false },
        }
    }
}

impl MavXmlFilter {
    pub fn filter(&mut self, elements: &mut Vec<Result<Event, quick_xml::Error>>) {
        // List of filters
        elements.retain(|x| self.filter_extension(x));
    }

    #[cfg(feature = "emit-extensions")]
    pub fn filter_extension(&mut self, _element: &Result<Event, quick_xml::Error>) -> bool {
        return true;
    }

    /// Ignore extension fields
    #[cfg(not(feature = "emit-extensions"))]
    pub fn filter_extension(&mut self, element: &Result<Event, quick_xml::Error>) -> bool {
        match element {
            Ok(content) => {
                match content {
                    Event::Start(bytes) => {
                        let id = match identify_element(bytes.name().into_inner()) {
                            None => {
                                panic!(
                                    "unexpected element {:?}",
                                    String::from_utf8_lossy(bytes.name().into_inner())
                                );
                            }
                            Some(kind) => kind,
                        };
                        if let MavXmlElement::Extensions = id {
                            self.extension_filter.is_in = true;
                        }
                    }
                    Event::Empty(bytes) => {
                        let id = match identify_element(bytes.name().into_inner()) {
                            None => {
                                panic!(
                                    "unexpected element {:?}",
                                    String::from_utf8_lossy(bytes.name().into_inner())
                                );
                            }
                            Some(kind) => kind,
                        };
                        if let MavXmlElement::Extensions = id {
                            self.extension_filter.is_in = true;
                        }
                    }
                    Event::End(bytes) => {
                        let id = match identify_element(bytes.name().into_inner()) {
                            None => {
                                panic!(
                                    "unexpected element {:?}",
                                    String::from_utf8_lossy(bytes.name().into_inner())
                                );
                            }
                            Some(kind) => kind,
                        };

                        if let MavXmlElement::Message = id {
                            self.extension_filter.is_in = false;
                        }
                    }
                    _ => {}
                }
                !self.extension_filter.is_in
            }
            Err(error) => panic!("Failed to filter XML: {}", error),
        }
    }
}
