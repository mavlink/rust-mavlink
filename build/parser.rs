use std::default::Default;
use std::cmp::Ordering;
use std::io::{Write, Read};
use crc16;

use xml::reader::{EventReader, XmlEvent};

#[derive(Debug, PartialEq, Clone)]
pub struct MavEnum {
    pub name: String,
    pub description: Option<String>,
    pub entries: Vec<MavEnumEntry>,
}

impl Default for MavEnum {
    fn default() -> MavEnum {
        MavEnum {
            name: "".into(),
            description: None,
            entries: vec![],
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct MavEnumEntry {
    pub value: i32,
    pub name: String,
    pub description: Option<String>,
    pub params: Option<Vec<String>>,
}

impl Default for MavEnumEntry {
    fn default() -> MavEnumEntry {
        MavEnumEntry {
            value: 0,
            name: "".into(),
            description: None,
            params: None,
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct MavMessage {
    pub id: u8,
    pub name: String,
    pub description: Option<String>,
    pub fields: Vec<MavField>,
}

impl Default for MavMessage {
    fn default() -> MavMessage {
        MavMessage {
            id: 0,
            name: "".into(),
            description: None,
            fields: vec![],
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
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

fn parse_type(s: &str) -> Option<MavType> {
    use parser::MavType::*;
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
        _ => {
            if s.ends_with("]") {
                let start = s.find("[").unwrap();
                let size = s[start + 1..(s.len() - 1)].parse::<usize>().unwrap();
                let mtype = parse_type(&s[0..start]).unwrap();
                Some(Array(Box::new(mtype), size))
            } else {
                panic!("UNHANDLED {:?}", s);
            }
        }
    }
}

impl MavType {
    fn len(&self) -> usize {
        use parser::MavType::*;
        match self.clone() {
            UInt8MavlinkVersion | UInt8 | Int8 | Char => 1,
            UInt16 | Int16 => 2,
            UInt32 | Int32 | Float => 4,
            UInt64 | Int64 | Double => 8,
            Array(t, size) => t.len() * size,
        }
    }

    fn order_len(&self) -> usize {
        use parser::MavType::*;
        match self.clone() {
            UInt8MavlinkVersion | UInt8 | Int8 | Char => 1,
            UInt16 | Int16 => 2,
            UInt32 | Int32 | Float => 4,
            UInt64 | Int64 | Double => 8,
            Array(t, _) => t.len(),
        }
    }

    pub fn primitive_type(&self) -> String {
        use parser::MavType::*;
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

    pub fn rust_type(&self) -> String {
        use parser::MavType::*;
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
            // Buffer(n) => "u8".into(),
            Array(t, size) => format!("Vec<{}> /* {} */", t.rust_type(), size),
        }
    }

    pub fn compare(&self, other: &Self) -> Ordering {
        let len = self.order_len();
        (-(len as isize)).cmp(&(-(other.order_len() as isize)))
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct MavField {
    pub mavtype: MavType,
    pub name: String,
    pub description: Option<String>,
    pub enumtype: Option<String>,
}

impl Default for MavField {
    fn default() -> MavField {
        MavField {
            mavtype: MavType::UInt8,
            name: "".into(),
            description: None,
            enumtype: None,
        }
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum MavXmlElement {
    Version,
    Mavlink,
    Include,
    Enums,
    Enum,
    Entry,
    Description,
    Param,
    Messages,
    Message,
    Field,
}

fn identify_element(s: &str) -> Option<MavXmlElement> {
    use parser::MavXmlElement::*;
    match s {
        "version" => Some(Version),
        "mavlink" => Some(Mavlink),
        "include" => Some(Include),
        "enums" => Some(Enums),
        "enum" => Some(Enum),
        "entry" => Some(Entry),
        "description" => Some(Description),
        "param" => Some(Param),
        "messages" => Some(Messages),
        "message" => Some(Message),
        "field" => Some(Field),
        _ => None,
    }
}

fn is_valid_parent(p: Option<MavXmlElement>, s: MavXmlElement) -> bool {
    use parser::MavXmlElement::*;
    match s {
        Version => p == Some(Mavlink),
        Mavlink => p == None,
        Include => p == Some(Mavlink),
        Enums => p == Some(Mavlink),
        Enum => p == Some(Enums),
        Entry => p == Some(Enum),
        Description => p == Some(Entry) || p == Some(Message) || p == Some(Enum),
        Param => p == Some(Entry),
        Messages => p == Some(Mavlink),
        Message => p == Some(Messages),
        Field => p == Some(Message),
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct MavProfile {
    pub includes: Vec<String>,
    pub messages: Vec<MavMessage>,
    pub enums: Vec<MavEnum>,
}

pub fn parse_profile(file: &mut Read) -> MavProfile {
    let mut stack: Vec<MavXmlElement> = vec![];

    let mut profile = MavProfile {
        includes: vec![],
        messages: vec![],
        enums: vec![],
    };

    let mut field: MavField = Default::default();
    let mut message: MavMessage = Default::default();
    let mut mavenum: MavEnum = Default::default();
    let mut entry: MavEnumEntry = Default::default();
    let mut paramid: Option<usize> = None;

    let parser = EventReader::new(file);
    for e in parser {
        match e {
            Ok(XmlEvent::StartElement { name, attributes: attrs, .. }) => {
                let id = match identify_element(&name.to_string()) {
                    None => {
                        panic!("unexpected element {:?}", name);
                    }
                    Some(kind) => kind,
                };

                if !is_valid_parent(match stack.last().clone() {
                                        Some(arg) => Some(arg.clone()),
                                        None => None,
                                    },
                                    id.clone()) {
                    panic!("not valid parent {:?} of {:?}", stack.last(), id);
                }

                match id {
                    MavXmlElement::Message => {
                        message = Default::default();
                    }
                    MavXmlElement::Field => {
                        field = Default::default();
                    }
                    MavXmlElement::Enum => {
                        mavenum = Default::default();
                    }
                    MavXmlElement::Entry => {
                        entry = Default::default();
                    }
                    MavXmlElement::Param => {
                        paramid = None;
                    }
                    _ => (),
                }

                stack.push(id);

                for attr in attrs {
                    match stack.last() {
                        Some(&MavXmlElement::Enum) => {
                            match attr.name.local_name.clone().as_ref() {
                                "name" => {
                                    mavenum.name = attr.value.clone();
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Entry) => {
                            match attr.name.local_name.clone().as_ref() {
                                "name" => {
                                    entry.name = attr.value.clone();
                                }
                                "value" => {
                                    entry.value = attr.value.parse::<i32>().unwrap();
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Message) => {
                            match attr.name.local_name.clone().as_ref() {
                                "name" => {
                                    message.name = attr.value.clone();
                                }
                                "id" => {
                                    message.id = attr.value.parse::<u8>().unwrap();
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Field) => {
                            match attr.name.local_name.clone().as_ref() {
                                "name" => {
                                    field.name = attr.value.clone();
                                }
                                "type" => {
                                    field.mavtype = parse_type(&attr.value).unwrap();
                                }
                                "enum" => {
                                    field.enumtype = Some(attr.value.clone());
                                }
                                _ => (),
                            }
                        }
                        Some(&MavXmlElement::Param) => {
                            if let None = entry.params {
                                entry.params = Some(vec![]);
                            }
                            match attr.name.local_name.clone().as_ref() {
                                "index" => {
                                    paramid = Some(attr.value.parse::<usize>().unwrap());
                                }
                                _ => (),
                            }
                        }
                        _ => (),
                    }
                }
            }
            Ok(XmlEvent::Characters(s)) => {
                use parser::MavXmlElement::*;
                match (stack.last(), stack.get(stack.len() - 2)) {
                    (Some(&Description), Some(&Message)) => {
                        message.description = Some(s);
                    }
                    (Some(&Field), Some(&Message)) => {
                        field.description = Some(s);
                    }
                    (Some(&Description), Some(&Enum)) => {
                        mavenum.description = Some(s);
                    }
                    (Some(&Description), Some(&Entry)) => {
                        entry.description = Some(s);
                    }
                    (Some(&Param), Some(&Entry)) => {
                        if let Some(ref mut params) = entry.params {
                            params.insert(paramid.unwrap() - 1, s);
                        }
                    }
                    (Some(&Include), Some(&Mavlink)) => {
                        println!("TODO: include {:?}", s);
                    }
                    (Some(&Version), Some(&Mavlink)) => {
                        println!("TODO: version {:?}", s);
                    }
                    data => {
                        panic!("unexpected text data {:?} reading {:?}", data, s);
                    }
                }
            }
            Ok(XmlEvent::EndElement { .. }) => {
                match stack.last() {
                    Some(&MavXmlElement::Field) => message.fields.push(field.clone()),
                    Some(&MavXmlElement::Entry) => {
                        mavenum.entries.push(entry.clone());
                    }
                    Some(&MavXmlElement::Message) => {
                        // println!("message: {:?}", message);
                        profile.messages.push(message.clone());
                    }
                    Some(&MavXmlElement::Enum) => {
                        profile.enums.push(mavenum.clone());
                    }
                    _ => (),
                }
                stack.pop();
                // println!("{}-{}", indent(depth), name);
            }
            Err(e) => {
                println!("Error: {}", e);
                break;
            }
            _ => {}
        }
    }

    profile
}

#[test]
fn test_all() {
    let file = File::open("solo.xml").unwrap();
    let file = BufReader::new(file);

    let profile = parse_profile(Box::new(file));
    // println!("done: {:?}", profile);

    println!("message 150: {:?}", profile.messages.get(&150));
    println!("enum MAV_CMD: {:?}", profile.enums.get("MAV_CMD".into()));
}

pub fn extra_crc(msg: &MavMessage) -> u8 {
    // calculate a 8-bit checksum of the key fields of a message, so we
    // can detect incompatible XML changes
    let mut crc = crc16::State::<crc16::MCRF4XX>::new();
    crc.update(msg.name.as_bytes());
    crc.update(" ".as_bytes());

    let mut f = msg.fields.clone();
    f.sort_by(|a, b| a.mavtype.compare(&b.mavtype));
    for field in &f {
        crc.update(field.mavtype.primitive_type().as_bytes());
        crc.update(" ".as_bytes());
        crc.update(field.name.as_bytes());
        crc.update(" ".as_bytes());
        if let MavType::Array(_, size) = field.mavtype {
            crc.update(&[size as u8]);
        }
    }

    let crcval = crc.get();
    ((crcval & 0xFF) ^ (crcval >> 8)) as u8
}

#[allow(unused_must_use)] // TODO fix
pub fn generate_mod<R: Read, W: Write>(input: &mut R, output: &mut W) {
    let profile = parse_profile(input);

    // writeln!(output, "#![allow(non_camel_case_types)]");
    // writeln!(output, "#![allow(non_snake_case)]");
    // writeln!(output, "");

    writeln!(output, "use std::io::Cursor;");
    writeln!(output, "use byteorder::{{LittleEndian, ReadBytesExt, WriteBytesExt}};");
    writeln!(output, "");

    writeln!(output, "pub trait Parsable {{");
    writeln!(output, "    fn parse(payload: &[u8]) -> Self;");
    writeln!(output, "    fn serialize(&self) -> Vec<u8>;");
    writeln!(output, "}}");
    writeln!(output, "");

    for item in &profile.messages {
        let mut f = item.fields.clone();
        f.sort_by(|a, b| a.mavtype.compare(&b.mavtype));

        writeln!(output, "#[derive(Clone, Debug)]");
        writeln!(output, "pub struct {}_DATA {{", item.name);
        for field in &f {
            let fname = if field.name == "type" {
                "mavtype".into()
            } else {
                field.name.clone()
            };

            writeln!(output, "    pub {}: {},", fname, field.mavtype.rust_type());
        }
        writeln!(output, "}}");
        writeln!(output, "");

        writeln!(output, "impl Parsable for {}_DATA {{", item.name);
        writeln!(output, "    fn parse(payload: &[u8]) -> {}_DATA {{", item.name);
        writeln!(output, "        let mut cur = Cursor::new(payload);");
        writeln!(output, "        {}_DATA {{", item.name);
        for field in &f {
            let fname = if field.name == "type" {
                "mavtype".into()
            } else {
                field.name.clone()
            };
            match field.mavtype {
                MavType::Char | MavType::UInt8 | MavType::Int8 | MavType::UInt8MavlinkVersion => {
                    writeln!(output, "            {}: cur.read_{}().unwrap(),",
                             fname,
                             field.mavtype.rust_type());
                }
                MavType::Array(ref t, size) => {
                    writeln!(output, "            {}: vec![", fname);
                    for _ in 0..size {
                        match *t.clone() {
                            MavType::Char |
                            MavType::UInt8 |
                            MavType::Int8 |
                            MavType::UInt8MavlinkVersion => {
                                writeln!(output, "                cur.read_{}().unwrap(),", t.rust_type());
                            }
                            MavType::Array(_, _) => {
                                panic!("error");
                            }
                            _ => {
                                writeln!(output, "                cur.read_{}::<LittleEndian>().unwrap(),",
                                         t.rust_type());
                            }
                        }
                    }
                    writeln!(output, "            ],");
                }
                _ => {
                    writeln!(output, "            {}: cur.read_{}::<LittleEndian>().unwrap(),",
                             fname,
                             field.mavtype.rust_type());
                }
            }
        }
        writeln!(output, "        }}");
        writeln!(output, "    }}");
        writeln!(output, "    fn serialize(&self) -> Vec<u8> {{");
        writeln!(output, "        let mut wtr = vec![];");
        for field in &f {
            let fname = if field.name == "type" {
                "mavtype".into()
            } else {
                field.name.clone()
            };
            match field.mavtype {
                MavType::Char | MavType::UInt8 | MavType::Int8 | MavType::UInt8MavlinkVersion => {
                    writeln!(output, "        wtr.write_{}(self.{}).unwrap();",
                             field.mavtype.rust_type(),
                             fname);
                }
                MavType::Array(ref t, size) => {
                    for i in 0..size {
                        match *t.clone() {
                            MavType::Char |
                            MavType::UInt8 |
                            MavType::Int8 |
                            MavType::UInt8MavlinkVersion => {
                                writeln!(output, "        wtr.write_{}(self.{}[{}]).unwrap();",
                                         t.rust_type(),
                                         fname,
                                         i);
                            }
                            MavType::Array(_, _) => {
                                panic!("error");
                            }
                            _ => {
                                writeln!(output, "        wtr.write_{}::<LittleEndian>(self.{}[{}]).\
                                          unwrap();",
                                         t.rust_type(),
                                         fname,
                                         i);
                            }
                        }
                    }
                }
                _ => {
                    writeln!(output, "        wtr.write_{}::<LittleEndian>(self.{}).unwrap();",
                             field.mavtype.rust_type(),
                             fname);
                }
            }
        }
        writeln!(output, "        wtr");
        writeln!(output, "    }}");
        writeln!(output, "}}");
        writeln!(output, "");
    }

    writeln!(output, "#[derive(Clone, Debug)]");
    writeln!(output, "pub enum MavMessage {{");
    for item in &profile.messages {
        writeln!(output, "  {}({}_DATA),", item.name, item.name);
    }
    writeln!(output, "}}");
    writeln!(output, "");

    writeln!(output, "impl MavMessage {{");
    writeln!(output, "    pub fn parse(id: u8, payload: &[u8]) -> Option<MavMessage> {{");
    writeln!(output, "        match id {{");
    for item in &profile.messages {
        writeln!(output, "            {} => Some(MavMessage::{}({}_DATA::parse(payload))),",
                 item.id,
                 item.name,
                 item.name);
    }
    writeln!(output, "            _ => None,");
    writeln!(output, "        }}");
    writeln!(output, "    }}");
    writeln!(output, "");
    writeln!(output, "    pub fn message_id(&self) -> u8 {{");
    writeln!(output, "        match self {{");
    for item in &profile.messages {
        writeln!(output, "            &MavMessage::{}(..) => {},", item.name, item.id);
    }
    writeln!(output, "        }}");
    writeln!(output, "    }}");
    writeln!(output, "");
    writeln!(output, "    pub fn extra_crc(id: u8) -> u8 {{");
    writeln!(output, "        match id {{");
    for item in &profile.messages {
        writeln!(output, "            {} => {},", item.id, extra_crc(item));
    }
    writeln!(output, "            _ => 0,");
    writeln!(output, "        }}");
    writeln!(output, "    }}");
    writeln!(output, "");
    writeln!(output, "    pub fn serialize(&self) -> Vec<u8> {{");
    writeln!(output, "        match self {{");
    for item in &profile.messages {
        writeln!(output, "            &MavMessage::{}(ref body) => body.serialize(),",
                 item.name);
    }
    writeln!(output, "        }}");
    writeln!(output, "    }}");
    writeln!(output, "}}");
    writeln!(output, "");
}
