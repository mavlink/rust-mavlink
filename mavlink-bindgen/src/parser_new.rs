use std::{
    collections::{BTreeMap, HashSet},
    path::{Path, PathBuf},
};

use quick_xml::{
    events::{BytesStart, Event},
    name::QName,
    Reader,
};

use crate::{
    error::MavXMLParseError,
    parser::{
        MavDeprecation, MavDeprecationType, MavEnum, MavEnumEntry, MavField, MavMessage, MavParam,
        MavProfile, MavType,
    },
    BindGenError,
};

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

const fn id_element(name: QName<'_>) -> Option<MavXmlElement> {
    use self::MavXmlElement::*;
    match name.into_inner() {
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

#[cfg(not(feature = "mav2-message-extensions"))]
struct ExtensionFilter {
    pub is_in: bool,
}

struct MessageFilter {
    pub is_in: bool,
    pub messages: Vec<String>,
}

struct MavXmlFilter {
    #[cfg(not(feature = "mav2-message-extensions"))]
    extension_filter: ExtensionFilter,
    message_filter: MessageFilter,
}

impl Default for MavXmlFilter {
    fn default() -> Self {
        Self {
            #[cfg(not(feature = "mav2-message-extensions"))]
            extension_filter: ExtensionFilter { is_in: false },
            message_filter: MessageFilter::new(),
        }
    }
}

impl MessageFilter {
    fn new() -> Self {
        Self {
            is_in: false,
            messages: vec![
                // device_cap_flags is u32, when enum is u16, which is not handled by the parser yet
                "STORM32_GIMBAL_MANAGER_INFORMATION".to_string(),
            ],
        }
    }
}

impl MavXmlFilter {
    fn filter(&mut self, event: &Event) -> bool {
        self.filter_extension(event)
            && self.filter_messages(event)
            && Self::filter_event_type(event)
    }

    #[cfg(feature = "mav2-message-extensions")]
    fn filter_extension(&mut self, _event: &Event) -> bool {
        true
    }

    /// Ignore extension fields
    #[cfg(not(feature = "mav2-message-extensions"))]
    fn filter_extension(&mut self, event: &Event) -> bool {
        match event {
            Event::Start(bytes) if id_element(bytes.name()) == Some(MavXmlElement::Extensions) => {
                self.extension_filter.is_in = true;
            }
            Event::End(bytes) if id_element(bytes.name()) == Some(MavXmlElement::Message) => {
                self.extension_filter.is_in = false;
            }
            _ => {}
        }
        !self.extension_filter.is_in
    }

    /// Filters messages by their name
    fn filter_messages(&mut self, event: &Event) -> bool {
        match event {
            Event::Start(bytes) if Some(MavXmlElement::Message) == id_element(bytes.name()) => {
                let name_attr = bytes
                    .attributes()
                    .filter_map(|attr| attr.ok())
                    .find(|attr| attr.key.into_inner() == b"name");
                if let Some(name_attr) = name_attr {
                    let value = String::from_utf8_lossy(&name_attr.value);
                    if self
                        .message_filter
                        .messages
                        .iter()
                        .any(|filter| filter.as_str() == value)
                    {
                        self.message_filter.is_in = true;
                        return false;
                    }
                }
            }
            Event::End(bytes)
                if Some(MavXmlElement::Message) == id_element(bytes.name())
                    && self.message_filter.is_in =>
            {
                self.message_filter.is_in = false;
                return false;
            }
            _ => {}
        }
        !self.message_filter.is_in
    }

    fn filter_event_type(event: &Event) -> bool {
        !matches!(
            event,
            Event::Comment(_) | Event::Decl(_) | Event::CData(_) | Event::DocType(_) | Event::PI(_)
        )
    }
}

struct MavCompositeProfile {
    profile: MavProfile,
    includes: Vec<String>,
}

impl MavCompositeProfile {
    fn resolve_includes(
        mut self,
        definitions_dir: &Path,
        parsed_files: &mut HashSet<PathBuf>,
    ) -> Result<MavProfile, BindGenError> {
        for include in &self.includes {
            let include = PathBuf::from(include.trim());
            let include_file = Path::new(&definitions_dir).join(include.clone());
            if !parsed_files.contains(&include_file) {
                let included_profile = parse_profile(definitions_dir, &include, parsed_files)?;
                for message in included_profile.messages.values() {
                    self.profile.add_message(message);
                }
                for enm in included_profile.enums.values() {
                    self.profile.add_enum(enm);
                }
                if self.profile.version.is_none() {
                    self.profile.version = included_profile.version;
                }
            }
        }
        Ok(self.profile)
    }
}

impl MavProfile {
    /// Go over all fields in the messages, and if you encounter an enum,
    /// which is a bitmask, set the bitmask size based on field size
    fn update_bitmask_enum_fields(mut self, definition_file: &Path) -> Result<Self, BindGenError> {
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
                                if entry.value.unwrap_or_default() > field.mavtype.max_int_value() {
                                    return Err(BindGenError::BitflagEnumTypeToNarrow {
                                        field_name: field.name.clone(),
                                        message_name: msg.name.clone(),
                                        enum_name: enum_name.to_string(),
                                    });
                                }
                            }

                            // Fix fields in backwards manner
                            if field.display.is_none() {
                                field.display = Some("bitmask".to_string());
                            }
                        }
                    } else {
                        return Err(BindGenError::InvalidEnumReference {
                            enum_name: enum_name.to_string(),
                            path: definition_file.to_path_buf(),
                        });
                    }
                }
            }
        }
        Ok(self)
    }
}

pub fn parse_profile(
    definitions_dir: &Path,
    definition_file: &Path,
    parsed_files: &mut HashSet<PathBuf>,
) -> Result<MavProfile, BindGenError> {
    let is_top_level = parsed_files.is_empty();
    let in_path = Path::new(&definitions_dir).join(definition_file);
    // Keep track of which files have been parsed
    parsed_files.insert(in_path.clone());

    let xml = std::fs::read_to_string(&in_path).map_err(|e| {
        BindGenError::CouldNotReadDefinitionFile {
            source: e,
            path: in_path.clone(),
        }
    })?;

    let mut reader = Reader::from_str(&xml);

    reader.config_mut().trim_text(true);
    reader.config_mut().expand_empty_elements = true;

    let mut parser = Parser::from_reader(reader);

    let profile = parser
        .parse()
        .map_err(|source| BindGenError::InvalidDefinitionFile {
            source,
            path: definition_file.to_path_buf(),
        })?;

    let profile = profile.resolve_includes(definitions_dir, parsed_files)?;

    if is_top_level {
        Ok(profile.update_bitmask_enum_fields(definition_file)?)
    } else {
        Ok(profile)
    }
}

struct Parser<'a> {
    reader: Reader<&'a [u8]>,
    filter: MavXmlFilter,
}

impl<'a> Parser<'a> {
    fn from_reader(reader: Reader<&'a [u8]>) -> Self {
        let filter = MavXmlFilter::default();
        Parser { reader, filter }
    }

    fn next_event(&mut self) -> Result<Event<'a>, MavXMLParseError> {
        loop {
            match self.reader.read_event()? {
                Event::Eof => return Err(MavXMLParseError::UnexpectedEnd),
                ev => {
                    if self.filter.filter(&ev) {
                        return Ok(ev);
                    }
                }
            }
        }
    }

    fn parse(&mut self) -> Result<MavCompositeProfile, MavXMLParseError> {
        match self.next_event()? {
            Event::Start(bytes_start)
                if id_element(bytes_start.name()) == Some(MavXmlElement::Mavlink) =>
            {
                self.parse_mavlink()
            }
            _ => Err(MavXMLParseError::ExpectedMAVLink),
        }
    }

    fn parse_mavlink(&mut self) -> Result<MavCompositeProfile, MavXMLParseError> {
        let mut profile = MavProfile::default();
        let mut includes = vec![];
        loop {
            match self.next_event()? {
                Event::Start(bytes_start) => match id_element(bytes_start.name()) {
                    Some(MavXmlElement::Include) => {
                        includes.push(self.parse_string_element(MavXmlElement::Include)?);
                    }
                    Some(MavXmlElement::Version) => {
                        profile.version = Some(self.parse_int_element(MavXmlElement::Version)?);
                    }
                    Some(MavXmlElement::Dialect) => {
                        profile.dialect = Some(self.parse_int_element(MavXmlElement::Dialect)?);
                    }
                    Some(MavXmlElement::Enums) => {
                        profile.enums = self.parse_enums()?;
                    }
                    Some(MavXmlElement::Messages) => profile.messages = self.parse_messages()?,
                    None => {}
                    Some(element) => return Err(MavXMLParseError::UnexpectedElement { element }),
                },
                Event::End(bytes_end)
                    if id_element(bytes_end.name()) == Some(MavXmlElement::Mavlink) =>
                {
                    break;
                }
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: MavXmlElement::Mavlink,
                    })
                }
            }
        }
        Ok(MavCompositeProfile { profile, includes })
    }

    fn parse_string_option_element(
        &mut self,
        element: MavXmlElement,
    ) -> Result<Option<String>, MavXMLParseError> {
        let mut string = None;

        loop {
            match self.next_event()? {
                Event::Text(bytes_text) => {
                    let text = String::from_utf8_lossy(&bytes_text).replace('\n', " ");
                    string = Some(
                        string
                            .map(|t| format!("{t}{text}"))
                            .unwrap_or(text.to_string()),
                    );
                }
                Event::GeneralRef(bytes_ref) => {
                    let entity = String::from_utf8_lossy(&bytes_ref);
                    string = Some(
                        string
                            .map(|t| format!("{t}&{entity};"))
                            .unwrap_or(format!("&{entity};")),
                    );
                }
                Event::End(bytes_end) if id_element(bytes_end.name()) == Some(element) => break,
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: element,
                    })
                }
            }
        }

        Ok(string)
    }

    fn parse_string_element(&mut self, element: MavXmlElement) -> Result<String, MavXMLParseError> {
        self.parse_string_option_element(element)?
            .ok_or(MavXMLParseError::ExpectedText)
    }

    fn parse_int_element(&mut self, element: MavXmlElement) -> Result<u8, MavXMLParseError> {
        Ok(self.parse_string_element(element)?.parse()?)
    }

    fn parse_deprecated(
        &mut self,
        bytes: &BytesStart<'_>,
        kind: MavDeprecationType,
    ) -> Result<MavDeprecation, MavXMLParseError> {
        let mut deprecation = MavDeprecation {
            deprecation_type: kind,
            ..Default::default()
        };

        for attr in bytes.attributes().filter_map(|attr| attr.ok()) {
            match attr.key.into_inner() {
                b"since" => {
                    deprecation.since = String::from_utf8_lossy(&attr.value).to_string();
                }
                b"replaced_by" => {
                    let value = String::from_utf8_lossy(&attr.value);
                    deprecation.replaced_by = if value.is_empty() {
                        None
                    } else {
                        Some(value.to_string())
                    };
                }
                _ => (),
            }
        }

        let element = if kind == MavDeprecationType::Superseded {
            MavXmlElement::Superseded
        } else {
            MavXmlElement::Deprecated
        };

        deprecation.note = self.parse_string_option_element(element)?;

        Ok(deprecation)
    }

    fn parse_wip(&mut self) -> Result<(), MavXMLParseError> {
        match self.next_event()? {
            Event::End(bytes_end) if id_element(bytes_end.name()) == Some(MavXmlElement::Wip) => {}
            ev => {
                return Err(MavXMLParseError::UnexpectedEvent {
                    event: ev.into_owned(),
                    parent: MavXmlElement::Wip,
                })
            }
        }
        Ok(())
    }

    fn parse_enums(&mut self) -> Result<BTreeMap<String, MavEnum>, MavXMLParseError> {
        let mut enums = BTreeMap::new();
        loop {
            match self.next_event()? {
                Event::Start(bytes_start)
                    if matches!(id_element(bytes_start.name()), Some(MavXmlElement::Enum)) =>
                {
                    let mav_enum = self.parse_enum(bytes_start)?;
                    enums.insert(mav_enum.name.clone(), mav_enum);
                }
                Event::End(bytes_end)
                    if matches!(id_element(bytes_end.name()), Some(MavXmlElement::Enums)) =>
                {
                    break;
                }
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: MavXmlElement::Enums,
                    })
                }
            }
        }
        Ok(enums)
    }

    fn parse_enum(&mut self, bytes: BytesStart<'_>) -> Result<MavEnum, MavXMLParseError> {
        let mut mav_enum = MavEnum::default();

        for attr in bytes.attributes().filter_map(|attr| attr.ok()) {
            match attr.key.into_inner() {
                b"name" => {
                    mav_enum.name = to_pascal_case(&attr.value);
                }
                b"bitmask" => mav_enum.bitmask = true,
                _ => {}
            }
        }

        loop {
            match self.next_event()? {
                ref ev @ Event::Start(ref bytes_start) => match id_element(bytes_start.name()) {
                    Some(MavXmlElement::Entry) => {
                        mav_enum.entries.push(self.parse_entry(bytes_start)?);
                    }
                    Some(MavXmlElement::Description) => {
                        mav_enum.description =
                            Some(self.parse_string_element(MavXmlElement::Description)?);
                    }
                    Some(MavXmlElement::Deprecated) => {
                        mav_enum.deprecated = Some(
                            self.parse_deprecated(bytes_start, MavDeprecationType::Deprecated)?,
                        );
                    }
                    Some(MavXmlElement::Superseded) => {
                        mav_enum.deprecated = Some(
                            self.parse_deprecated(bytes_start, MavDeprecationType::Superseded)?,
                        );
                    }
                    Some(MavXmlElement::Wip) => self.parse_wip()?,
                    Some(element) => return Err(MavXMLParseError::UnexpectedElement { element }),
                    None => {
                        return Err(MavXMLParseError::UnexpectedEvent {
                            event: ev.clone().into_owned(),
                            parent: MavXmlElement::Enum,
                        })
                    }
                },
                Event::End(bytes_end)
                    if id_element(bytes_end.name()) == Some(MavXmlElement::Enum) =>
                {
                    break
                }
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: MavXmlElement::Enum,
                    })
                }
            }
        }

        Ok(mav_enum)
    }

    fn parse_entry(&mut self, bytes: &BytesStart<'_>) -> Result<MavEnumEntry, MavXMLParseError> {
        let mut entry = MavEnumEntry::default();

        for attr in bytes.attributes().filter_map(|attr| attr.ok()) {
            match attr.key.into_inner() {
                b"name" => entry.name = String::from_utf8_lossy(&attr.value).to_string(),
                b"value" => {
                    let value = String::from_utf8_lossy(&attr.value);
                    let (src, radix) = value
                        .strip_prefix("0x")
                        .map(|value| (value, 16))
                        .unwrap_or((value.as_ref(), 10));
                    entry.value = Some(u64::from_str_radix(src, radix)?);
                }
                _ => {}
            }
        }

        loop {
            match self.next_event()? {
                ref ev @ Event::Start(ref bytes_start) => {
                    match id_element(bytes_start.name()) {
                        Some(MavXmlElement::Description) => {
                            entry.description =
                                self.parse_string_option_element(MavXmlElement::Description)?;
                        }
                        Some(MavXmlElement::Deprecated) => {
                            entry.deprecated = Some(
                                self.parse_deprecated(bytes_start, MavDeprecationType::Deprecated)?,
                            );
                        }
                        Some(MavXmlElement::Superseded) => {
                            entry.deprecated = Some(
                                self.parse_deprecated(bytes_start, MavDeprecationType::Superseded)?,
                            );
                        }
                        Some(MavXmlElement::Param) => {
                            let param = self.parse_param(bytes_start)?;
                            let param_index = param.index;

                            // Some messages can jump between values, like: 1, 2, 7
                            let params = entry.params.get_or_insert(Vec::with_capacity(7));
                            while params.len() < param_index {
                                params.push(MavParam {
                                    index: params.len() + 1,
                                    ..Default::default()
                                });
                            }
                            params[param_index - 1] = param;
                        }
                        Some(MavXmlElement::Wip) => _ = Some(self.parse_wip()?),
                        Some(element) => {
                            return Err(MavXMLParseError::UnexpectedElement { element })
                        }
                        None => {
                            return Err(MavXMLParseError::UnexpectedEvent {
                                event: ev.clone().into_owned(),
                                parent: MavXmlElement::Entry,
                            })
                        }
                    }
                }
                Event::End(bytes_end)
                    if Some(MavXmlElement::Entry) == id_element(bytes_end.name()) =>
                {
                    break
                }
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: MavXmlElement::Entry,
                    })
                }
            }
        }
        Ok(entry)
    }

    fn parse_param(&mut self, bytes: &BytesStart<'_>) -> Result<MavParam, MavXMLParseError> {
        let mut param = MavParam::default();

        for attr in bytes.attributes().filter_map(|attr| attr.ok()) {
            match attr.key.into_inner() {
                b"index" => {
                    let value = String::from_utf8_lossy(&attr.value).parse()?;
                    param.index = value;
                }
                b"label" => {
                    param.label = std::str::from_utf8(&attr.value).ok().map(str::to_owned);
                }
                b"increment" => {
                    param.increment = Some(String::from_utf8_lossy(&attr.value).parse()?);
                }
                b"minValue" => {
                    param.min_value = Some(String::from_utf8_lossy(&attr.value).parse()?);
                }
                b"maxValue" => {
                    param.max_value = Some(String::from_utf8_lossy(&attr.value).parse()?);
                }
                b"units" => {
                    param.units = std::str::from_utf8(&attr.value).ok().map(str::to_owned);
                }
                b"enum" => {
                    param.enum_used = std::str::from_utf8(&attr.value).ok().map(to_pascal_case);
                }
                b"reserved" => {
                    param.reserved = attr.value.as_ref() == b"true";
                }
                b"default" => {
                    param.default = Some(String::from_utf8_lossy(&attr.value).parse()?);
                }
                _ => (),
            }
        }

        if !(1..=7).contains(&param.index) {
            return Err(MavXMLParseError::ParamIndexOutOfRange);
        }

        param.description = self.parse_string_option_element(MavXmlElement::Param)?;
        Ok(param)
    }

    fn parse_messages(&mut self) -> Result<BTreeMap<String, MavMessage>, MavXMLParseError> {
        let mut messages = BTreeMap::new();
        loop {
            match self.next_event()? {
                Event::Start(bytes_start)
                    if matches!(id_element(bytes_start.name()), Some(MavXmlElement::Message)) =>
                {
                    let msg = self.parse_message(bytes_start)?;
                    messages.insert(msg.name.clone(), msg);
                }
                Event::End(bytes_end)
                    if matches!(id_element(bytes_end.name()), Some(MavXmlElement::Messages)) =>
                {
                    break;
                }
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: MavXmlElement::Messages,
                    })
                }
            }
        }
        Ok(messages)
    }

    fn parse_message(&mut self, bytes: BytesStart<'_>) -> Result<MavMessage, MavXMLParseError> {
        let mut message = MavMessage::default();
        for attr in bytes.attributes().filter_map(|attr| attr.ok()) {
            match attr.key.into_inner() {
                b"name" => message.name = String::from_utf8_lossy(&attr.value).to_string(),
                b"id" => message.id = String::from_utf8_lossy(&attr.value).parse()?,
                _ => {}
            }
        }

        let mut extention = false;

        loop {
            match self.next_event()? {
                ref ev @ Event::Start(ref bytes_start) => match id_element(bytes_start.name()) {
                    Some(MavXmlElement::Description) => {
                        message.description =
                            Some(self.parse_string_element(MavXmlElement::Description)?);
                    }
                    Some(MavXmlElement::Field) => message
                        .fields
                        .push(self.parse_field(bytes_start, extention)?),
                    Some(MavXmlElement::Extensions) => match self.next_event()? {
                        Event::End(bytes_end)
                            if id_element(bytes_end.name()) == Some(MavXmlElement::Extensions) =>
                        {
                            extention = true;
                        }
                        _ => return Err(MavXMLParseError::NoneSelfClosingExtTag),
                    },
                    Some(MavXmlElement::Deprecated) => {
                        message.deprecated = Some(
                            self.parse_deprecated(bytes_start, MavDeprecationType::Deprecated)?,
                        );
                    }
                    Some(MavXmlElement::Wip) => self.parse_wip()?,
                    Some(MavXmlElement::Superseded) => {
                        message.deprecated = Some(
                            self.parse_deprecated(bytes_start, MavDeprecationType::Superseded)?,
                        );
                    }
                    Some(element) => return Err(MavXMLParseError::UnexpectedElement { element }),
                    None => {
                        return Err(MavXMLParseError::UnexpectedEvent {
                            event: ev.clone().into_owned(),
                            parent: MavXmlElement::Message,
                        })
                    }
                },
                Event::End(bytes_end)
                    if id_element(bytes_end.name()) == Some(MavXmlElement::Message) =>
                {
                    break;
                }
                ev => {
                    return Err(MavXMLParseError::UnexpectedEvent {
                        event: ev.into_owned(),
                        parent: MavXmlElement::Message,
                    })
                }
            }
        }

        // Validate there are no duplicate field names
        message.validate_unique_fields()?;
        // Validate field count must be between 1 and 64
        message.validate_field_count()?;

        // Sort fields, first all none extension fields by type, then all extension fields in original order
        message
            .fields
            .sort_by(|a, b| match (a.is_extension, b.is_extension) {
                (false, false) => a.mavtype.compare(&b.mavtype),
                (a, b) => a.cmp(&b),
            });

        Ok(message)
    }

    fn parse_field(
        &mut self,
        bytes: &BytesStart<'_>,
        is_extension: bool,
    ) -> Result<MavField, MavXMLParseError> {
        let mut field = MavField {
            is_extension,
            ..Default::default()
        };
        for attr in bytes.attributes().filter_map(|attr| attr.ok()) {
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
                    let mav_type = String::from_utf8_lossy(&attr.value);
                    field.mavtype =
                        MavType::parse_type(&mav_type).ok_or(MavXMLParseError::InvalidType {
                            mav_type: mav_type.to_string(),
                        })?;
                }
                b"enum" => {
                    let enum_type = to_pascal_case(&attr.value);
                    field.enumtype = Some(enum_type);
                }
                b"display" => {
                    field.display = Some(String::from_utf8_lossy(&attr.value).to_string());
                }
                _ => {}
            }
        }

        field.description = self.parse_string_option_element(MavXmlElement::Field)?;

        Ok(field)
    }
}

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
