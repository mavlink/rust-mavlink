use crate::parser::MavEnumEntry;

pub fn get_custom_entries() -> Vec<MavEnumEntry> {
    vec![MavEnumEntry {
        value: Some(81),
        name: "CUSTOM_DRAGON_LMT".to_string(),
        description: Some("Custom mode for special operations".to_string()),
        params: None,
    }]
}
