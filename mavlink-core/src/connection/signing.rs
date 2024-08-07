use crate::MAVLinkV2MessageRaw;

use std::{collections::HashMap, sync::Mutex};

// signing configuration
pub struct SigningConfig {
    secret_key: [u8; 32],
    pub(crate) sign_outgoing: bool,
    allow_unsigned: bool,
}

// mutable state of signing per connection
pub(crate) struct SigningState {
    timestamp: u64,
    // does never really change but is definitely not part of the setup
    link_id: u8,
    stream_timestamps: HashMap<(u8, u8, u8), u64>, // TODO unsigned callback
}

pub struct SigningData {
    pub(crate) config: SigningConfig,
    pub(crate) state: Mutex<SigningState>,
}

impl SigningConfig {
    pub fn new(secret_key: [u8; 32], sign_outgoing: bool, allow_unsigned: bool) -> Self {
        SigningConfig {
            secret_key,
            sign_outgoing,
            allow_unsigned,
        }
    }
}

impl SigningData {
    pub fn from_config(config: SigningConfig) -> Self {
        Self {
            config,
            state: Mutex::new(SigningState {
                timestamp: 0,
                link_id: 0,
                stream_timestamps: HashMap::new(),
            }),
        }
    }

    pub(crate) fn verify_signature(&self, message: MAVLinkV2MessageRaw) -> bool {
        use crate::MAVLINK_IFLAG_SIGNED;
        // TODO: fix that unwrap poison
        let mut state = self.state.lock().unwrap();
        if message.incompatibility_flags() & MAVLINK_IFLAG_SIGNED > 0 {
            state.timestamp = u64::max(state.timestamp, Self::get_current_timestamp());
            let timestamp = message.signature_timestamp();
            let src_system = message.system_id();
            let src_component = message.component_id();
            let stream_key = (message.signature_link_id(), src_system, src_component);
            match state.stream_timestamps.get(&stream_key) {
                Some(stream_timestamp) => {
                    if timestamp <= *stream_timestamp {
                        // reject old timestamp
                        return false;
                    }
                }
                None => {
                    if timestamp + 60 * 1000 * 100 < state.timestamp {
                        // bad new stream, more then a minute older the the last one
                        return false;
                    }
                }
            }

            let mut signature_buffer = [0u8; 6];
            message.calculate_signature(&self.config.secret_key, &mut signature_buffer);
            let result = signature_buffer == message.signature_value();
            if result {
                state.stream_timestamps.insert(stream_key, timestamp);
                state.timestamp = u64::max(state.timestamp, timestamp)
            }
            result
        } else {
            self.config.allow_unsigned
        }
    }

    pub(crate) fn sign_message(&self, message: &mut MAVLinkV2MessageRaw) {
        // TODO: fix that unwrap poison
        let mut state = self.state.lock().unwrap();
        state.timestamp = u64::max(state.timestamp, Self::get_current_timestamp());
        let ts_bytes = u64::to_le_bytes(state.timestamp);
        message
            .signature_timestamp_bytes_mut()
            .copy_from_slice(&ts_bytes[0..6]);
        // TODO link id set
        *message.signature_link_id_mut() = state.link_id;

        let mut signature_buffer = [0u8; 6];
        message.calculate_signature(&self.config.secret_key, &mut signature_buffer);

        message
            .signature_value_mut()
            .copy_from_slice(&signature_buffer);
        state.timestamp += 1;
    }

    fn get_current_timestamp() -> u64 {
        use std::time::SystemTime;
        // fallback to 0 if the system time appears to be before epoch
        let now = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .map(|n| n.as_micros())
            .unwrap_or(0);
        // use 1st January 2015 GMT as offset, fallback to 0 if before that date, this will overflow in April 2104
        ((now
            .checked_sub(1420070400u128 * 1000000u128)
            .unwrap_or_default())
            / 10u128) as u64
    }
}
