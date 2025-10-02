mod test_shared;

#[cfg(feature = "serde")]
mod serde_test {
    use serde_test::{assert_tokens, Configure, Token::*};

    /// Test the serialization and deserialization of just a bitflag enum
    #[cfg(feature = "common")]
    #[test]
    fn test_bitflags() {
        use mavlink::common::MavModeFlag;

        let flags =
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED;

        assert_tokens(
            &flags.readable(),
            &[
                NewtypeStruct {
                    name: "MavModeFlag",
                },
                String("MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_GUIDED_ENABLED"),
            ],
        );

        assert_tokens(
            &flags.compact(),
            &[
                NewtypeStruct {
                    name: "MavModeFlag",
                },
                U8(flags.bits()),
            ],
        );
    }

    /// Tests both serialization and deserialization of enum, bitflag and integer fields
    #[cfg(feature = "common")]
    #[test]
    fn test_ser_de_heartbeat() {
        use mavlink::common::{MavMessage, HEARTBEAT_DATA};
        let heartbeat_message = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());

        assert_tokens(
            &heartbeat_message.readable(),
            &[
                Struct {
                    name: "HEARTBEAT_DATA",
                    len: 7,
                },
                Str("type"),
                Str("HEARTBEAT"),
                // u32 field
                Str("custom_mode"),
                U32(0),
                // enum field
                Str("mavtype"),
                Struct {
                    name: "MavType",
                    len: 1,
                },
                Str("type"),
                Str("MAV_TYPE_GENERIC"),
                StructEnd,
                // enum field
                Str("autopilot"),
                Struct {
                    name: "MavAutopilot",
                    len: 1,
                },
                Str("type"),
                Str("MAV_AUTOPILOT_GENERIC"),
                StructEnd,
                // bitflags field
                Str("base_mode"),
                NewtypeStruct {
                    name: "MavModeFlag",
                },
                Str("MAV_MODE_FLAG_SAFETY_ARMED"),
                // enum field
                Str("system_status"),
                Struct {
                    name: "MavState",
                    len: 1,
                },
                Str("type"),
                Str("MAV_STATE_UNINIT"),
                StructEnd,
                // u8 field
                Str("mavlink_version"),
                U8(3),
                StructEnd,
            ],
        );
    }

    /// Tests both serialization and deserialization of all none enum/bitflag types
    #[cfg(feature = "test")]
    #[test]
    fn test_ser_de_all_types() {
        use core::{f32, f64};
        use std::u64;

        use mavlink::test::{MavMessage, TEST_TYPES_DATA};
        let test_message = MavMessage::TEST_TYPES(TEST_TYPES_DATA {
            u64: 0,
            s64: -1,
            u64_array: [0, 1, u64::MAX],
            s64_array: [i64::MIN, 0, i64::MAX],
            u32: 0,
            s32: -1,
            u32_array: [0, 1, u32::MAX],
            s32_array: [i32::MIN, 0, i32::MAX],
            u16: 0,
            s16: -1,
            u16_array: [0, 1, u16::MAX],
            s16_array: [i16::MIN, 0, i16::MAX],
            u8: 0,
            s8: -1,
            u8_array: [0, 1, u8::MAX],
            s8_array: [i8::MIN, 0, i8::MAX],
            // Note: testing NaN does not work since the testing framework uses simple float cmp which uses Nan != Nan
            d: f64::MAX,
            d_array: [f64::INFINITY, 0.0, f64::MIN_POSITIVE],
            f: f32::EPSILON,
            f_array: [f32::NEG_INFINITY, 0.0, f32::MIN],
            c: b'R',
            s: *b"rustmavlin", // 10 chars
        });
        assert_tokens(
            &test_message,
            &[
                Struct {
                    name: "TEST_TYPES_DATA",
                    len: 23,
                },
                Str("type"),
                Str("TEST_TYPES"),
                Str("u64"),
                U64(0),
                Str("s64"),
                I64(-1),
                Str("d"),
                F64(f64::MAX),
                Str("u64_array"),
                Tuple { len: 3 },
                U64(0),
                U64(1),
                U64(u64::MAX),
                TupleEnd,
                Str("s64_array"),
                Tuple { len: 3 },
                I64(i64::MIN),
                I64(0),
                I64(i64::MAX),
                TupleEnd,
                Str("d_array"),
                Tuple { len: 3 },
                F64(f64::INFINITY),
                F64(0.0),
                F64(f64::MIN_POSITIVE),
                TupleEnd,
                Str("u32"),
                U32(0),
                Str("s32"),
                I32(-1),
                Str("f"),
                F32(f32::EPSILON),
                Str("u32_array"),
                Tuple { len: 3 },
                U32(0),
                U32(1),
                U32(u32::MAX),
                TupleEnd,
                Str("s32_array"),
                Tuple { len: 3 },
                I32(i32::MIN),
                I32(0),
                I32(i32::MAX),
                TupleEnd,
                Str("f_array"),
                Tuple { len: 3 },
                F32(f32::NEG_INFINITY),
                F32(0.0),
                F32(f32::MIN),
                TupleEnd,
                Str("u16"),
                U16(0),
                Str("s16"),
                I16(-1),
                Str("u16_array"),
                Tuple { len: 3 },
                U16(0),
                U16(1),
                U16(u16::MAX),
                TupleEnd,
                Str("s16_array"),
                Tuple { len: 3 },
                I16(i16::MIN),
                I16(0),
                I16(i16::MAX),
                TupleEnd,
                Str("c"),
                U8(b'R'),
                Str("s"),
                Tuple { len: 10 },
                U8(b'r'),
                U8(b'u'),
                U8(b's'),
                U8(b't'),
                U8(b'm'),
                U8(b'a'),
                U8(b'v'),
                U8(b'l'),
                U8(b'i'),
                U8(b'n'),
                TupleEnd,
                Str("u8"),
                U8(0),
                Str("s8"),
                I8(-1),
                Str("u8_array"),
                Tuple { len: 3 },
                U8(0),
                U8(1),
                U8(u8::MAX),
                TupleEnd,
                Str("s8_array"),
                Tuple { len: 3 },
                I8(i8::MIN),
                I8(0),
                I8(i8::MAX),
                TupleEnd,
                StructEnd,
            ],
        );
    }
}

mod serde_test_json {
    use mavlink::common;
    use serde_json::json;

    #[test]
    fn test_serde_output() {
        let json = serde_json::to_string(&common::MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: common::MavType::MAV_TYPE_GENERIC,
            autopilot: common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: common::MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
            system_status: common::MavState::MAV_STATE_UNINIT,
            mavlink_version: 3,
        }))
        .unwrap();
        let expected = json!({
            "type": "HEARTBEAT",
            "custom_mode": 0,
            "mavtype": { "type": "MAV_TYPE_GENERIC" },
            "autopilot": { "type": "MAV_AUTOPILOT_GENERIC" },
            "base_mode": "MAV_MODE_FLAG_SAFETY_ARMED",
            "system_status": { "type": "MAV_STATE_UNINIT" },
            "mavlink_version": 3
        })
        .to_string();
        assert_eq!(json, expected);

        let json = serde_json::to_string(&common::MavMessage::PARAM_REQUEST_READ(
            common::PARAM_REQUEST_READ_DATA {
                param_id: {
                    let mut buf = [0; 16];
                    let src = "TEST_PARAM".as_bytes();
                    for (i, c) in src.iter().enumerate() {
                        buf[i] = *c;
                    }
                    buf
                },
                target_system: 0,
                target_component: 0,
                param_index: 0,
            },
        ))
        .unwrap();
        let expected = json!({
            "type": "PARAM_REQUEST_READ",
            "param_index": 0,
            "target_system": 0,
            "target_component": 0,
            "param_id": [84, 69, 83, 84, 95, 80, 65, 82, 65, 77, 0, 0, 0, 0, 0, 0]
        })
        .to_string();
        assert_eq!(json, expected);
    }
}
