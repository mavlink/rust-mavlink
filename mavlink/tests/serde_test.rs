mod test_shared;

#[cfg(feature = "serde")]
mod serde_test {
    use serde_test::{assert_tokens, Token};

    /// Tests both serialization and deserialization of enum, bitflag and integer fields
    #[cfg(feature = "common")]
    #[test]
    fn test_ser_de_heartbeat() {
        use mavlink::common::{MavMessage, HEARTBEAT_DATA};
        let heartbeat_message = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        assert_tokens(
            &heartbeat_message,
            &[
                Token::Struct {
                    name: "HEARTBEAT_DATA",
                    len: 7,
                },
                Token::Str("type"),
                Token::Str("HEARTBEAT"),
                // u32 field
                Token::Str("custom_mode"),
                Token::U32(0),
                // enum field
                Token::Str("mavtype"),
                Token::Struct {
                    name: "MavType",
                    len: 1,
                },
                Token::Str("type"),
                Token::Str("MAV_TYPE_GENERIC"),
                Token::StructEnd,
                // enum field
                Token::Str("autopilot"),
                Token::Struct {
                    name: "MavAutopilot",
                    len: 1,
                },
                Token::Str("type"),
                Token::Str("MAV_AUTOPILOT_GENERIC"),
                Token::StructEnd,
                // bitflags field
                Token::Str("base_mode"),
                Token::Struct {
                    name: "MavModeFlag",
                    len: 1,
                },
                Token::Str("bits"),
                Token::U8(128),
                Token::StructEnd,
                // enum field
                Token::Str("system_status"),
                Token::Struct {
                    name: "MavState",
                    len: 1,
                },
                Token::Str("type"),
                Token::Str("MAV_STATE_UNINIT"),
                Token::StructEnd,
                // u8 field
                Token::Str("mavlink_version"),
                Token::U8(0),
                Token::StructEnd,
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
            s: arrayvec::ArrayString::from("rustmavlin").unwrap(), // 10 chars
        });
        assert_tokens(
            &test_message,
            &[
                Token::Struct {
                    name: "TEST_TYPES_DATA",
                    len: 23,
                },
                Token::Str("type"),
                Token::Str("TEST_TYPES"),
                Token::Str("u64"),
                Token::U64(0),
                Token::Str("s64"),
                Token::I64(-1),
                Token::Str("d"),
                Token::F64(f64::MAX),
                Token::Str("u64_array"),
                Token::Tuple { len: 3 },
                Token::U64(0),
                Token::U64(1),
                Token::U64(u64::MAX),
                Token::TupleEnd,
                Token::Str("s64_array"),
                Token::Tuple { len: 3 },
                Token::I64(i64::MIN),
                Token::I64(0),
                Token::I64(i64::MAX),
                Token::TupleEnd,
                Token::Str("d_array"),
                Token::Tuple { len: 3 },
                Token::F64(f64::INFINITY),
                Token::F64(0.0),
                Token::F64(f64::MIN_POSITIVE),
                Token::TupleEnd,
                Token::Str("u32"),
                Token::U32(0),
                Token::Str("s32"),
                Token::I32(-1),
                Token::Str("f"),
                Token::F32(f32::EPSILON),
                Token::Str("u32_array"),
                Token::Tuple { len: 3 },
                Token::U32(0),
                Token::U32(1),
                Token::U32(u32::MAX),
                Token::TupleEnd,
                Token::Str("s32_array"),
                Token::Tuple { len: 3 },
                Token::I32(i32::MIN),
                Token::I32(0),
                Token::I32(i32::MAX),
                Token::TupleEnd,
                Token::Str("f_array"),
                Token::Tuple { len: 3 },
                Token::F32(f32::NEG_INFINITY),
                Token::F32(0.0),
                Token::F32(f32::MIN),
                Token::TupleEnd,
                Token::Str("u16"),
                Token::U16(0),
                Token::Str("s16"),
                Token::I16(-1),
                Token::Str("u16_array"),
                Token::Tuple { len: 3 },
                Token::U16(0),
                Token::U16(1),
                Token::U16(u16::MAX),
                Token::TupleEnd,
                Token::Str("s16_array"),
                Token::Tuple { len: 3 },
                Token::I16(i16::MIN),
                Token::I16(0),
                Token::I16(i16::MAX),
                Token::TupleEnd,
                Token::Str("c"),
                Token::U8(b'R'),
                Token::Str("s"),
                Token::Str("rustmavlin"),
                Token::Str("u8"),
                Token::U8(0),
                Token::Str("s8"),
                Token::I8(-1),
                Token::Str("u8_array"),
                Token::Tuple { len: 3 },
                Token::U8(0),
                Token::U8(1),
                Token::U8(u8::MAX),
                Token::TupleEnd,
                Token::Str("s8_array"),
                Token::Tuple { len: 3 },
                Token::I8(i8::MIN),
                Token::I8(0),
                Token::I8(i8::MAX),
                Token::TupleEnd,
                Token::StructEnd,
            ],
        );
    }
}
