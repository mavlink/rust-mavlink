use std::error::Error;
use std::f32::consts::PI;
use std::{env, sync::Arc, thread, time::Duration};

use rppal::pwm::{Channel, Polarity, Pwm};
use mavlink::error::MessageReadError;
use mavlink::common::MavMessage;
use mavlink;

// Servo configuration. The datasheet was wrong for mine so these are manually calibrated.
// Your configuration may be different.
const PERIOD_MS: u64 = 20;
const PULSE_MIN_US: u64 = 640;
const PULSE_NEUTRAL_US: u64 = 1540;
const PULSE_MAX_US: u64 = 2540;

fn radian_to_servo_pulse(radians: f32) -> u64 {
    let percentage = radians / PI;
    let distance_from_neutral = percentage * (PULSE_MAX_US - PULSE_MIN_US) as f32;
    let pulse_width = PULSE_NEUTRAL_US as f32 + distance_from_neutral;
    if pulse_width < PULSE_MIN_US as f32 {
        PULSE_MIN_US
    } else if pulse_width > PULSE_MAX_US as f32 {
        PULSE_MAX_US
    } else {
        pulse_width as u64
    }
}

fn mavlink_dump() {
    let serial_address = "serial:/dev/ttyAMA0:57600".to_string();
    let mavlink = mavlink::connect::<MavMessage>(&serial_address).unwrap();
    let pwm = Pwm::with_period(
        Channel::Pwm0,
        Duration::from_millis(PERIOD_MS),
        Duration::from_micros(PULSE_NEUTRAL_US),
        Polarity::Normal,
        true,
    ).unwrap();
    loop {
        match mavlink.recv() {
            Ok(msg) => {
                println!("{:?}", msg);
                match msg.1 {
                    MavMessage::HEARTBEAT(heartbeat) => {
                        println!("Heartbeat: {:?}", heartbeat);
                    }
                    MavMessage::SYS_STATUS(sys_status) => {
                        println!("SysStatus: {:?}", sys_status);
                    }
                    MavMessage::SYSTEM_TIME(system_time) => {
                        println!("SystemTime: {:?}", system_time);
                    }
                    MavMessage::PING(ping) => {
                        println!("Ping: {:?}", ping);
                    }
                    MavMessage::CHANGE_OPERATOR_CONTROL(change_operator_control) => {
                        println!("ChangeOperatorControl: {:?}", change_operator_control);
                    }
                    MavMessage::CHANGE_OPERATOR_CONTROL_ACK(change_operator_control_ack) => {
                        println!("ChangeOperatorControlAck: {:?}", change_operator_control_ack);
                    }
                    MavMessage::AUTH_KEY(auth_key) => {
                        println!("AuthKey: {:?}", auth_key);
                    }
                    MavMessage::SET_MODE(set_mode) => {
                        println!("SetMode: {:?}", set_mode);
                    }
                    MavMessage::PARAM_REQUEST_READ(param_request_read) => {
                        println!("ParamRequestRead: {:?}", param_request_read);
                    }
                    MavMessage::PARAM_REQUEST_LIST(param_request_list) => {
                        println!("ParamRequestList: {:?}", param_request_list);
                    }
                    MavMessage::PARAM_VALUE(param_value) => {
                        println!("ParamValue: {:?}", param_value);
                    }
                    MavMessage::PARAM_SET(param_set) => {
                        println!("ParamSet: {:?}", param_set);
                    }
                    MavMessage::GPS_RAW_INT(gps_raw_int) => {
                        println!("GpsRawInt: {:?}", gps_raw_int);
                    }
                    MavMessage::GPS_STATUS(gps_status) => {
                        println!("GpsStatus: {:?}", gps_status);
                    }
                    MavMessage::SCALED_IMU(scaled_imu) => {
                        println!("ScaledImu: {:?}", scaled_imu);
                    }
                    MavMessage::RAW_IMU(raw_imu) => {
                        println!("RawImu: {:?}", raw_imu);
                    }
                    MavMessage::RAW_PRESSURE(raw_pressure) => {
                        println!("RawPressure: {:?}", raw_pressure);
                    }
                    MavMessage::SCALED_PRESSURE(scaled_pressure) => {
                        println!("ScaledPressure: {:?}", scaled_pressure);
                    }
                    MavMessage::ATTITUDE(attitude) => {
                        println!("Attitude: {:?}", attitude);
                        let pulse_width = radian_to_servo_pulse(attitude.roll);
                        pwm.set_pulse_width(Duration::from_micros(pulse_width)).unwrap();
                    }
                    _ => {
                        println!("Unhandled message: {:?}", msg);
                    }                    
                }
            }
            Err(MessageReadError::Io(e)) => {
                println!("IO error: {}", e);
                break;
            }
            _ => {
                println!("Error reading message");
            }
        }
    }
}

fn main() -> () {
    mavlink_dump();
}
