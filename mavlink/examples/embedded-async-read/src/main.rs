//! Target board: stm32f446RETx (stm32nucleo)
//! Manual: https://www.st.com/resource/en/reference_manual/dm00043574-stm32f303xb-c-d-e-stm32f303x6-8-stm32f328x8-stm32f358xc-stm32f398xe-advanced-arm-based-mcus-stmicroelectronics.pdf
#![no_main]
#![no_std]

// Panic handler
use panic_rtt_target as _;

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, mode::Async, peripherals::*, usart};
use embassy_time::Timer;
use mavlink;
use mavlink::common::{MavMessage, HEARTBEAT_DATA};
use mavlink::{read_v2_raw_message_async, MAVLinkV2MessageRaw, MavlinkVersion, MessageData};
use rtt_target::{rprintln, rtt_init_print};
use static_cell::ConstStaticCell;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_print!();

    // Peripherals access
    let p = embassy_stm32::init(embassy_stm32::Config::default());

    // Create an interface USART2 with 115200 baudrate
    let mut config = usart::Config::default();
    config.baudrate = 115200;
    let serial = usart::Uart::new(
        p.USART1, p.PA10, p.PA9, Irqs, p.DMA2_CH7, p.DMA2_CH2, config,
    )
    .unwrap();

    // Break serial in TX and RX (not used)
    let (mut tx, rx) = serial.split();

    // Create our mavlink header and heartbeat message
    let header = mavlink::MavHeader {
        incompat_flags: 0,
        system_id: 1,
        component_id: 1,
        sequence: 42,
    };
    let heartbeat = mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_SUBMARINE,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    };

    // Spawn Rx loop
    spawner.spawn(rx_task(rx)).unwrap();

    // Main loop
    loop {
        // Write the raw heartbeat message to reduce firmware flash size (using Message::ser will be add ~70KB because
        // all *_DATA::ser methods will be add to firmware).
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_data(header, &heartbeat);
        tx.write(raw.raw_bytes()).await.unwrap();

        // Delay for 1 second
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
pub async fn rx_task(rx: usart::UartRx<'static, Async>) {
    // Make ring-buffered RX (over DMA)
    static BUF_MEMORY: ConstStaticCell<[u8; 1024]> = ConstStaticCell::new([0; 1024]);
    let mut rx_buffered = rx.into_ring_buffered(BUF_MEMORY.take());

    loop {
        // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
        // all *_DATA::deser methods will be add to firmware).
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx_buffered)
            .await
            .unwrap();
        rprintln!("Read raw message: msg_id={}", raw.message_id());

        if raw.message_id() == HEARTBEAT_DATA::ID {
            let heartbeat = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
            rprintln!("heartbeat: {:?}", heartbeat);
        }
    }
}
