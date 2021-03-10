//! Target board: stm32f303RETx (stm32nucleo)
//! Manual: https://www.st.com/resource/en/reference_manual/dm00043574-stm32f303xb-c-d-e-stm32f303x6-8-stm32f328x8-stm32f358xc-stm32f398xe-advanced-arm-based-mcus-stmicroelectronics.pdf
#![feature(alloc_error_handler)]
#![no_main]
#![no_std]

// Panic handler
use panic_halt as _;

use cortex_m_rt::entry;
use hal::pac;
use hal::prelude::*;
use mavlink;
use static_alloc::Bump;
use stm32f3xx_hal as hal;

#[global_allocator] // 1KB allocator
static mut ALLOCATOR: Bump<[u8; 1 << 10]> = Bump::uninit();

#[entry]
fn main() -> ! {
    // Peripherals access
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 9: RCC: Reset and clock control (RCC)
    let mut rcc = dp.RCC.constrain();

    // Configure GPIOA using AHB
    // 9.4.6: AHB peripheral clock enable register (RCC_AHBENR)
    // Bit 17 IOPAEN: I/O port A clock enable
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    // stm32nucleo has a LED on pin PA5
    let mut led = gpioa
        .pa5
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    // Constrains the FLASH peripheral to play nicely with the other abstractions
    let mut flash = dp.FLASH.constrain();

    // Freezes the clock configuration
    // This function internally calculates the specific
    // divisors for the different clock peripheries
    // 4.5.1 Flash access control register (FLASH_ACR)
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // USART2 uses Pins A9 and A10
    // We don't need the datasheet to check which alternative function to use
    // https://docs.rs/stm32f3xx-hal/0.6.1/stm32f3xx_hal/gpio/gpioa/struct.PA2.html#impl-TxPin%3CUSART2%3E
    // The documentation provide the necessary information about each possible hardware configuration
    let pin_tx = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
    let pin_rx = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);

    // Create an interface USART2 with 115200 baudrate
    let serial = hal::serial::Serial::usart2(
        dp.USART2,
        (pin_tx, pin_rx),
        115_200.bps(),
        clocks,
        &mut rcc.apb1,
    );

    // Break serial in TX and RX (not used)
    let (mut tx, _) = serial.split();

    // Create our mavlink header and heartbeat message
    let header = mavlink_header();
    let heartbeat = mavlink_heartbeat_message();

    // Create a delay object based on SysTick
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    // Main loop
    loop {
        // Clear allocator before restarting our loop (cheap allocator)
        // unnecessary for alloc_cortex_m::CortexMHeap
        unsafe {
            ALLOCATOR.reset();
        };

        // Write the mavlink message via serial
        mavlink::write_versioned_msg(&mut tx, mavlink::MavlinkVersion::V2, header, &heartbeat)
            .unwrap();

        // Toggle the LED
        led.toggle().unwrap();

        // Delay for 1 second
        delay.delay_ms(1_000u32);
    }
}

fn mavlink_header() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    }
}

pub fn mavlink_heartbeat_message() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_SUBMARINE,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    // Init debug state
    cortex_m::asm::bkpt();
    loop {}
}
