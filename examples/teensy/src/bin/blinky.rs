//! This example test the Teensy 4.X on board LED.
//!

#![no_std]
#![no_main]

// use defmt::*;
use embassy_executor::Spawner;
use embassy_teensy::*;
use embassy_time::Instant;
use embassy_time::Timer;
// use gpio::{Level, Output};
use core::arch::asm;
use embassy_teensy::teensy4_panic as _;

use embassy_teensy::hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};
use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
};
use usbd_serial::SerialPort;

/// Change me if you want to play with a full-speed USB device.
const SPEED: Speed = Speed::High;
/// Matches whatever is in imxrt-log.
const VID_PID: UsbVidPid = UsbVidPid(0x5824, 0x27dd);
const PRODUCT: &str = "imxrt-hal-example";

/// This allocation is shared across all USB endpoints. It needs to be large
/// enough to hold the maximum packet size for *all* endpoints. If you start
/// noticing panics, check to make sure that this is large enough for all endpoints.
static EP_MEMORY: EndpointMemory<2048> = EndpointMemory::new();
/// This manages the endpoints. It's large enough to hold the maximum number
/// of endpoints; we're not using all the endpoints in this example.
static EP_STATE: EndpointState = EndpointState::max_endpoints();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_teensy::init();
    let led = embassy_teensy::board::led(&mut p.gpio2, p.pins.p13);

    let bus = BusAdapter::with_speed(p.usb, &EP_MEMORY, &EP_STATE, SPEED);
    bus.set_interrupts(true);

    let bus: UsbBusAllocator<BusAdapter> = UsbBusAllocator::new(bus);
    let mut class = SerialPort::new(&bus);
    let mut device = UsbDeviceBuilder::new(&bus, VID_PID)
        .product(PRODUCT)
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64)
        .build();

    let mut configured = false;

    loop {
        if device.poll(&mut [&mut class]) {
            if device.state() == UsbDeviceState::Configured {
                if !configured {
                    device.bus().configure();
                }
                configured = true;
            }
        }

        // info!("led on!");
        led.set();
        // Timer::after_ticks(100_000_000).await;
        let mut n: u32 = u32::MAX / 1_000;
        while n > 0 {
            n = n - 1;
            unsafe {
                asm!("nop");
            }
        }

        if device.poll(&mut [&mut class]) {
            if device.state() == UsbDeviceState::Configured {
                if !configured {
                    device.bus().configure();
                }
                configured = true;
            }
        }

        // info!("led off!");
        led.clear();
        // Timer::after_ticks(10).await;
        let ticks = Instant::now();
        let b = ticks.as_ticks();
        class.write(&b.to_be_bytes()).ok();
        let mut n: u32 = u32::MAX / 1_000;
        while n > 0 {
            n = n - 1;
            unsafe {
                asm!("nop");
            }
        }
    }
}
