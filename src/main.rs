#![no_std]
#![no_main]

// mod keyboard;mod keyboard;
mod keyboard;
// mod rotary_encoder;
// mod seven_segment_display;
use rp_pico as bsp;

// use crate::rotary_encoder::{Direction, RotaryEncoder};
// use crate::seven_segment_display::Glyph::{One, Zero};
use crate::keyboard::{KeyMap, Keyboard, MyKeyboardReport};
use bsp::entry;
use bsp::hal;
use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use bsp::hal::gpio::DynPin;
use bsp::hal::pac;
use bsp::hal::pac::interrupt;
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;
use bsp::pac::{CorePeripherals, Interrupt, Peripherals};
use cortex_m::delay::Delay;
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
unsafe fn main() -> ! {
    // START BOILERPLATE
    //// GENERIC
    let mut pac = Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let core = CorePeripherals::take().unwrap();

    //// OLED
    // let i2c = I2C::i2c0(
    //     pac.I2C0,
    //     pins.gpio16.into_mode::<FunctionI2C>(),
    //     pins.gpio17.into_mode::<FunctionI2C>(),
    //     400.kHz(),
    //     &mut pac.RESETS,
    //     &clocks.peripheral_clock,
    // );

    //let interface = ssd1306::I2CDisplayInterface::new(i2c);

    // let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    //     .into_buffered_graphics_mode();

    // display.init().unwrap();

    //// Keyboard
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let usb_hid = HIDClass::new(bus_ref, MyKeyboardReport::desc(), 60);

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Fake company")
        .product("Twitchy Mousey")
        .serial_number("TEST")
        .device_class(0)
        .build();

    unsafe {
        USB_HID = Some(usb_hid);
        USB_DEVICE = Some(usb_dev);
    }

    // END BOILERPLATE

    let row_pins: [DynPin; 2] = [pins.gpio14.into(), pins.gpio15.into()];
    let col_pins: [DynPin; 3] = [pins.gpio11.into(), pins.gpio10.into(), pins.gpio9.into()];

    let layer_one = KeyMap::new([[0x04, 0x05, 0x06], [0x14, 0x15, 0x16]]);

    let mut kb = Keyboard::new(row_pins, col_pins, [layer_one]);

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut led_pin = pins.led.into_push_pull_output();

    // let mut ssd = seven_segment_display::SevenSegmentDisplay::create(
    //     pins.gpio26,
    //     pins.gpio22,
    //     pins.gpio19,
    //     pins.gpio20,
    //     pins.gpio21,
    //     pins.gpio27,
    //     pins.gpio28,
    //     pins.gpio18,
    // );

    // Create a text style for drawing the font:

    //let mut rotary_encoder = RotaryEncoder::create(pins.gpio13, pins.gpio14, pins.gpio15);

    let mut test = pins.gpio12.into_push_pull_output();
    test.set_low().unwrap();

    unsafe {
        pac::NVIC::unmask(Interrupt::USBCTRL_IRQ);
    }

    let mut counter: usize = 0;

    loop {
        //loop actions

        //rotary_encoder.loop_action();
        kb.send_report(&mut delay);

        // remainder of loop

        // if rotary_encoder.is_pressed() {
        //     // ssd.set_glyph(One);
        //     test.set_low().unwrap()
        // } else {
        //     test.set_high().unwrap();
        //     // ssd.set_glyph(Zero);
        // }

        // match rotary_encoder.direction() {
        //     Direction::Clockwise => ssd.set_glyph(One),
        //     Direction::CounterClockwise => ssd.set_glyph(Zero),
        // }

        if counter % 500 == 0 {
            if led_pin.is_set_high().unwrap() {
                led_pin.set_low().ok();
            } else {
                led_pin.set_high().ok();
            }
        }

        delay.delay_ms(1);
        counter += 1;
    }
}

// #[panic_handler]
// fn panic(_info: &PanicInfo) -> ! {
//     loop {}
// }

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
// End of file
