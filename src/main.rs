#![no_std]
#![no_main]

// mod keyboard;mod keyboard;
mod keyboard;

use core::any::Any;
use core::convert::Infallible;
// mod rotary_encoder;
// mod seven_segment_display;
//use rp_pico as bsp;
use pimoroni_tiny2040 as bsp;
// use crate::rotary_encoder::{Direction, RotaryEncoder};
// use crate::seven_segment_display::Glyph::{One, Zero};
use crate::keyboard::{KeyMap, Keyboard, MyKeyboardReport};
use bsp::entry;
use bsp::hal;
use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use bsp::hal::gpio::DynPin;
use bsp::hal::pac;
use bsp::hal::pac::interrupt;
use bsp::hal::pio::PIOExt;
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::Timer;
use bsp::pac::{CorePeripherals, Interrupt, Peripherals};
use cortex_m::delay::Delay;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use pimoroni_tiny2040::hal::gpio::{AnyPin, DynPinId, PushPullOutput};
use pimoroni_tiny2040::pac::pads_bank0::GPIO;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;
use ws2812_pio::Ws2812;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

const STRIP_LEN: usize = 20;
#[entry]
unsafe fn main() -> ! {
    // START BOILERPLATE
    //// GENERIC
    let mut pac = Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
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

    let mut pins = bsp::Pins::new(
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

    // Setup a delay for the LED blink signals:

    // Import the `sin` function for a smooth hue animation from the
    // Pico rp2040 ROM:
    let sin = hal::rom_data::float_funcs::fsin::ptr();

    // Create a count down timer for the Ws2812 instance:
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // Split the PIO state machine 0 into individual objects, so that
    // Ws2812 can use it:
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip:
    let mut ws = Ws2812::new(
        // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
        // for the LED data output:
        pins.gpio29.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut t = 0.0;

    // Bring down the overall brightness of the strip to not blow
    // the USB power supply: every LED draws ~60mA, RGB means 3 LEDs per
    // ws2812 LED, for 3 LEDs that would be: 3 * 3 * 60mA, which is
    // already 540mA for just 3 white LEDs!
    let strip_brightness = 10u8; // Limit brightness to 64/256

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;

    // let mut r1: DynPin = pins.gpio0.into();
    // r1.into_push_pull_output();
    // r1.set_high();

    // let row_pins: [DynPin; 2] = [pins.gpio0.into(), pins.gpio1.into()];
    // let col_pins: [DynPin; 3] = [pins.gpio5.into(), pins.gpio6.into(), pins.gpio7.into()];

    let row_pins: [DynPin; 5] = [
        pins.gpio4.into(),
        pins.gpio3.into(),
        pins.gpio2.into(),
        pins.gpio1.into(),
        pins.gpio0.into(),
    ];
    let col_pins: [DynPin; 4] = [
        pins.gpio28.into(),
        pins.gpio7.into(),
        pins.gpio6.into(),
        pins.gpio5.into(),
    ];

    //let layer_one = KeyMap::new([[0x04, 0x05, 0x06], [0x14, 0x15, 0x16]]);

    let layer_one = KeyMap::new([
        [0x04, 0x05, 0x06, 0x54],
        [0x5F, 0x60, 0x61, 0x56],
        [0x5C, 0x5D, 0x5E, 0x55],
        [0x59, 0x5A, 0x5B, 0x57],
        [0x53, 0x62, 0x63, 0x58],
    ]);

    // let layer_one = KeyMap::new([
    //     [0x04, 0x04, 0x04, 0x04],
    //     [0x60, 0x60, 0x60, 0x60],
    //     [0x5C, 0x5C, 0x5C, 0x5C],
    //     [0x5B, 0x5B, 0x5B, 0x5B],
    //     [0x63, 0x63, 0x63, 0x63],
    // ]);

    let mut kb = Keyboard::new(row_pins, col_pins, [layer_one]);

    // for r in row_pins.iter_mut() {
    //     r.into_push_pull_output();
    //     r.set_high();
    // }
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // let mut led_pin = pins.led.into_push_pull_output();

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

    // let mut test = pins.gpio12.into_push_pull_output();
    // test.set_low().unwrap();

    unsafe {
        pac::NVIC::unmask(Interrupt::USBCTRL_IRQ);
    }

    let mut counter: usize = 0;

    loop {
        //loop actions
        // let _ = Keyboard::send_keyboard_report(MyKeyboardReport {
        //     modifier: 0,
        //     reserved: 0,
        //     keycodes: [0x04, 0, 0, 0, 0, 0x06],
        // });

        //rotary_encoder.loop_action();
        kb.send_report(&mut delay);

        // remainder of loop

        // if counter % 500 == 0 {
        //     if led_pin.is_set_high().unwrap() {
        //         led_pin.set_low().ok();
        //     } else {
        //         led_pin.set_high().ok();
        //     }
        // }

        for (i, led) in leds.iter_mut().enumerate() {
            // An offset to give 3 consecutive LEDs a different color:
            let hue_offs = match i % 3 {
                1 => 0.25,
                2 => 0.5,
                _ => 0.0,
            };

            let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
            // Bring -1..1 sine range to 0..1 range:
            let sin_01 = (sin_11 + 1.0) * 0.5;

            let hue = 360.0 * sin_01;
            let sat = 1.0;
            let val = 1.0;

            let rgb = hsv2rgb_u8(hue, sat, val);
            *led = rgb.into();
        }

        // Here the magic happens and the `leds` buffer is written to the
        // ws2812 LEDs:
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        // Wait a bit until calculating the next frame:
        delay.delay_ms(16); // ~60 FPS

        // Increase the time counter variable and make sure it
        // stays inbetween 0.0 to 1.0 range:
        t += (16.0 / 1000.0) * animation_speed;
        while t > 1.0 {
            t -= 1.0;
        }

        delay.delay_ms(1);
        counter += 1;
    }
}

// #[panic_handler]
// fn panic(_info: &PanicInfo) -> ! {
//     loop {}
// }

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}

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
