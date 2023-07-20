use cortex_m::delay::Delay;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico::hal::gpio::DynPin;
use usb_device::class_prelude::*;
use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::generator_prelude::*;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0xE0, usage_max = 0xE7) = {
            #[packed_bits 8] #[item_settings data,variable,absolute] modifier=input;
        };
        (usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings constant,variable,absolute] reserved=input;
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xDD) = {
            #[item_settings data,array,absolute] keycodes=input;
        };
    }
)]
#[allow(dead_code)]
pub(crate) struct MyKeyboardReport {
    pub modifier: u8,
    pub reserved: u8,
    pub keycodes: [u8; 6],
}
// USB Human Interface Device (HID) Class support
use crate::USB_HID;

pub(crate) struct KeyMap {
    matrix: [[u8; 3]; 2],
}

impl KeyMap {
    pub(crate) fn new(matrix: [[u8; 3]; 2]) -> KeyMap {
        Self { matrix }
    }

    pub(crate) fn get_keycode(&self, row: usize, col: usize) -> u8 {
        self.matrix[row][col]
    }
}

pub(crate) struct Keyboard {
    row_pins: [DynPin; 2],
    col_pins: [DynPin; 3],
    keymaps: [KeyMap; 1],
}

impl Keyboard {
    pub(crate) fn new(
        row_pins: [DynPin; 2],
        col_pins: [DynPin; 3],
        keymaps: [KeyMap; 1],
    ) -> Keyboard {
        let mut kb = Self {
            row_pins,
            col_pins,
            keymaps,
        };

        for r in kb.row_pins.as_mut_slice() {
            r.into_pull_down_input()
        }
        for c in kb.col_pins.as_mut_slice() {
            c.into_push_pull_output();
        }

        kb
    }

    pub(crate) fn send_report(&mut self, delay: &mut Delay) {
        let mut keys: [u8; 6] = [0; 6];
        let mut keys_pressed: u8 = 0;

        for col in 0..self.col_pins.len() {
            self.col_pins[col].set_high().unwrap();

            delay.delay_us(10);

            for row in 0..self.row_pins.len() {
                if self.row_pins[row].is_high().unwrap() {
                    keys[keys_pressed as usize] = self.keymaps[0].get_keycode(row, col);
                    keys_pressed += 1;
                    if keys_pressed == 5 {
                        let _ = Keyboard::send_keyboard_report(MyKeyboardReport {
                            modifier: 0,
                            reserved: 0,
                            keycodes: keys,
                        });
                        return;
                    }
                }
            }

            self.col_pins[col].set_low().unwrap();
        }

        let _ = Keyboard::send_keyboard_report(MyKeyboardReport {
            modifier: 0,
            reserved: 0,
            keycodes: keys,
        });
    }

    pub(crate) fn send_empty_report(&self) {
        let _ = Keyboard::send_keyboard_report(MyKeyboardReport {
            modifier: 0,
            reserved: 0,
            keycodes: [0, 0, 0, 0, 0, 0],
        });
    }

    fn send_keyboard_report(report: MyKeyboardReport) -> Result<usize, UsbError> {
        critical_section::with(|_| unsafe {
            // Now interrupts are disabled, grab the global variable and, if
            // available, send it a HID report
            USB_HID.as_mut().map(|hid| hid.push_input(&report))
        })
        .unwrap()
    }
}
