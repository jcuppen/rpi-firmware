//! # Rotary Encoder
//!
//!     Top Down Perspective
//!       ┌────────────────────────┐
//!     ┌─┤                        ├─┐
//!     │ │ dt(PD/PU)        + / - │ │
//!     └─┤                        ├─┘
//!     ┌─┤                        │
//!     │ │ + / -                  │
//!     └─┤                        │
//!     ┌─┤                        ├─┐
//!     │ │clk(PD/PU)   btn(PD/PU) │ │
//!     └─┤                        ├─┘
//!       └────────────────────────┘

use crate::rotary_encoder::Direction::{Clockwise, CounterClockwise};
use embedded_hal::digital::v2::InputPin;
use rp_pico::hal::gpio::{Pin, PinId, PullDownDisabled, PullDownInput};

#[derive(Copy, Clone)]
pub(crate) enum Direction {
    Clockwise,
    CounterClockwise,
}

pub(crate) struct RotaryEncoder<A: PinId, B: PinId, C: PinId> {
    button_pin: Pin<A, PullDownInput>,
    click_pin: Pin<B, PullDownInput>,
    direction_pin: Pin<C, PullDownInput>,

    last_state_click: bool,
    counter: isize,
    direction: Direction,
}

impl<A: PinId, B: PinId, C: PinId> RotaryEncoder<A, B, C> {
    pub(crate) fn create(
        btn: Pin<A, PullDownDisabled>,
        clk: Pin<B, PullDownDisabled>,
        dt: Pin<C, PullDownDisabled>,
    ) -> Self {
        let mut this = Self {
            button_pin: btn.into_pull_down_input(),
            click_pin: clk.into_pull_down_input(),
            direction_pin: dt.into_pull_down_input(),
            last_state_click: false,
            counter: 0,
            direction: Clockwise,
        };

        this.last_state_click = this.click_pin.is_high().unwrap();

        this
    }

    pub(crate) fn direction(&self) -> Direction {
        self.direction
    }

    pub(crate) fn loop_action(&mut self) {
        let current_state_click = self.click_pin.is_high().unwrap();

        if current_state_click != self.last_state_click && current_state_click {
            if self.direction_pin.is_high().unwrap() != current_state_click {
                self.counter -= 1;
                self.direction = CounterClockwise;
            } else {
                self.counter += 1;
                self.direction = Clockwise;
            }
        }

        self.last_state_click = current_state_click;
    }

    pub(crate) fn is_pressed(&self) -> bool {
        self.button_pin.is_high().unwrap()
    }
}
