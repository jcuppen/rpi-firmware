//! # Seven Digit Display
//!
//!      g f - a b
//!      │ │ │ │ │
//!     ┌┴─┴─┴─┴─┴┐
//!     │         │
//!     │ ┌─────┐ │
//!     │ │     │ │
//!     │ │     │ │
//!     │ ├─────┤ │
//!     │ │     │ │
//!     │ │     │ │
//!     │ └─────┘ │
//!     │         │
//!     └┬─┬─┬─┬─┬┘
//!      │ │ │ │ │
//!      e d - c .
//!  note: `-` needs to be connected to a 220Ω resistor

use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio::{Pin, PinId, PullDownDisabled, PushPullOutput};

#[allow(dead_code)]
pub(crate) enum Glyph {
    Zero,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
    Nine,
    A,
    C,
    E,
    F,
    H,
    J,
    L,
    N,
    P,
    U,
    Y,
    Minus,
    Dot,
    Invalid,
}

pub(crate) struct SevenSegmentDisplay<
    A: PinId,
    B: PinId,
    C: PinId,
    D: PinId,
    E: PinId,
    F: PinId,
    G: PinId,
    H: PinId,
> {
    pin_a: Pin<A, PushPullOutput>,
    pin_b: Pin<B, PushPullOutput>,
    pin_c: Pin<C, PushPullOutput>,
    pin_d: Pin<D, PushPullOutput>,
    pin_e: Pin<E, PushPullOutput>,
    pin_f: Pin<F, PushPullOutput>,
    pin_g: Pin<G, PushPullOutput>,
    pin_dp: Pin<H, PushPullOutput>,
}

impl<A: PinId, B: PinId, C: PinId, D: PinId, E: PinId, F: PinId, G: PinId, H: PinId>
    SevenSegmentDisplay<A, B, C, D, E, F, G, H>
{
    fn set_pins(&mut self, data: [u8; 8]) {
        if data[0] == 0 {
            self.pin_a.set_low()
        } else {
            self.pin_a.set_high()
        }
        .unwrap();
        if data[1] == 0 {
            self.pin_b.set_low()
        } else {
            self.pin_b.set_high()
        }
        .unwrap();
        if data[2] == 0 {
            self.pin_c.set_low()
        } else {
            self.pin_c.set_high()
        }
        .unwrap();
        if data[3] == 0 {
            self.pin_d.set_low()
        } else {
            self.pin_d.set_high()
        }
        .unwrap();
        if data[4] == 0 {
            self.pin_e.set_low()
        } else {
            self.pin_e.set_high()
        }
        .unwrap();
        if data[5] == 0 {
            self.pin_f.set_low()
        } else {
            self.pin_f.set_high()
        }
        .unwrap();
        if data[6] == 0 {
            self.pin_g.set_low()
        } else {
            self.pin_g.set_high()
        }
        .unwrap();
        if data[7] == 0 {
            self.pin_dp.set_low()
        } else {
            self.pin_dp.set_high()
        }
        .unwrap()
    }

    #[allow(dead_code)]
    pub(crate) fn unset_glyph(&mut self) {
        self.set_pins([0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn create(
        a: Pin<A, PullDownDisabled>,
        b: Pin<B, PullDownDisabled>,
        c: Pin<C, PullDownDisabled>,
        d: Pin<D, PullDownDisabled>,
        e: Pin<E, PullDownDisabled>,
        f: Pin<F, PullDownDisabled>,
        g: Pin<G, PullDownDisabled>,
        dp: Pin<H, PullDownDisabled>,
    ) -> Self {
        Self {
            pin_a: a.into_push_pull_output(),
            pin_b: b.into_push_pull_output(),
            pin_c: c.into_push_pull_output(),
            pin_d: d.into_push_pull_output(),
            pin_e: e.into_push_pull_output(),
            pin_f: f.into_push_pull_output(),
            pin_g: g.into_push_pull_output(),
            pin_dp: dp.into_push_pull_output(),
        }
    }

    pub(crate) fn set_glyph(&mut self, glyph: Glyph) {
        match glyph {
            Glyph::Zero => self.set_pins([1, 1, 1, 1, 1, 1, 0, 0]),
            Glyph::One => self.set_pins([0, 1, 1, 0, 0, 0, 0, 0]),
            Glyph::Two => self.set_pins([1, 1, 0, 1, 1, 0, 1, 0]),
            Glyph::Three => self.set_pins([1, 1, 1, 1, 0, 0, 1, 0]),
            Glyph::Four => self.set_pins([0, 1, 1, 0, 0, 1, 1, 0]),
            Glyph::Five => self.set_pins([1, 0, 1, 1, 0, 1, 1, 0]),
            Glyph::Six => self.set_pins([1, 0, 1, 1, 1, 1, 1, 0]),
            Glyph::Seven => self.set_pins([1, 1, 1, 0, 0, 0, 0, 0]),
            Glyph::Eight => self.set_pins([1, 1, 1, 1, 1, 1, 1, 0]),
            Glyph::Nine => self.set_pins([1, 1, 1, 1, 0, 1, 1, 0]),
            Glyph::A => self.set_pins([1, 1, 1, 0, 1, 1, 1, 0]),
            Glyph::C => self.set_pins([1, 0, 0, 1, 1, 1, 0, 0]),
            Glyph::E => self.set_pins([1, 0, 0, 1, 1, 1, 1, 0]),
            Glyph::F => self.set_pins([1, 0, 0, 0, 1, 1, 1, 0]),
            Glyph::H => self.set_pins([0, 1, 1, 0, 1, 1, 1, 0]),
            Glyph::J => self.set_pins([0, 1, 1, 1, 0, 0, 0, 0]),
            Glyph::L => self.set_pins([0, 0, 0, 1, 1, 1, 0, 0]),
            Glyph::N => self.set_pins([1, 1, 1, 0, 1, 1, 0, 0]),
            Glyph::P => self.set_pins([1, 1, 0, 0, 1, 1, 1, 0]),
            Glyph::U => self.set_pins([0, 1, 1, 1, 1, 1, 0, 0]),
            Glyph::Y => self.set_pins([0, 1, 1, 1, 0, 1, 1, 0]),
            Glyph::Minus => self.set_pins([0, 0, 0, 0, 0, 0, 1, 0]),
            Glyph::Dot => self.set_pins([0, 0, 0, 0, 0, 0, 0, 1]),
            Glyph::Invalid => self.set_pins([0, 1, 0, 0, 1, 0, 1, 0]),
        }
    }
}
