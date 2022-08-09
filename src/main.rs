#![no_std]
#![no_main]

// For string formatting.
use core::fmt::Write;

// The macro for our start-up function
use rp_pico::entry;

// Time handling traits:
use embedded_time::duration::*;
use embedded_time::rate::Extensions;

// CountDown timer for the counter on the display:
use embedded_hal::{digital::v2::OutputPin, timer::CountDown};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp_pico::hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
// use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;

// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

// The display driver:
use ssd1306::{prelude::*, Ssd1306};

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    // let pins = hal::gpio::Pins::new(
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    // Create the I²C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(i2c);

    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Create a text style for drawing the font:
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let mut count = 0;

    let mut buf = FmtBuf::new();

    loop {
        if count % 2 == 1 {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }

        buf.reset();
        // Format some text into a static buffer:
        write!(&mut buf, "counter: {}", count).unwrap();
        count += 1;

        // Empty the display:
        display.clear();

        // Draw 3 lines of text:
        // Text::with_baseline("Hello World!", Point::zero(), text_style, Baseline::Top)
        //     .draw(&mut display)
        //     .unwrap();

        // Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        //     .draw(&mut display)
        //     .unwrap();

        Text::with_baseline(buf.as_str(), Point::new(0, 32), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();

        // Wait a bit:
        // delay.delay_ms(100);
        delay.start(100.milliseconds());
        let _ = nb::block!(delay.wait());
    }
}

/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 64 bytes.
struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            ptr: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
