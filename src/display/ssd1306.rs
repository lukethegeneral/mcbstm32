use defmt::*;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embedded_graphics::mono_font::iso_8859_13::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::{String, Vec};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

pub const TEXT_BUFFER_LEN: usize = 16;

const TEXT_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
    .font(&FONT_10X20)
    .text_color(BinaryColor::On)
    .build();

pub struct Lcd {
    display: Ssd1306<
        I2CInterface<I2c<'static, Async>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    // Each item is a line of text
    pub text_buffer: Vec<String<TEXT_BUFFER_LEN>, 2>,
}
impl Lcd {
    pub fn new(i2c: I2c<'static, Async>) -> Self {
        // Initialize text buffer with 2 lines
        let mut text_buffer: Vec<String<TEXT_BUFFER_LEN>, 2> = Vec::new();
        text_buffer.push(String::<TEXT_BUFFER_LEN>::new()).unwrap();
        text_buffer.push(String::<TEXT_BUFFER_LEN>::new()).unwrap();

        // Initialize display
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        Self {
            display,
            text_buffer,
        }
    }

    pub async fn display_text(&mut self) {
        // Check text buffer length
        if (self.text_buffer.len()) < 2 {
            info!("Text buffer length is less than 2");
            core::panic!("Text buffer length is less than 2");
        }

        self.display.clear_buffer();

        Text::with_baseline(
            self.text_buffer[0].as_str(),
            Point::zero(),
            TEXT_STYLE,
            Baseline::Top,
        )
        .draw(&mut self.display)
        .unwrap();

        Text::with_baseline(
            self.text_buffer[1].as_str(),
            Point::new(0, 20),
            TEXT_STYLE,
            Baseline::Top,
        )
        .draw(&mut self.display)
        .unwrap();

        self.display.flush().unwrap();
    }
}
