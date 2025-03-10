#![no_std]
#![no_main]

// mod display;
// use display::*;

use core::fmt::write;
use embedded_graphics::mono_font::iso_8859_13::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use heapless::String;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, Temperature};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{ADC1, I2C1, PA1};
use embassy_stm32::time::Hertz;
use embassy_stm32::{adc, bind_interrupts, i2c};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

type AdcType = Mutex<ThreadModeRawMutex, Option<Adc<'static, ADC1>>>;
static ADC: AdcType = Mutex::new(None);

type DisplayType = Mutex<
    ThreadModeRawMutex,
    Option<
        Ssd1306<
            I2CInterface<I2c<'static, Async>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
    >,
>;
static DISPLAY: DisplayType = Mutex::new(None);

const TEXT_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
    .font(&FONT_10X20)
    .text_color(BinaryColor::On)
    .build();

fn convert_to_millivolts(vrefint_sample: u16) -> impl Fn(u16) -> u16 {
    let convert_to_millivolts = move |sample| {
        // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
        // 5.3.4 Embedded reference voltage
        const VREFINT_MV: u32 = 1200; // mV

        (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
    };

    convert_to_millivolts
}

#[embassy_executor::task]
async fn temp(
    adc: &'static AdcType,
    vrefint_sample: u16,
    delay: Duration,
    mut adc_temp: Temperature,
) {
    let convert_to_celcius = |sample| {
        // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
        // Temperature sensor characteristics
        const V25: i32 = 1430; // mV
        const AVG_SLOPE: f32 = 4.3; // mV/C

        let sample_mv = convert_to_millivolts(vrefint_sample)(sample) as i32;

        (sample_mv - V25) as f32 / AVG_SLOPE + 25.0
    };

    loop {
        {
            let mut adc_unlocked = adc.lock().await;
            // Read internal temperature
            if let Some(adc_ref) = adc_unlocked.as_mut() {
                let v = adc_ref.read(&mut adc_temp).await;
                let celcius = convert_to_celcius(v);
                info!("Internal temp: {=u16} ({} C)", v, celcius);
                let mut text_line1: String<8> = String::new();
                let mut text_line2: String<8> = String::new();
                let _ = write(&mut text_line1, format_args!("T: {}", v));
                let _ = write(&mut text_line2, format_args!("C: {:.2}", celcius));
                display_text(&DISPLAY, &text_line1, &text_line2).await;
            }
        }
        //Timer::after_millis(300).await;
        Timer::after(delay).await;
    }
}

#[embassy_executor::task]
async fn volt(adc: &'static AdcType, vrefint_sample: u16, delay: Duration, mut pin: PA1) {
    loop {
        {
            let mut adc_unlocked = adc.lock().await;
            // Read ADC
            if let Some(adc_ref) = adc_unlocked.as_mut() {
                let v = adc_ref.read(&mut pin).await;
                let mv = convert_to_millivolts(vrefint_sample)(v);
                info!("Volt--> {} - {} mV", v, mv);
                let mut text_line1: String<8> = String::new();
                let mut text_line2: String<8> = String::new();
                //let _ = core::fmt::Write::write_fmt(&mut text_line1, format_args!("V:  {}", v));
                let _ = write(&mut text_line1, format_args!("V:  {}", v));
                let _ = write(&mut text_line2, format_args!("mV: {}", mv));
                display_text(&DISPLAY, &text_line1, &text_line2).await;
            }
        }
        Timer::after(delay).await;
    }
}

async fn display_text(display: &'static DisplayType, text_line1: &str, text_line2: &str) {
    let mut display_unlocked = display.lock().await;
    if let Some(display_ref) = display_unlocked.as_mut() {
        display_ref.clear_buffer();
        Text::with_baseline(text_line1, Point::zero(), TEXT_STYLE, Baseline::Top)
            .draw(display_ref)
            .unwrap();
        Text::with_baseline(text_line2, Point::new(0, 20), TEXT_STYLE, Baseline::Top)
            .draw(display_ref)
            .unwrap();
        display_ref.flush().unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    //let i2c = I2c::new_blocking(
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz::khz(400),
        Default::default(),
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();

    {
        *(DISPLAY.lock().await) = Some(display);
    }

    display_text(&DISPLAY, "text_line1", "").await;

    Timer::after_millis(300).await;

    // ADC
    let adc = Adc::new(p.ADC1);
    {
        *(ADC.lock().await) = Some(adc);
    }

    Timer::after_millis(300).await;

    let mut vrefint = ADC.lock().await.as_mut().unwrap().enable_vref();
    let vrefint_sample = ADC.lock().await.as_mut().unwrap().read(&mut vrefint).await;
    let adc_temp = ADC.lock().await.as_mut().unwrap().enable_temperature();

    unwrap!(spawner.spawn(temp(
        &ADC,
        vrefint_sample,
        Duration::from_millis(300),
        adc_temp
    )));
    unwrap!(spawner.spawn(volt(
        &ADC,
        vrefint_sample,
        Duration::from_millis(500),
        p.PA1
    )));
}
