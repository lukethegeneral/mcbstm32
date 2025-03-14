#![no_std]
#![no_main]

// mod display;
// use display::*;

use core::fmt::write;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_graphics::mono_font::iso_8859_13::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use heapless::{String, Vec};
use static_cell::StaticCell;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, Temperature};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{ADC1, I2C1};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::{adc, bind_interrupts, i2c, peripherals, spi, Config};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::sdcard::SdCard;
use embedded_sdmmc::{File, VolumeManager};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

static SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

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

const TEXT_BUFFER_LEN: usize = 10;
// LCD buffer for text
// Each item is a line of text
static TEXT_BUFFER: Mutex<ThreadModeRawMutex, Vec<String<TEXT_BUFFER_LEN>, 2>> =
    Mutex::new(Vec::new());

type LogFileType = Mutex<
    ThreadModeRawMutex,
    Option<
        File<
            'static,
            SdCard<ExclusiveDevice<Spi<'static, Async>, Output<'static>, NoDelay>, Delay>,
            DummyTimesource,
            4,
            4,
            1,
        >,
    >,
>;

static LOG_FILE: LogFileType = Mutex::new(None);

// Dummy time source for SD card
struct DummyTimesource();

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

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
/// Task that ticks periodically
async fn tick_periodic() -> ! {
    let mut counter: u32 = 0;
    loop {
        SIGNAL.signal(counter);
        counter = counter.wrapping_add(1);

        info!("tick! {}", counter);

        let mut text: String<TEXT_BUFFER_LEN> = String::new();
        let _ = write(&mut text, format_args!("\n[{}]\n", counter));
        log_data(&LOG_FILE, text.as_bytes()).await;
        LOG_FILE.lock().await.as_mut().unwrap().flush().unwrap();

        Timer::after(Duration::from_millis(1000)).await;
    }
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
                let mut text: String<TEXT_BUFFER_LEN> = String::new();
                let _ = write(&mut text, format_args!("C: {:.2}", celcius));
                //info!("len: {}", text.clone().into_bytes().len());
                {
                    let mut t = TEXT_BUFFER.lock().await;
                    t[0] = text;
                }
                display_text(&DISPLAY).await;
            }
        }
        Timer::after(delay).await;
    }
}

#[embassy_executor::task]
async fn volt(
    adc: &'static AdcType,
    vrefint_sample: u16,
    delay: Duration,
    mut pin: peripherals::PA1,
) {
    loop {
        {
            let mut adc_unlocked = adc.lock().await;
            // Read ADC
            if let Some(adc_ref) = adc_unlocked.as_mut() {
                let v = adc_ref.read(&mut pin).await;
                let mv = convert_to_millivolts(vrefint_sample)(v);
                info!("Volt--> {} - {} mV", v, mv);

                // Write to display
                let mut text: String<TEXT_BUFFER_LEN> = String::new();
                let _ = write(&mut text, format_args!("mV: {}", mv));
                {
                    let mut t = TEXT_BUFFER.lock().await;
                    t[1] = text.clone();
                }
                display_text(&DISPLAY).await;

                // Write to SD card
                text.clear();
                //let received_counter = SIGNAL.wait().await;
                //let _ = write(&mut text, format_args!("{}\n", mv));
                let _ = write(&mut text, format_args!("{},", mv));
                log_data(&LOG_FILE, text.as_bytes()).await;
            }
        }
        Timer::after(delay).await;
    }
}

async fn log_data(log_file: &'static LogFileType, data: &[u8]) {
    let mut log_file_unlocked = log_file.lock().await;
    if let Some(log_file_ref) = log_file_unlocked.as_mut() {
        log_file_ref.write(data).expect("Error writing to log file");
        //log_file_ref.flush().unwrap();
    }
}

async fn display_text(display: &'static DisplayType) {
    // Check text buffer length
    if (TEXT_BUFFER.lock().await.len()) < 2 {
        info!("Text buffer length is less than 2");
        core::panic!("Text buffer length is less than 2");
    }

    let mut display_unlocked = display.lock().await;
    if let Some(display_ref) = display_unlocked.as_mut() {
        display_ref.clear_buffer();
        Text::with_baseline(
            TEXT_BUFFER.lock().await[0].as_str(),
            Point::zero(),
            TEXT_STYLE,
            Baseline::Top,
        )
        .draw(display_ref)
        .unwrap();
        Text::with_baseline(
            TEXT_BUFFER.lock().await[1].as_str(),
            Point::new(0, 20),
            TEXT_STYLE,
            Baseline::Top,
        )
        .draw(display_ref)
        .unwrap();
        display_ref.flush().unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.ls = LsConfig::default_lse();
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            // Oscillator for bluepill, Bypass for nucleos.
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    //let p = embassy_stm32::init(Default::default());
    let p = embassy_stm32::init(config);

    info!("Time to start!");

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
    let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    //display.init().unwrap();
    {
        *(DISPLAY.lock().await) = Some(display);
    }
    DISPLAY.lock().await.as_mut().unwrap().init().unwrap();

    // Initialize text buffer with 2 lines
    TEXT_BUFFER
        .lock()
        .await
        .push(String::<TEXT_BUFFER_LEN>::new())
        .unwrap();
    TEXT_BUFFER
        .lock()
        .await
        .push(String::<TEXT_BUFFER_LEN>::new())
        .unwrap();

    // SPI
    // SPI clock needs to be running at <= 400kHz during initialization
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(400_000);
    let (miso, mosi, clk) = (p.PA6, p.PA7, p.PA5);
    let spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA1_CH3, p.DMA1_CH2, spi_cfg);
    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi_dev = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    //let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let sdcard = SdCard::new(spi_dev, Delay);
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());

    // Now that the card is initialized, the SPI clock can go faster
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(72_000_000);
    sdcard
        .spi(|dev| dev.bus_mut().set_config(&spi_cfg))
        .unwrap();

    // Now let's look for volumes (also known as partitions) on our block device.
    // To do this we need a Volume Manager. It will take ownership of the block device.
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());

    static VOL: StaticCell<
        VolumeManager<
            SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, NoDelay>, Delay>,
            DummyTimesource,
        >,
    > = StaticCell::new();
    let volume_mgr = VOL.init(volume_mgr);

    // Try and access Volume 0 (i.e. the first partition).
    // The volume object holds information about the filesystem on that volume.
    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

    static VOL0: StaticCell<
        embedded_sdmmc::Volume<
            '_,
            SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, NoDelay>, Delay>,
            DummyTimesource,
            4,
            4,
            1,
        >,
    > = StaticCell::new();
    let volume0 = VOL0.init(volume0);

    // Open the root directory (mutably borrows from the volume).
    let mut root_dir = volume0.open_root_dir().unwrap();

    static ROOT: StaticCell<
        embedded_sdmmc::Directory<
            '_,
            SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, NoDelay>, Delay>,
            DummyTimesource,
            4,
            4,
            1,
        >,
    > = StaticCell::new();
    let root_dir = ROOT.init(root_dir);

    let log_file = root_dir
        .open_file_in_dir(
            "RPM_DATA.CSV",
            //embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        )
        .unwrap();
    {
        *(LOG_FILE.lock().await) = Some(log_file);
    }
    //log_file.close().unwrap();
    let mut text: String<TEXT_BUFFER_LEN> = String::new();
    let _ = write(&mut text, format_args!("Start\n"));
    log_data(&LOG_FILE, text.as_bytes()).await;
    LOG_FILE.lock().await.as_mut().unwrap().flush().unwrap();

    // ADC
    let adc = Adc::new(p.ADC1);
    {
        *(ADC.lock().await) = Some(adc);
    }

    let mut vrefint = ADC.lock().await.as_mut().unwrap().enable_vref();
    let vrefint_sample = ADC.lock().await.as_mut().unwrap().read(&mut vrefint).await;
    let adc_temp = ADC.lock().await.as_mut().unwrap().enable_temperature();

    // Run tasks
    unwrap!(spawner.spawn(temp(
        &ADC,
        vrefint_sample,
        Duration::from_millis(100),
        adc_temp
    )));
    unwrap!(spawner.spawn(volt(&ADC, vrefint_sample, Duration::from_millis(1), p.PA1)));
    unwrap!(spawner.spawn(tick_periodic()));
}
