#![no_std]
#![no_main]

mod display;
use display::{Lcd, TEXT_BUFFER_LEN};

mod log;
use embassy_stm32::pac::Interrupt::SPI1;
use log::LogFile;

use core::fmt::write;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::{String, Vec};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime, Temperature};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{ADC1, I2C1};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::{adc, bind_interrupts, i2c, peripherals, spi, Config};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

static SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

type AdcType = Mutex<ThreadModeRawMutex, Option<Adc<'static, ADC1>>>;
static ADC: AdcType = Mutex::new(None);

type LcdType = Mutex<ThreadModeRawMutex, Option<Lcd>>;
static LCD: LcdType = Mutex::new(None);

type LogFileType = Mutex<ThreadModeRawMutex, Option<LogFile>>;
static LOG_FILE: LogFileType = Mutex::new(None);

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
// Task that ticks periodically
async fn tick_periodic(log_file: &'static LogFileType) -> ! {
    let mut counter: u32 = 0;
    //let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        // SIGNAL.signal(counter);
        // counter = counter.wrapping_add(1);

        info!("tick! {}", counter);

        let mut text: String<TEXT_BUFFER_LEN> = String::new();
        let _ = write(&mut text, format_args!("\n[{}]\n", counter));

        let mut log_file_unlocked = log_file.lock().await;
        if let Some(log_file_ref) = log_file_unlocked.as_mut() {
            log_file_ref.log_data(text.as_bytes()).await;
            //   log_file_ref.log_file.flush().unwrap();
        }

        Timer::after(Duration::from_millis(1000)).await; // 1 second
                                                         //ticker.next().await;

        counter += 1;
    }
}

#[embassy_executor::task]
async fn temp(
    adc: &'static AdcType,
    vrefint_sample: u16,
    delay: Duration,
    mut adc_temp: Temperature,
    lcd: &'static LcdType,
) -> ! {
    //let mut ticker = Ticker::every(delay);

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
                let mut text_lcd: String<TEXT_BUFFER_LEN> = String::new();
                let _ = write(&mut text_lcd, format_args!("C: {:.2}", celcius));

                let mut lcd_unlocked = lcd.lock().await;
                if let Some(lcd_ref) = lcd_unlocked.as_mut() {
                    lcd_ref.text_buffer[0] = text_lcd;
                    lcd_ref.display_text().await;
                }
            }
        }
        Timer::after(delay).await;
        //ticker.next().await;
    }
}

#[embassy_executor::task]
async fn volt(
    adc: &'static AdcType,
    vrefint_sample: u16,
    delay: Duration,
    mut pin: peripherals::PA1,
    lcd: &'static LcdType,
    log_file: &'static LogFileType,
) -> ! {
    //let mut ticker = Ticker::every(delay);
    static BUF_SIZE: usize = 300;
    let mut buf_mv = [0u8; BUF_SIZE];
    let mut buf_mv_idx = 0;
    loop {
        {
            let mut adc_unlocked = adc.lock().await;
            // Read ADC
            if let Some(adc_ref) = adc_unlocked.as_mut() {
                let v = adc_ref.read(&mut pin).await;
                let mv = convert_to_millivolts(vrefint_sample)(v);
                info!("Volt--> {} - {} mV", v, mv);

                // Write to display
                let mut text_lcd: String<TEXT_BUFFER_LEN> = String::new();
                let _ = write(&mut text_lcd, format_args!["mV: {}", mv]);

                let mut lcd_unlocked = lcd.lock().await;
                if let Some(lcd_ref) = lcd_unlocked.as_mut() {
                    lcd_ref.text_buffer[1] = text_lcd;
                    lcd_ref.display_text().await;
                }

                // Write to buffer
                let mv_bytes = mv.to_be_bytes();
                buf_mv[buf_mv_idx] = mv_bytes[0];
                buf_mv[buf_mv_idx + 1] = mv_bytes[1];
                buf_mv[buf_mv_idx + 2] = ',' as u8;
                info!(
                    "index[{}] mv: {} buf: 0x{:x} 0x{:x} {}",
                    buf_mv_idx,
                    mv,
                    buf_mv[buf_mv_idx],
                    buf_mv[buf_mv_idx + 1],
                    buf_mv[buf_mv_idx + 2],
                );
                if buf_mv_idx >= BUF_SIZE - 3 {
                    //for elem in buf_mv.iter() {
                    info!("buf_mv: {:?}", buf_mv);
                    let mut log_file_unlocked = log_file.lock().await;
                    if let Some(log_file_ref) = log_file_unlocked.as_mut() {
                        log_file_ref.log_data(&buf_mv).await;
                        // log_file_ref.log_file.flush().unwrap();
                    }
                    //}
                    buf_mv_idx = 0;
                } else {
                    buf_mv_idx += 3;
                }
            }
        }
        Timer::after(delay).await;
        //ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.ls = LsConfig {
            rtc: RtcClockSource::LSE,
            lsi: false,
            lse: Some(LseConfig {
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator(LseDrive::MediumLow),
            }),
        };
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL2,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.adc_pre = ADCPrescaler::DIV2;
    }
    let p = embassy_stm32::init(config);
    //let p = embassy_stm32::init(Default::default());

    info!("Program to start!");

    // I2C
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

    // LCD initialization
    {
        *(LCD.lock().await) = Some(Lcd::new(i2c));
    }

    // SPI
    // SPI clock needs to be running at <= 400kHz during initialization
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(400_000);
    let (miso, mosi, clk) = (p.PA6, p.PA7, p.PA5);
    let spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA1_CH3, p.DMA1_CH2, spi_cfg);
    let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi_dev = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    //let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    // Log file & SD card initialization
    {
        *(LOG_FILE.lock().await) = Some(LogFile::new(spi_dev, "RPM_DATA.CSV"));
    }

    // ADC initialization
    let adc = Adc::new(p.ADC1);
    //adc.set_sample_time(SampleTime::CYCLES28_5);
    {
        *(ADC.lock().await) = Some(adc);
    }

    let mut vrefint = ADC.lock().await.as_mut().unwrap().enable_vref();
    let vrefint_sample = ADC.lock().await.as_mut().unwrap().read(&mut vrefint).await;
    let adc_temp = ADC.lock().await.as_mut().unwrap().enable_temperature();

    let dt = 100 * 1_000_000;
    let k = 1.003;
    // Run tasks
    unwrap!(spawner.spawn(volt(
        &ADC,
        vrefint_sample,
        Duration::from_nanos(dt),
        p.PA1,
        &LCD,
        &LOG_FILE
    )));
    unwrap!(spawner.spawn(temp(
        &ADC,
        vrefint_sample,
        //Duration::from_millis(100),
        Duration::from_nanos((dt as f64 * k) as u64),
        adc_temp,
        &LCD
    )));
    //unwrap!(spawner.spawn(tick_periodic(&LOG_FILE)));
}
