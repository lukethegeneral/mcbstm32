#![no_std]
#![no_main]

mod display;
use cortex_m::singleton;
use display::{Lcd, TEXT_BUFFER_LEN};
use embassy_stm32::exti::{self, ExtiInput};

mod log;
use embassy_stm32::dma::{self, AnyChannel, Channel, Priority, ReadableRingBuffer, Transfer};
use embassy_stm32::rcc::mux::ClockMux;
use embassy_stm32::timer::input_capture::{CapturePin, InputCapture};
use embassy_sync::waitqueue::AtomicWaker;
//use embassy_stm32::timer::low_level::Timer;
use log::LogFile;
use static_cell::{ConstStaticCell, StaticCell};
use stm32_metapac::bdma::{regs, Dma};

use core::fmt::{write, Write};
use core::task::{self, Waker};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::{String, Vec};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, RxDma, SampleTime, Temperature, Vref};
use embassy_stm32::gpio::{Flex, Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
//use embassy_stm32::pac;
use embassy_stm32::peripherals::{ADC1, DMA1, DMA1_CH1, DMA1_CH2, I2C1, TIM1};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::{adc, bind_interrupts, i2c, peripherals, spi, Config, Peripheral};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use {defmt_rtt as _, panic_probe as _, stm32_metapac as pac};

bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    //DMA1_CHANNEL1 => dyn dma::ChannelInterrupt<peripherals::DMA1_CH1>;
});

const LOG_FILE_NAME: &str = "RPM_DATA.bin";

static SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

type AdcType = Mutex<ThreadModeRawMutex, Option<Adc<'static, ADC1>>>;
static ADC: AdcType = Mutex::new(None);

type DmaCh1Type = Mutex<ThreadModeRawMutex, Option<DMA1_CH1>>;
static DMA1_CH_1: DmaCh1Type = Mutex::new(None);

type LcdType = Mutex<ThreadModeRawMutex, Option<Lcd>>;
static LCD: LcdType = Mutex::new(None);

type LogFileType = Mutex<ThreadModeRawMutex, Option<LogFile>>;
static LOG_FILE: LogFileType = Mutex::new(None);

fn convert_to_millivolts(vrefint_sample: u16) -> impl Fn(u16) -> u16 {
    let convert_to_millivolts = move |sample| {
        // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
        // 5.3.4 Embedded reference voltage
        const VREFINT_MV: u32 = 1200; // mV

        //    info!("Vrefint_sample[ctm]: {} - {}", vrefint_sample, sample);
        if vrefint_sample > 0 {
            (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
        } else {
            0
        }
    };

    convert_to_millivolts
}
fn from_u16(from: &mut [u16]) -> &[u8] {
    let ptr: *const [u8] = from as *const [u16] as _;
    let len = from.len().checked_mul(2).unwrap();

    if len > 0 && from[0].to_ne_bytes() != from[0].to_be_bytes() {
        for byte in from.iter_mut() {
            *byte = u16::from_ne_bytes((*byte).to_be_bytes());
        }
    }

    unsafe { core::slice::from_raw_parts(ptr as _, len) }
}

#[embassy_executor::task]
// Task that ticks periodically
async fn tick_periodic(log_file: &'static LogFileType) -> ! {
    let mut counter: u32 = 0;
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut tic = Instant::now();
    loop {
        // SIGNAL.signal(counter);
        // counter = counter.wrapping_add(1);

        let toc = Instant::now();
        let elapsed = (toc - tic).as_millis();
        info!("tick! {} {}", counter, elapsed);
        let mut text: String<15> = String::new();
        //let _ = write(&mut text, format_args!("\n[{}][{}]\n", counter, elapsed));
        core::write!(&mut text, "\n[{}][{}]\n", counter, elapsed).unwrap();

        {
            let mut log_file_unlocked = log_file.lock().await;
            if let Some(log_file_ref) = log_file_unlocked.as_mut() {
                //log_file_ref.log_data(text.as_bytes()).await;
                log_file_ref.write(text.as_bytes()).await;
                log_file_ref.flush().await;
            }
        }
        counter += 1;
        //Timer::after(Duration::from_millis(1000)).await; // 1 second
        tic = toc;
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn adc_data(
    adc: &'static AdcType,
    mut pin: peripherals::PA1,
    vrefint_sample: u16,
    mut adc_temp: Temperature,
    lcd: &'static LcdType,
    log_file: &'static LogFileType,
    delay: Duration,
) {
    let convert_to_celcius = |sample| {
        // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
        // Temperature sensor characteristics
        const V25: i32 = 1430; // mV
        const AVG_SLOPE: f32 = 4.3; // mV/C

        let sample_mv = convert_to_millivolts(vrefint_sample)(sample) as i32;

        (sample_mv - V25) as f32 / AVG_SLOPE + 25.0
        //((V25 - sample_mv) as f32 / AVG_SLOPE) + 25.0
    };

    loop {
        {
            let mut adc_unlocked = adc.lock().await;
            // Read internal temperature
            if let Some(adc_ref) = adc_unlocked.as_mut() {
                let v = adc_ref.read(&mut adc_temp).await;
                let celcius = convert_to_celcius(v);
                info!("Internal temp: {=u16} ({} C)", v, celcius);
                let v = adc_ref.read(&mut pin).await;
                let mv = convert_to_millivolts(vrefint_sample)(v);
                info!("Volt--> {} - {} mV", v, mv);

                // Write to LCD
                let mut text_lcd_line_1: String<TEXT_BUFFER_LEN> = String::new();
                let _ = write(&mut text_lcd_line_1, format_args!("C: {:.2}", celcius));

                let mut text_lcd_line_2: String<TEXT_BUFFER_LEN> = String::new();
                let _ = write(&mut text_lcd_line_2, format_args!["mV: {}", mv]);

                let mut lcd_unlocked = lcd.lock().await;
                if let Some(lcd_ref) = lcd_unlocked.as_mut() {
                    lcd_ref.text_buffer[0] = text_lcd_line_1;
                    lcd_ref.text_buffer[1] = text_lcd_line_2;
                    lcd_ref.display_text().await;
                }

                // Write to file
                let mut text_log: String<5> = String::new();
                let _ = write(&mut text_log, format_args!["{},", mv]);
                {
                    let mut log_file_unlocked = log_file.lock().await;
                    if let Some(log_file_ref) = log_file_unlocked.as_mut() {
                        //log_file_ref.log_data(&text_log.as_bytes()).await;
                        log_file_ref.write(&text_log.as_bytes()).await;
                    }
                }
            }
        }
        Timer::after(delay).await;
        //ticker.next().await;
    }
}

#[embassy_executor::task]
async fn temp(
    adc: &'static AdcType,
    vrefint_sample: u16,
    delay: Duration,
    mut adc_temp: Temperature,
    lcd: &'static LcdType,
) {
    //let mut ticker = Ticker::every(delay);

    let convert_to_celcius = |sample| {
        // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
        // Temperature sensor characteristics
        const V25: i32 = 1430; // mV
        const AVG_SLOPE: f32 = 4.3; // mV/C

        let sample_mv = convert_to_millivolts(vrefint_sample)(sample) as i32;

        (sample_mv - V25) as f32 / AVG_SLOPE + 25.0
        //(V25 - sample_mv) as f32 / AVG_SLOPE + 25.0
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

//    let adc_dma_transfer = |adc_dma_buf| unsafe {
fn adc_dma_transfer(
    dma_ch: &'static DMA1_CH1,
    adc_dma_buf: &'static mut [u16],
) -> Transfer<'static> {
    unsafe {
        //let dma_ch = dma_ch.clone_unchecked();
        //let dma_ch = p.DMA1_CH1.clone_unchecked();
        //let dma_ch = embassy_stm32::Peripheral::clone_unchecked(&p.DMA1_CH1);
        //let req = embassy_stm32::adc::RxDma::request(&dma_ch);
        let req = embassy_stm32::adc::RxDma::request(dma_ch);
        let mut options = embassy_stm32::dma::TransferOptions::default();

        //options.circular = true;

        // dma_pac.ch(0).cr().modify(|w| w.set_msize(0b01.into()));
        // dma_pac.ch(0).cr().modify(|w| w.set_psize(0b01.into()));
        // dma_pac.ch(0).ndtr().modify(|w| w.set_ndt(3));
        // dma_pac.ch(0).cr().modify(|w| w.set_circ(true));
        // dma_pac.ch(0).cr().modify(|w| w.set_en(true));

        let dma_pac = embassy_stm32::pac::DMA1;
        let adc_pac = embassy_stm32::pac::ADC1;
        /*
        info!(
            "[before] mem2mem {} circ {} msize {} psize {} dir {} en {} ndt {}",
            dma_pac.ch(0).cr().read().mem2mem() as u8,
            dma_pac.ch(0).cr().read().circ() as u8,
            dma_pac.ch(0).cr().read().msize().to_bits(),
            dma_pac.ch(0).cr().read().psize().to_bits(),
            dma_pac.ch(0).cr().read().dir().to_bits(),
            dma_pac.ch(0).cr().read().en() as u8,
            dma_pac.ch(0).ndtr().read().ndt(),
        );
        */
        info!(
            "ADC befor: sqr3_sq0 {} sqr3_sq1 {} sqr3_sq2 {}",
            adc_pac.sqr3().read().sq(0),
            adc_pac.sqr3().read().sq(1),
            adc_pac.sqr3().read().sq(2),
        );
        adc_pac.sqr3().modify(|w| w.set_sq(0, 1));
        adc_pac.sqr3().modify(|w| w.set_sq(1, 16));
        adc_pac.sqr3().modify(|w| w.set_sq(2, 17));
        info!(
            "ADC after: sqr3_sq0 {} sqr3_sq1 {} sqr3_sq2 {}",
            adc_pac.sqr3().read().sq(0),
            adc_pac.sqr3().read().sq(1),
            adc_pac.sqr3().read().sq(2),
        );

        let dma_transf = Transfer::new_read(
            dma_ch.clone_unchecked(),
            req,
            pac::ADC1.dr().as_ptr() as *mut u16, //0x40012400
            //adc_pac.dr().as_ptr() as *mut u16,
            adc_dma_buf,
            options,
        );

        dma_transf
    }
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    complete_count: core::sync::atomic::AtomicUsize,
}
impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        complete_count: core::sync::atomic::AtomicUsize::new(0),
    };
}

struct DmaTransfer<'a> {
    channel: embassy_stm32::PeripheralRef<'a, AnyChannel>,
}
impl<'a> DmaTransfer<'a> {
    fn is_running() -> bool {
        let dma_pac = embassy_stm32::pac::DMA1;

        if dma_pac.ch(0).ndtr().read().ndt() == 0 {
            true
        } else {
            false
        }
    }
}

impl<'a> core::future::Future for DmaTransfer<'a> {
    type Output = ();
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut task::Context<'_>,
    ) -> task::Poll<Self::Output> {
        // let state: &ChannelState = &STATE[self.channel.id as usize];
        let state: &ChannelState = &ChannelState::NEW;
        //let x = AtomicWaker::new();

        state.waker.register(cx.waker());

        if DmaTransfer::<'a>::is_running() {
            task::Poll::Pending
        } else {
            task::Poll::Ready(())
        }
    }
}

#[embassy_executor::task]
async fn blink(mut sensor: ExtiInput<'static>, mut led: Output<'static>) {
    loop {
        sensor.wait_for_any_edge().await;
        //sensor.wait_for_falling_edge().await;
        if sensor.is_low() {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}

#[embassy_executor::task]
async fn dma_transfer(
    dma_ch: &'static DMA1_CH1,
    dma_transfer_closure: &'static dyn Fn(
        &'static DMA1_CH1,
        &'static mut [u16],
    ) -> Transfer<'static>,
) {
    //let dma_pac = embassy_stm32::pac::DMA1;
    let adc_pac = embassy_stm32::pac::ADC1;

    // set the GPIO pin to analog mode
    // let dma_gpio_a = embassy_stm32::pac::GPIOA;
    // dma_gpio_a.cr(1).modify(|w| w.set_mode(0, 0b00.into()));
    // TODO: this may not be necessary
    //let mut pa1 = Flex::new(p.PA1);
    //pa1.set_as_analog();

    // Software trigger
    adc_pac.cr2().modify(|w| w.set_extsel(0b111.into()));

    adc_pac.cr1().modify(|w| {
        w.set_scan(true); // Scan mode
                          // w.set_eocie(true); // Interrupt enable for EOC
    });

    adc_pac.cr2().modify(|w| {
        w.set_dma(true); // Direct memory access mode (for single ADC mode)
        w.set_cont(true); // Continuous conversion
    });

    // Enable vref & temp
    adc_pac.cr2().modify(|w| w.set_tsvrefe(true));

    // Set length to 3 conversions
    adc_pac.sqr1().modify(|w| w.set_l(2)); // 3 conversion.

    // Set sample sequence. Assign channels to conversion
    const PIN_CHANNEL: u8 = 0x02;
    adc_pac.sqr3().modify(|w| w.set_sq(0, 16));
    adc_pac.sqr3().modify(|w| w.set_sq(1, 17));
    adc_pac.sqr3().modify(|w| w.set_sq(2, PIN_CHANNEL));

    // Set sample times
    adc_pac
        .smpr2()
        .modify(|w| w.set_smp(PIN_CHANNEL as usize, adc::SampleTime::CYCLES41_5));
    adc_pac
        .smpr1()
        .modify(|w| w.set_smp(6 as usize, adc::SampleTime::CYCLES239_5));
    adc_pac
        .smpr1()
        .modify(|w| w.set_smp(7 as usize, adc::SampleTime::CYCLES239_5));

    info!(
        "ADC: sqr1_l {}, sqr3 {} smp_6: {}, smp_7: {}",
        adc_pac.sqr1().read().l(),
        adc_pac.sqr3().read().0.to_be_bytes(),
        adc_pac.smpr1().read().smp(6).to_bits(),
        adc_pac.smpr1().read().smp(7).to_bits(),
    );

    // Set DMA
    let dma_pac = embassy_stm32::pac::DMA1;

    dma_pac.ch(0).cr().modify(|w| {
        w.set_msize(0b01.into()); //16 bits
        w.set_psize(0b01.into()); //16 bits
        w.set_minc(true);
        w.set_circ(true);
    });

    // Set ADC address
    dma_pac
        .ch(0)
        .par()
        .write_value(adc_pac.dr().as_ptr() as u32);

    const NUM_CHANNELS: usize = 3;
    const NUM_SAMPLES: usize = 100;
    let adc_buf: [u16; NUM_SAMPLES * NUM_CHANNELS] = [0u16; { NUM_SAMPLES * NUM_CHANNELS }];
    dma_pac.ch(0).mar().write_value(adc_buf.as_ptr() as u32);

    dma_pac
        .ch(0)
        .ndtr()
        .modify(|w| w.set_ndt((NUM_CHANNELS * NUM_SAMPLES) as u16));

    // Enable DMA channel
    dma_pac.ch(0).cr().modify(|w| w.set_en(true));

    // Power up ADC
    adc_pac.cr2().modify(|w| w.set_adon(true));

    // Wait a bit
    Timer::after(Duration::from_millis(100)).await;

    // Set adon to start conversion
    adc_pac.cr2().modify(|w| w.set_adon(true));
    adc_pac.cr2().modify(|w| w.set_swstart(true));

    //const NUM_CHANNELS: usize = 3;
    //const NUM_SAMPLES: usize = 100;
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        info!(
            "[loop] minc {} circ {} msize {} psize {} dir {} en {} ndt {}",
            dma_pac.ch(0).cr().read().minc() as u8,
            dma_pac.ch(0).cr().read().circ() as u8,
            dma_pac.ch(0).cr().read().msize().to_bits(),
            dma_pac.ch(0).cr().read().psize().to_bits(),
            dma_pac.ch(0).cr().read().dir().to_bits(),
            dma_pac.ch(0).cr().read().en() as u8,
            dma_pac.ch(0).ndtr().read().ndt(),
        );

        /*
        static mut ADC_BUF: [u16; NUM_SAMPLES * NUM_CHANNELS] =
            [0u16; { NUM_SAMPLES * NUM_CHANNELS }];
        let adc_buf = unsafe { &mut ADC_BUF[..] };

        let adc_transfer = dma_transfer_closure(dma_ch, adc_buf);
        // wait for all of the samples to be taken
        adc_transfer.await;

        let adc_buf = unsafe { &ADC_BUF[..] };
        */
        info!("DMA transfer: {:?}", adc_buf[..NUM_CHANNELS * 3]);
        //info!("DMA transfer: {:?}", adc_buf[..]);

        let convert_to_celcius = |vrefint_sample, v_sense| {
            // From http://www.st.com/resource/en/datasheet/CD00161566.pdf
            // Temperature sensor characteristics
            const V25: i32 = 1430; // mV
            const AVG_SLOPE: f32 = 4.3; // mV/C

            let sample_mv = convert_to_millivolts(vrefint_sample)(v_sense) as i32;

            //((sample_mv - V25) as f32 / AVG_SLOPE + 25.0) as u32
            (((V25 - sample_mv) as f32 / AVG_SLOPE) + 25.0) as u32
        };

        let celcius = convert_to_celcius(adc_buf[2], adc_buf[1]);
        let mv = adc_buf[0];
        // Write to LCD
        let mut text_lcd_line_1: String<TEXT_BUFFER_LEN> = String::new();
        core::write!(&mut text_lcd_line_1, "C: {:.2}", celcius).unwrap();

        let mut text_lcd_line_2: String<TEXT_BUFFER_LEN> = String::new();
        core::write!(&mut text_lcd_line_2, "mV: {}", mv).unwrap();

        {
            let lcd_unlocked = &mut LCD.lock().await;
            if let Some(lcd_ref) = lcd_unlocked.as_mut() {
                lcd_ref.text_buffer[0] = text_lcd_line_1;
                lcd_ref.text_buffer[1] = text_lcd_line_2;
                lcd_ref.display_text().await;
            }
        }

        // Write to file
        {
            let log_file_unlocked = &mut LOG_FILE.lock().await;
            if let Some(log_file_ref) = log_file_unlocked.as_mut() {
                log_file_ref.log_data(bytemuck::cast_slice(&adc_buf)).await;
            }
        }
        ticker.next().await;
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
) {
    //let mut ticker = Ticker::every(delay);
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

                let mut text_log: String<5> = String::new();
                let _ = write(&mut text_log, format_args!["{},", mv]);
                {
                    let mut log_file_unlocked = log_file.lock().await;
                    if let Some(log_file_ref) = log_file_unlocked.as_mut() {
                        //log_file_ref.log_data(&text_log.as_bytes()).await;
                        log_file_ref.write(&text_log.as_bytes()).await;
                    }
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
        //config.rcc.ls = LsConfig::default();
        config.rcc.ls = LsConfig {
            rtc: RtcClockSource::LSE,
            lsi: false,
            lse: Some(LseConfig {
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator(LseDrive::MediumLow),
            }),
        };
        config.rcc.hse = Some(Hse {
            freq: mhz(8),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.mux = ClockMux::default();
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.adc_pre = ADCPrescaler::DIV6;

        //config.bdma_interrupt_priority = embassy_stm32::interrupt::Priority::P1;
        //config.enable_debug_during_sleep = true;
    }
    let p = embassy_stm32::init(config);
    //let p = embassy_stm32::init(Default::default());

    let rcc_pac = embassy_stm32::pac::RCC;
    //rcc_pac.cr().write(|reg| reg.set_csson(true));
    info!(
        "RCC:\n b{:b}\n b{:b}\n b{:b}\n b{:b}\n {:b}\n {:b}\n {}\n",
        rcc_pac.cfgr().read().0.to_le(),
        rcc_pac.cfgr().read().0.to_be_bytes(),
        rcc_pac.cfgr().read().pllsrc().to_bits(),
        rcc_pac.cr().read().0.to_le(),
        rcc_pac.cr().read().hseon(),
        rcc_pac.cr().read().hserdy(),
        rcc_pac.cr().read().csson(),
    );

    info!("Program start!");
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
    //let spi_dev = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let spi_dev = ExclusiveDevice::new(spi, cs, embassy_time::Delay).unwrap();

    // Log file & SD card initialization
    {
        *(LOG_FILE.lock().await) = Some(LogFile::new(spi_dev, LOG_FILE_NAME));
    }

    // ADC initialization
    let mut adc = Adc::new(p.ADC1);
    let adc_pac = embassy_stm32::pac::ADC1;

    /*
        adc.set_sample_time(SampleTime::CYCLES41_5);

        //Duration::from_millis(1000);
        //let adc_pac = pac::ADC1;
        info!("before smpr1= {}", adc_pac.smpr1().read().0.to_be_bytes());
        adc_pac
            .smpr1()
            .modify(|reg| reg.set_smp(6, SampleTime::CYCLES41_5));
        adc_pac
            .smpr1()
            .modify(|reg| reg.set_smp(7, SampleTime::CYCLES41_5));
        info!(
            "smp_6: {}, smp_7: {}",
            adc_pac.smpr1().read().smp(6).to_bits(),
            adc_pac.smpr1().read().smp(7).to_bits(),
        );
        let smpr1 = adc_pac.smpr1().read().0.to_be_bytes();
        let sr = adc_pac.sr().read().0.to_be_bytes();
        let cr1 = adc_pac.cr1().read().0.to_be_bytes();
        let cr2 = adc_pac.cr2().read().0.to_be_bytes();
        let dma = adc_pac.cr2().read().dma();
        info!(
            "ADC:\n smpr1 = {}\n sr = {}\n cr1 = {}\n cr2 = {}\n dma = {}",
            smpr1, sr, cr1, cr2, dma
        );
    */

    /////
    /*
    let tim = embassy_stm32::timer::low_level::Timer::new(p.TIM2);
    let timer_registers = tim.regs_gp16();
    timer_registers
        .cr2()
        .modify(|w| w.set_ccds(embassy_stm32::pac::timer::vals::Ccds::ON_UPDATE));
    timer_registers.dier().modify(|w| {
        // Enable update DMA request
        w.set_ude(true);
        // Enable update interrupt request
        w.set_uie(true);
    });

    tim.set_frequency(Hertz(100_000));
    */

    //let dma_ch = unsafe { p.DMA1_CH1.clone_unchecked() };
    let dma_ch = p.DMA1_CH1;
    static DMA_CH: StaticCell<DMA1_CH1> = StaticCell::new();
    let dma_ch = DMA_CH.init(dma_ch);
    //{
    //    *(DMA1_CH_1.lock().await) = Some(dma_ch);
    //}
    //let mut dma_ch_unlocked = DMA1_CH_1.lock().await.as_mut().unwrap();

    //    if let Some(dma_ch_ref) = dma_ch_unlocked.as_mut() {
    //        dma_ch = unsafe { dma_ch_ref.clone_unchecked() };
    //    }
    unwrap!(spawner.spawn(dma_transfer(dma_ch, &adc_dma_transfer)));

    let sensor = ExtiInput::new(p.PC10, p.EXTI10, Pull::Up);
    let led = Output::new(p.PB10, Level::Low, Speed::VeryHigh);

    unwrap!(spawner.spawn(blink(sensor, led)));

    // fake task to keep the program running
    let fut = async {};
    fut.await;

    ///////

    {
        *(ADC.lock().await) = Some(adc);
    }

    let mut vrefint: Vref;
    let mut vrefint_sample: u16 = 0;
    let mut adc_temp: Temperature = Temperature {};

    {
        let mut adc_unlocked = ADC.lock().await;
        if let Some(adc_ref) = adc_unlocked.as_mut() {
            info!("[1] smpr1= {}", adc_pac.smpr1().read().0.to_be_bytes());
            vrefint = adc_ref.enable_vref();
            info!(
                "dma ->\n {} {}",
                adc_pac.cr2().read().dma(),
                adc_pac.cr2().read().adon()
            );
            vrefint_sample = adc_ref.read(&mut vrefint).await;
            info!("[3] smpr1= {}", adc_pac.smpr1().read().0.to_be_bytes());
            let mv = convert_to_millivolts(vrefint_sample)(vrefint_sample);
            info!("Vrefint_sample: {} - {}", vrefint_sample, mv);
            adc_temp = adc_ref.enable_temperature();
        }
    }
    //defmt::panic!();

    let dt = 100 * 1_000_000;
    let k = 1.003;
    // Run tasks
    /*
    unwrap!(spawner.spawn(volt(
        &ADC,
        vrefint_sample,
        Duration::from_nanos(dt),
        //Duration::from_millis(10),
        p.PA1,
        &LCD,
        &LOG_FILE
    )));
    unwrap!(spawner.spawn(temp(
        &ADC,
        vrefint_sample,
        //Duration::from_millis(10),
        Duration::from_nanos((dt as f64 * k) as u64),
        adc_temp,
        &LCD
    )));
    */
    //    unwrap!(spawner.spawn(tick_periodic(&LOG_FILE)));
    /*
    unwrap!(spawner.spawn(adc_data(
        &ADC,
        p.PA1,
        vrefint_sample,
        adc_temp,
        &LCD,
        &LOG_FILE,
        Duration::from_nanos(dt)
    )));
    */
}
