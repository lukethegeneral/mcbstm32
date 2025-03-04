#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, Temperature};
use embassy_stm32::peripherals::{ADC1, PA1};
use embassy_stm32::{adc, bind_interrupts};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
});

type AdcType = Mutex<ThreadModeRawMutex, Option<Adc<'static, ADC1>>>;
static ADC: AdcType = Mutex::new(None);

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
        info!("temp sample_mv={}", sample_mv);

        (sample_mv - V25) as f32 / AVG_SLOPE + 25.0
    };

    loop {
        {
            let mut adc_unlocked = adc.lock().await;
            // Read internal temperature
            if let Some(adc_ref) = adc_unlocked.as_mut() {
                let v = adc_ref.read(&mut adc_temp).await;
                let celcius = convert_to_celcius(v);
                //info!("Internal temp: {=u16} ({} C)", v, celcius);
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
                info!(
                    "Volt--> {} - {} mV",
                    v,
                    convert_to_millivolts(vrefint_sample)(v)
                );
            }
        }
        Timer::after(delay).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let adc = Adc::new(p.ADC1);
    {
        *(ADC.lock().await) = Some(adc);
    }

    Timer::after_millis(300).await;

    let mut vrefint = ADC.lock().await.as_mut().unwrap().enable_vref();
    let vrefint_sample = ADC.lock().await.as_mut().unwrap().read(&mut vrefint).await;
    let adc_temp = ADC.lock().await.as_mut().unwrap().enable_temperature();

    Timer::after_millis(300).await;

    unwrap!(spawner.spawn(temp(
        &ADC,
        vrefint_sample,
        Duration::from_millis(100),
        adc_temp
    )));
    unwrap!(spawner.spawn(volt(
        &ADC,
        vrefint_sample,
        Duration::from_millis(100),
        p.PA1
    )));
}
