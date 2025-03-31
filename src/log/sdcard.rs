use defmt::*;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_time::Delay;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::sdcard::SdCard;
use embedded_sdmmc::{File, VolumeManager};
use static_cell::StaticCell;

// Dummy time source for SD card
pub struct DummyTimesource();

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
// Log file
pub struct LogFile {
    pub log_file: File<
        'static,
        //SdCard<ExclusiveDevice<Spi<'static, Async>, Output<'static>, NoDelay>, Delay>,
        SdCard<ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>, Delay>,
        DummyTimesource,
        4,
        4,
        1,
    >,
}

impl LogFile {
    pub fn new(
        //spi_dev: ExclusiveDevice<Spi<'static, Async>, Output<'static>, NoDelay>,
        spi_dev: ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>,
        file_name: &str,
    ) -> Self {
        let sdcard = SdCard::new(spi_dev, Delay);
        info!("Card size is {} bytes", sdcard.num_bytes().unwrap());

        // Now that the card is initialized, the SPI clock can go faster
        let mut spi_cfg = spi::Config::default();
        spi_cfg.frequency = Hertz(36_000_000); // 16MHz
        sdcard
            .spi(|dev| dev.bus_mut().set_config(&spi_cfg))
            .unwrap();

        // Now let's look for volumes (also known as partitions) on our block device.
        // To do this we need a Volume Manager. It will take ownership of the block device.
        let volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());

        static VOL: StaticCell<
            VolumeManager<
                //SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, NoDelay>, Delay>,
                SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, Delay>, Delay>,
                DummyTimesource,
            >,
        > = StaticCell::new();
        let volume_mgr = VOL.init(volume_mgr);

        // Try and access Volume 0 (i.e. the first partition).
        // The volume object holds information about the filesystem on that volume.
        let volume0 = volume_mgr
            .open_volume(embedded_sdmmc::VolumeIdx(0))
            .unwrap();
        info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

        static VOL0: StaticCell<
            embedded_sdmmc::Volume<
                '_,
                //SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, NoDelay>, Delay>,
                SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, Delay>, Delay>,
                DummyTimesource,
                4,
                4,
                1,
            >,
        > = StaticCell::new();
        let volume0 = VOL0.init(volume0);

        // Open the root directory (mutably borrows from the volume).
        let root_dir = volume0.open_root_dir().unwrap();

        static ROOT: StaticCell<
            embedded_sdmmc::Directory<
                '_,
                //SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, NoDelay>, Delay>,
                SdCard<ExclusiveDevice<Spi<'_, Async>, Output<'_>, Delay>, Delay>,
                DummyTimesource,
                4,
                4,
                1,
            >,
        > = StaticCell::new();
        let root_dir = ROOT.init(root_dir);

        let log_file = root_dir
            .open_file_in_dir(file_name, embedded_sdmmc::Mode::ReadWriteCreateOrTruncate)
            .unwrap();

        Self { log_file }
    }

    pub async fn log_data(&mut self, data: &[u8]) {
        self.log_file
            .write(data)
            .expect("Error writing to log file");
        self.log_file.flush().unwrap();
    }

    pub async fn write(&mut self, data: &[u8]) {
        self.log_file.write(data).unwrap();
    }

    pub async fn flush(&mut self) {
        self.log_file.flush().unwrap();
    }
}
